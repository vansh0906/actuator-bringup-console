#include "controller.hpp"

#include <algorithm>
#include <cmath>

namespace act {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxStepRad = 0.05;       // per 1 ms tick = 50 rad/s: not physical, so an encoder fault
constexpr double kVelFilterHz = 150.0;
constexpr int kUndervoltageTicks = 10;
constexpr int kOvercurrentTicks = 5;
constexpr int kOverspeedTicks = 10;
constexpr int kFollowingTicks = 300;
constexpr int kStallTicks = 500;
constexpr double kStallVelocity = 0.05;    // rad/s
constexpr double kOvertempHysteresis = 5.0;
constexpr double kUndervoltageHysteresis = 1.0;

double clampd(double v, double lim) { return std::clamp(v, -lim, lim); }
}  // namespace

Drive::Drive(ParamTable& params, double gear, double encoder_cpr)
    : params_(params), rad_per_count_(2.0 * kPi / (encoder_cpr * gear)) {}

void Drive::raise(uint16_t bits) {
    uint16_t fresh = static_cast<uint16_t>(bits & ~faults_);
    faults_ |= bits;
    new_faults_ |= fresh;
    if (bits & proto::fault::ENCODER) homed_ = false;  // position can no longer be trusted
    if (state_ != proto::State::Boot) state_ = proto::State::Fault;
    current_cmd_ = 0.0;
}

uint16_t Drive::take_new_faults() {
    uint16_t f = new_faults_;
    new_faults_ = 0;
    return f;
}

void Drive::reset_loops() {
    integral_ = 0.0;
    fe_count_ = 0;
    stall_count_ = 0;
    oc_count_ = 0;
    os_count_ = 0;
}

void Drive::on_estop() { raise(proto::fault::ESTOP); }

void Drive::on_command(const proto::Command& c, uint32_t now_ms) {
    have_command_ = true;
    last_cmd_ms_ = now_ms;
    if (c.mode != cmd_mode_) integral_ = 0.0;
    cmd_mode_ = c.mode;
    cmd_enable_ = c.enable;
    cmd_setpoint_ = c.setpoint;

    if (c.clear_faults && !c.enable && faults_ != 0) {
        uint16_t keep = 0;
        if ((faults_ & proto::fault::OVERTEMP) &&
            motor_temp_ > params_.get(P_OVERTEMP_C) - kOvertempHysteresis)
            keep |= proto::fault::OVERTEMP;
        if ((faults_ & proto::fault::UNDERVOLTAGE) &&
            bus_voltage_ < params_.get(P_UNDERVOLTAGE_V) + kUndervoltageHysteresis)
            keep |= proto::fault::UNDERVOLTAGE;
        faults_ = keep;
        if (faults_ == 0 && state_ == proto::State::Fault) {
            state_ = proto::State::Disabled;
            reset_loops();
        }
    }
}

double Drive::update(const Sensors& s, uint32_t now_ms) {
    if (!started_) {
        started_ = true;
        start_ms_ = now_ms;
    }

    // --- Measurements --------------------------------------------------
    const double raw = static_cast<double>(s.encoder_counts) * rad_per_count_;
    if (!have_prev_) {
        prev_raw_ = raw;
        have_prev_ = true;
    }
    const double step = raw - prev_raw_;
    prev_raw_ = raw;
    const bool encoder_jump = std::fabs(step) > kMaxStepRad;
    position_ = raw - params_.get(P_ENCODER_OFFSET);
    if (!encoder_jump) {
        const double alpha = kDt / (kDt + 1.0 / (2.0 * kPi * kVelFilterHz));
        velocity_ += alpha * (step / kDt - velocity_);
    }
    const double polarity = params_.get(P_MOTOR_POLARITY) < 0 ? -1.0 : 1.0;
    current_ = polarity * (s.current_raw - params_.get(P_CURRENT_OFFSET));
    bus_voltage_ = s.bus_voltage;
    motor_temp_ = s.motor_temp;

    // --- Boot ------------------------------------------------------------
    if (state_ == proto::State::Boot) {
        if (now_ms - start_ms_ >= kBootMs) state_ = faults_ ? proto::State::Fault : proto::State::Disabled;
        current_cmd_ = 0.0;
        return 0.0;
    }

    // --- Faults that apply in any state ---------------------------------
    if (encoder_jump) raise(proto::fault::ENCODER);
    if (motor_temp_ > params_.get(P_OVERTEMP_C)) raise(proto::fault::OVERTEMP);
    uv_count_ = bus_voltage_ < params_.get(P_UNDERVOLTAGE_V) ? uv_count_ + 1 : 0;
    if (uv_count_ >= kUndervoltageTicks) raise(proto::fault::UNDERVOLTAGE);

    const double current_limit = params_.get(P_CURRENT_LIMIT);
    const double velocity_limit = params_.get(P_VELOCITY_LIMIT);

    if (state_ == proto::State::Enabled) {
        oc_count_ = std::fabs(current_) > 1.5 * current_limit ? oc_count_ + 1 : 0;
        if (oc_count_ >= kOvercurrentTicks) raise(proto::fault::OVERCURRENT);
        os_count_ = std::fabs(velocity_) > 1.5 * velocity_limit ? os_count_ + 1 : 0;
        if (os_count_ >= kOverspeedTicks) raise(proto::fault::OVERSPEED);
        if (now_ms - last_cmd_ms_ > static_cast<uint32_t>(params_.get(P_CMD_TIMEOUT_MS)))
            raise(proto::fault::CMD_TIMEOUT);
    }

    // --- State transitions -----------------------------------------------
    if (faults_ != 0) state_ = proto::State::Fault;
    if (state_ == proto::State::Disabled && cmd_enable_ && have_command_ &&
        now_ms - last_cmd_ms_ <= static_cast<uint32_t>(params_.get(P_CMD_TIMEOUT_MS))) {
        state_ = proto::State::Enabled;
        reset_loops();
    } else if (state_ == proto::State::Enabled && !cmd_enable_) {
        state_ = proto::State::Disabled;
    }
    mode_ = cmd_mode_;

    if (state_ != proto::State::Enabled) {
        reset_loops();
        current_cmd_ = 0.0;
        return 0.0;
    }

    // --- Control ---------------------------------------------------------
    double i_cmd = 0.0;
    bool velocity_loop = false;
    double vel_sp = 0.0;
    const double pos_min = params_.get(P_POSITION_MIN);
    const double pos_max = params_.get(P_POSITION_MAX);

    switch (mode_) {
        case proto::Mode::Idle:
            integral_ = 0.0;
            break;
        case proto::Mode::Current:
            i_cmd = cmd_setpoint_;
            break;
        case proto::Mode::Velocity:
            vel_sp = clampd(cmd_setpoint_, velocity_limit);
            velocity_loop = true;
            break;
        case proto::Mode::Position: {
            const double sp = homed_ ? std::clamp(cmd_setpoint_, pos_min, pos_max) : cmd_setpoint_;
            vel_sp = clampd(params_.get(P_POS_KP) * (sp - position_), velocity_limit);
            velocity_loop = true;
            break;
        }
    }

    if (velocity_loop) {
        // Soft limits apply once the joint is homed.
        if (homed_ && ((position_ >= pos_max && vel_sp > 0) || (position_ <= pos_min && vel_sp < 0))) vel_sp = 0.0;
        const double err = vel_sp - velocity_;
        fe_count_ = std::fabs(err) > params_.get(P_FOLLOWING_ERROR) ? fe_count_ + 1 : 0;
        if (fe_count_ >= kFollowingTicks) raise(proto::fault::FOLLOWING_ERROR);

        const double kp = params_.get(P_VEL_KP);
        const double ki = params_.get(P_VEL_KI);
        const double p = kp * err;
        const double candidate = integral_ + ki * err * kDt;
        // Anti-windup: integrate only while not saturated, or when the
        // error pulls the output back out of saturation.
        if (std::fabs(p + candidate) < current_limit || (err > 0) != (integral_ > 0)) integral_ = candidate;
        integral_ = clampd(integral_, current_limit);
        i_cmd = p + integral_;
    }

    i_cmd = clampd(i_cmd, current_limit);

    stall_count_ = (std::fabs(current_) >= 0.95 * current_limit && std::fabs(velocity_) < kStallVelocity)
                       ? stall_count_ + 1
                       : 0;
    if (stall_count_ >= kStallTicks) raise(proto::fault::STALL);

    if (state_ != proto::State::Enabled) {  // a fault was raised this tick
        current_cmd_ = 0.0;
        return 0.0;
    }
    current_cmd_ = i_cmd;
    return polarity * i_cmd;
}

}  // namespace act
