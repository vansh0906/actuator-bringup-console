#include "plant.hpp"

#include <algorithm>
#include <cmath>

namespace act {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kStickVelocity = 1e-3;  // rad/s, below this the joint can stick
}  // namespace

Plant::Plant(const PlantConfig& cfg)
    : cfg_(cfg), theta_(cfg.initial_position), temp_(cfg.T_ambient), v_bus_(cfg.v_supply), rng_(cfg.seed) {}

double Plant::gaussian() {
    // Box-Muller on raw mt19937 output. std::normal_distribution differs
    // between libstdc++ and libc++, which would make the simulator give
    // different numbers on Linux CI and on a Mac.
    double u1 = (static_cast<double>(rng_()) + 0.5) / 4294967296.0;
    double u2 = (static_cast<double>(rng_()) + 0.5) / 4294967296.0;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

void Plant::step(double dt, double current_cmd) {
    const double s = cfg_.wiring_reversed ? -1.0 : 1.0;
    const double J = cfg_.J_motor * cfg_.gear * cfg_.gear + cfg_.J_load;
    const double w_motor = omega_ * cfg_.gear;
    const double back_emf = s * cfg_.Kt * w_motor;

    // Current loop: first-order response, limited by the bus voltage.
    const double i_hi = (v_bus_ - back_emf) / cfg_.R;
    const double i_lo = (-v_bus_ - back_emf) / cfg_.R;
    const double target = std::clamp(current_cmd, std::min(i_lo, i_hi), std::max(i_lo, i_hi));
    if (dt != cached_dt_) {
        cached_dt_ = dt;
        current_blend_ = 1.0 - std::exp(-dt / cfg_.tau_current);
    }
    current_ += (target - current_) * current_blend_;
    current_ = std::clamp(current_, std::min(i_lo, i_hi), std::max(i_lo, i_hi));

    // Joint-side torques.
    double tau = s * cfg_.Kt * current_ * cfg_.gear - cfg_.b_out * omega_;
    if (theta_ < cfg_.stop_lo) tau += -cfg_.stop_k * (theta_ - cfg_.stop_lo) - cfg_.stop_c * omega_;
    if (theta_ > cfg_.stop_hi) tau += -cfg_.stop_k * (theta_ - cfg_.stop_hi) - cfg_.stop_c * omega_;

    // Stick-slip Coulomb friction.
    const double fc = cfg_.coulomb_out + jam_friction_nm_;
    if (std::fabs(omega_) < kStickVelocity && std::fabs(tau) <= fc) {
        omega_ = 0.0;
    } else {
        const double dir = (std::fabs(omega_) >= kStickVelocity) ? (omega_ > 0 ? 1.0 : -1.0) : (tau > 0 ? 1.0 : -1.0);
        const double alpha = (tau - fc * dir) / J;
        const double next = omega_ + alpha * dt;
        // Friction cannot reverse the motion on its own: stop at zero.
        omega_ = (omega_ != 0.0 && (next > 0) != (omega_ > 0) && std::fabs(tau) <= fc) ? 0.0 : next;
    }
    theta_ += omega_ * dt;

    // Thermal model.
    const double p_heat = current_ * current_ * cfg_.R + extra_heat_w_;
    temp_ += dt * (p_heat - (temp_ - cfg_.T_ambient) / cfg_.R_th) / cfg_.C_th;

    // Supply sag from the electrical power drawn.
    const double v_supply = supply_override_v_ > 0.0 ? supply_override_v_ : cfg_.v_supply;
    const double p_elec = (cfg_.R * current_ + back_emf) * current_;
    const double i_bus = p_elec / v_supply;
    v_bus_ = std::clamp(v_supply - cfg_.r_supply * i_bus, 0.0, v_supply + 5.0);
}

int64_t Plant::encoder_counts() const {
    const double motor_turns = (theta_ - cfg_.initial_position + encoder_glitch_rad_) * cfg_.gear / (2.0 * kPi);
    return static_cast<int64_t>(std::floor(motor_turns * cfg_.enc_cpr));
}

double Plant::read_current_sensor() {
    const double std_dev = cfg_.current_noise_std + extra_noise_a_;
    return current_ + cfg_.current_sensor_offset + std_dev * gaussian();
}

void Plant::inject(proto::InjectKind kind, float value) {
    using proto::InjectKind;
    switch (kind) {
        case InjectKind::ClearAll: clear_injections(); break;
        case InjectKind::Overheat: extra_heat_w_ = std::max(0.0f, value); break;
        case InjectKind::Undervoltage: supply_override_v_ = std::max(0.0f, value); break;
        case InjectKind::EncoderGlitch: encoder_glitch_rad_ += value; break;
        case InjectKind::Jam: jam_friction_nm_ = std::max(0.0f, value); break;
        case InjectKind::CurrentNoise: extra_noise_a_ = std::max(0.0f, value); break;
        case InjectKind::DropStatus: break;  // handled by the node
    }
}

void Plant::clear_injections() {
    // An encoder glitch is lost counts: clearing the injection does not
    // give them back, which is why the joint has to be homed again.
    extra_heat_w_ = 0.0;
    supply_override_v_ = 0.0;
    jam_friction_nm_ = 0.0;
    extra_noise_a_ = 0.0;
}

}  // namespace act
