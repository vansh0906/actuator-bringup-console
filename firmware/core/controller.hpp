// The drive: state machine, cascaded control loops, fault detection and the
// command watchdog. It runs at 1 kHz and only uses what real firmware has:
// encoder counts, the current-sensor reading, bus voltage and temperature.
//
//   position loop (P)  ->  velocity loop (PI + anti-windup)  ->  current
//
// Frames:
//   joint position = encoder angle / gear - encoder_offset
//   joint current  = motor_polarity * (sensor - current_offset)
//   terminal current command = motor_polarity * joint current command
#pragma once

#include <cstdint>

#include "params.hpp"
#include "protocol.hpp"

namespace act {

struct Sensors {
    int64_t encoder_counts = 0;
    double current_raw = 0.0;  // A, terminal polarity
    double bus_voltage = 0.0;  // V
    double motor_temp = 0.0;   // degC
};

class Drive {
public:
    static constexpr uint32_t kBootMs = 200;
    static constexpr double kDt = 0.001;

    Drive(ParamTable& params, double gear, double encoder_cpr);

    void on_command(const proto::Command& c, uint32_t now_ms);
    void on_estop();
    // One 1 kHz control step. Returns the terminal current command (A).
    double update(const Sensors& s, uint32_t now_ms);

    void mark_homed() { homed_ = true; }
    void clear_homed() { homed_ = false; }

    proto::State state() const { return state_; }
    proto::Mode mode() const { return mode_; }
    bool enabled() const { return state_ == proto::State::Enabled; }
    bool homed() const { return homed_; }
    double position() const { return position_; }
    double velocity() const { return velocity_; }
    double current() const { return current_; }
    double bus_voltage() const { return bus_voltage_; }
    double motor_temp() const { return motor_temp_; }
    double current_command() const { return current_cmd_; }
    uint16_t faults() const { return faults_; }
    // Fault bits set since the last call (for EMCY frames).
    uint16_t take_new_faults();

private:
    void raise(uint16_t bits);
    void reset_loops();

    ParamTable& params_;
    double rad_per_count_;

    proto::State state_ = proto::State::Boot;
    proto::Mode mode_ = proto::Mode::Idle;
    proto::Mode cmd_mode_ = proto::Mode::Idle;
    bool cmd_enable_ = false;
    double cmd_setpoint_ = 0.0;
    bool have_command_ = false;
    uint32_t last_cmd_ms_ = 0;
    uint32_t start_ms_ = 0;
    bool started_ = false;

    bool have_prev_ = false;
    double prev_raw_ = 0.0;
    double position_ = 0.0;
    double velocity_ = 0.0;
    double current_ = 0.0;
    double bus_voltage_ = 0.0;
    double motor_temp_ = 0.0;
    double integral_ = 0.0;
    double current_cmd_ = 0.0;  // joint frame
    bool homed_ = false;

    uint16_t faults_ = 0;
    uint16_t new_faults_ = 0;
    int uv_count_ = 0;
    int oc_count_ = 0;
    int os_count_ = 0;
    int fe_count_ = 0;
    int stall_count_ = 0;
};

}  // namespace act
