// Simulated actuator hardware: a motor with a voltage-limited current loop,
// a 50:1 gearbox, a joint with stick-slip friction and hard stops, a
// motor-side incremental encoder, a noisy current sensor with an offset, a
// thermal model and a 48 V supply that sags under load.
//
// The drive (controller.hpp) only sees what real firmware would see:
// encoder counts, the current-sensor reading, bus voltage and temperature.
// Some properties are hidden on purpose so the bring-up tools have
// something real to find: an unknown current-sensor offset, a joint that
// does not start at zero, and (on one node) reversed motor wiring.
#pragma once

#include <cstdint>
#include <random>

#include "protocol.hpp"

namespace act {

struct PlantConfig {
    double R = 0.5;                  // ohm, winding resistance
    double Kt = 0.08;                // Nm/A (and V/(rad/s)), motor side
    double gear = 50.0;              // motor turns per joint turn
    double J_motor = 2e-5;           // kg m^2, rotor
    double J_load = 0.02;            // kg m^2, joint side
    double b_out = 0.05;             // Nm/(rad/s), joint-side viscous friction
    double coulomb_out = 0.3;        // Nm, joint-side Coulomb friction
    double v_supply = 48.0;          // V
    double r_supply = 0.1;           // ohm, supply + cable resistance
    double tau_current = 0.0005;     // s, closed current-loop response
    double enc_cpr = 16384.0;        // encoder counts per motor revolution
    double stop_lo = -2.5;           // rad, true joint hard stops
    double stop_hi = 2.5;
    double stop_k = 500.0;           // Nm/rad
    double stop_c = 5.0;             // Nm/(rad/s)
    double R_th = 3.0;               // K/W, winding to ambient
    double C_th = 8.0;               // J/K
    double T_ambient = 25.0;         // degC
    double initial_position = 0.7;   // rad, true joint angle at power-up
    bool wiring_reversed = false;    // motor phases swapped
    double current_sensor_offset = 0.0;  // A
    double current_noise_std = 0.01;     // A
    uint32_t seed = 1;
};

class Plant {
public:
    explicit Plant(const PlantConfig& cfg);

    // Advance by dt seconds with the given terminal current command (A).
    void step(double dt, double current_cmd);

    int64_t encoder_counts() const;
    double read_current_sensor();  // A, terminal polarity, offset + noise
    double bus_voltage() const { return v_bus_; }
    double motor_temp() const { return temp_; }

    double true_position() const { return theta_; }  // joint side, rad
    double true_velocity() const { return omega_; }  // joint side, rad/s
    double true_current() const { return current_; }

    void inject(proto::InjectKind kind, float value);
    void clear_injections();
    const PlantConfig& config() const { return cfg_; }

private:
    double gaussian();

    PlantConfig cfg_;
    double theta_;
    double omega_ = 0.0;
    double current_ = 0.0;
    double temp_;
    double v_bus_;
    double encoder_glitch_rad_ = 0.0;
    double extra_heat_w_ = 0.0;
    double supply_override_v_ = 0.0;
    double jam_friction_nm_ = 0.0;
    double extra_noise_a_ = 0.0;
    double cached_dt_ = -1.0;      // exp(-dt/tau) cache: the step size never changes,
    double current_blend_ = 0.0;   // and exp() is slow on an ESP32 without a double FPU
    std::mt19937 rng_;
};

}  // namespace act
