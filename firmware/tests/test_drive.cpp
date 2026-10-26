#include <algorithm>
#include <cmath>

#include "test_common.hpp"

using namespace act;
using namespace act::proto;

namespace {
void clear_faults(Harness& h) {
    h.send_commands = true;
    h.cmd.enable = false;
    h.cmd.clear_faults = true;
    h.run(30);
    h.cmd.clear_faults = false;
}
}  // namespace

TEST(Drive, BootsThenDisabled) {
    Harness h;
    h.run(150);
    EXPECT_TRUE(h.node.drive().state() == State::Boot);
    h.run(100);
    EXPECT_TRUE(h.node.drive().state() == State::Disabled);
}

TEST(Drive, VelocityLoopTracks) {
    Harness h;
    h.run(300);
    h.enable(Mode::Velocity, 1.0);
    h.run(1000);
    EXPECT_TRUE(h.node.drive().state() == State::Enabled);
    double sq = 0.0;
    for (int i = 0; i < 500; ++i) {
        h.run(1);
        double e = 1.0 - h.node.plant().true_velocity();
        sq += e * e;
    }
    EXPECT_LT(std::sqrt(sq / 500.0), 0.05);
    EXPECT_NEAR(h.node.drive().velocity(), 1.0, 0.05);
}

TEST(Drive, PositionStepNoBigOvershoot) {
    Harness h;
    h.run(300);
    h.enable(Mode::Position, 0.0);
    h.run(500);
    const double start = h.node.drive().position();
    h.cmd.setpoint = start + 0.5;
    double peak = start;
    for (int i = 0; i < 2000; ++i) {
        h.run(1);
        peak = std::max(peak, h.node.drive().position());
    }
    const double overshoot = (peak - (start + 0.5)) / 0.5;
    EXPECT_LT(overshoot, 0.10);
    EXPECT_NEAR(h.node.drive().position(), start + 0.5, 0.005);
}

TEST(Drive, CommandWatchdogFaults) {
    Harness h;
    h.run(300);
    h.enable(Mode::Velocity, 0.0);
    h.run(500);
    ASSERT_TRUE(h.node.drive().enabled());
    h.send_commands = false;
    const uint32_t last_cmd = (h.node.uptime_ms() - 1) - ((h.node.uptime_ms() - 1) % 10);
    int fault_at = -1;
    for (int i = 0; i < 300 && fault_at < 0; ++i) {
        h.run(1);
        if (h.node.drive().faults() & fault::CMD_TIMEOUT) fault_at = static_cast<int>(h.node.uptime_ms() - last_cmd);
    }
    EXPECT_GE(fault_at, 100);
    EXPECT_LE(fault_at, 112);
    EXPECT_EQ(h.emcy_count(fault::CMD_TIMEOUT), 1);
    EXPECT_EQ(h.node.drive().current_command(), 0.0);
}

TEST(Drive, EstopDisablesWithinOneTick) {
    Harness h;
    h.run(300);
    h.enable(Mode::Velocity, 1.0);
    h.run(500);
    h.node.on_frame(pack_estop(1));
    h.run(1);
    EXPECT_TRUE(h.node.drive().state() == State::Fault);
    EXPECT_TRUE((h.node.drive().faults() & fault::ESTOP) != 0);
    EXPECT_EQ(h.emcy_count(fault::ESTOP), 1);
    clear_faults(h);
    EXPECT_TRUE(h.node.drive().state() == State::Disabled);
}

TEST(Drive, ReversedWiringIsCaughtAndPolarityFixesIt) {
    PlantConfig cfg;
    cfg.wiring_reversed = true;
    Harness h(cfg);
    h.run(300);
    h.enable(Mode::Velocity, 0.5);
    h.run(1500);
    // Positive feedback: the joint runs away until a fault stops it.
    EXPECT_TRUE(h.node.drive().state() == State::Fault);
    EXPECT_TRUE((h.node.drive().faults() & (fault::OVERSPEED | fault::FOLLOWING_ERROR)) != 0);

    clear_faults(h);
    ParamMsg r = h.param(ParamOp::Write, P_MOTOR_POLARITY, -1.0f);
    EXPECT_EQ(static_cast<int>(r.status), static_cast<int>(ParamStatus::Ok));
    h.run(200);  // let the joint coast to a stop
    h.enable(Mode::Velocity, 0.5);
    h.run(1000);
    EXPECT_TRUE(h.node.drive().enabled());
    EXPECT_NEAR(h.node.plant().true_velocity(), 0.5, 0.05);
}

TEST(Drive, JamRaisesFollowingError) {
    Harness h;
    h.run(300);
    h.enable(Mode::Velocity, 1.0);
    h.run(300);
    h.node.plant().inject(InjectKind::Jam, 40.0f);
    h.run(600);
    EXPECT_TRUE((h.node.drive().faults() & fault::FOLLOWING_ERROR) != 0);
}

TEST(Drive, StallAgainstHardStop) {
    Harness h;
    h.run(300);
    // Drive slowly into the stop with a low current limit, as homing does.
    h.param(ParamOp::Write, P_CURRENT_LIMIT, 1.0f);
    h.enable(Mode::Velocity, 0.4);
    h.run(6000);
    EXPECT_TRUE((h.node.drive().faults() & fault::STALL) != 0);
    EXPECT_GT(h.node.plant().true_position(), 2.49);
}

TEST(Drive, OvertempLatchesUntilCool) {
    Harness h;
    h.run(300);
    h.node.plant().inject(InjectKind::Overheat, 200.0f);
    int t = 0;
    while (!(h.node.drive().faults() & fault::OVERTEMP) && t < 30000) {
        h.run(10);
        t += 10;
    }
    ASSERT_TRUE((h.node.drive().faults() & fault::OVERTEMP) != 0);
    h.node.plant().clear_injections();
    clear_faults(h);  // still hot: refused
    EXPECT_TRUE((h.node.drive().faults() & fault::OVERTEMP) != 0);
    h.run(20000);  // cool down
    clear_faults(h);
    EXPECT_EQ(h.node.drive().faults(), 0);
    EXPECT_TRUE(h.node.drive().state() == State::Disabled);
}

TEST(Drive, EncoderGlitchFaultsAndClearsHomed) {
    Harness h;
    h.run(300);
    h.param(ParamOp::Write, P_ENCODER_OFFSET, 0.3f);
    EXPECT_TRUE(h.node.drive().homed());
    h.node.plant().inject(InjectKind::EncoderGlitch, 0.2f);
    h.run(5);
    EXPECT_TRUE((h.node.drive().faults() & fault::ENCODER) != 0);
    EXPECT_FALSE(h.node.drive().homed());
}

TEST(Drive, BadCommandsAreCountedAndIgnored) {
    Harness h;
    h.run(300);
    Command c;
    c.mode = Mode::Velocity;
    c.enable = true;
    c.setpoint = 1.0;
    c.counter = 5;
    CanFrame good = pack_command(1, c);
    CanFrame bad = good;
    bad.data[2] ^= 0x01;
    h.node.on_frame(bad);   // bad CRC
    h.node.on_frame(good);  // accepted
    h.node.on_frame(good);  // same counter again: stale, ignored
    EXPECT_EQ(static_cast<int>(h.node.e2e_errors()), 2);
}

TEST(Drive, CalibrationParamsRefusedWhileEnabled) {
    Harness h;
    h.run(300);
    h.enable(Mode::Velocity, 0.0);
    h.run(100);
    ParamMsg r = h.param(ParamOp::Write, P_ENCODER_OFFSET, 1.0f);
    EXPECT_EQ(static_cast<int>(r.status), static_cast<int>(ParamStatus::NotWhileEnabled));
}

TEST(Drive, SoftLimitsAfterHoming) {
    Harness h;
    h.run(300);
    // Joint starts at true 0.7 rad and the encoder reads 0: set the zero.
    h.param(ParamOp::Write, P_ENCODER_OFFSET, -0.7f);
    h.param(ParamOp::Write, P_POSITION_MAX, 1.5f);
    h.enable(Mode::Velocity, 1.0);
    h.run(3000);
    EXPECT_NEAR(h.node.drive().position(), 1.5, 0.05);
    EXPECT_NEAR(h.node.plant().true_position(), 1.5, 0.05);
}

TEST(Drive, CurrentSensorOffsetIsVisibleWhileDisabled) {
    PlantConfig cfg;
    cfg.current_sensor_offset = 0.12;
    Harness h(cfg);
    h.run(300);
    double sum = 0.0;
    for (int i = 0; i < 1000; ++i) {
        h.run(1);
        sum += h.node.drive().current();
    }
    EXPECT_NEAR(sum / 1000.0, 0.12, 0.005);
    h.param(ParamOp::Write, P_CURRENT_OFFSET, 0.12f);
    sum = 0.0;
    for (int i = 0; i < 1000; ++i) {
        h.run(1);
        sum += h.node.drive().current();
    }
    EXPECT_NEAR(sum / 1000.0, 0.0, 0.005);
}
