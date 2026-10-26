#include <cstring>

#include "slcan.hpp"
#include "test_common.hpp"

using namespace act;
using namespace act::proto;

TEST(Protocol, Crc8J1850MatchesStandardCheckValue) {
    // The published check value of CRC-8/SAE-J1850 for "123456789" is 0x4B.
    const char* s = "123456789";
    EXPECT_EQ(crc8_j1850(reinterpret_cast<const uint8_t*>(s), 9), 0x4B);
}

TEST(Protocol, CommandRoundTripAndE2EChecks) {
    Command c;
    c.mode = Mode::Position;
    c.enable = true;
    c.setpoint = -1.2345;
    c.counter = 77;
    CanFrame f = pack_command(3, c);
    EXPECT_EQ(f.id, 0x203u);

    Command out;
    ASSERT_TRUE(unpack_command(f, out) == CmdCheck::Ok);
    EXPECT_TRUE(out.mode == Mode::Position);
    EXPECT_TRUE(out.enable);
    EXPECT_FALSE(out.clear_faults);
    EXPECT_NEAR(out.setpoint, -1.2345, 1e-4);
    EXPECT_EQ(static_cast<int>(out.counter), 77);

    // Any single bit flip in the payload is caught by the CRC.
    for (int byte = 0; byte < 7; ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            CanFrame bad = f;
            bad.data[byte] ^= static_cast<uint8_t>(1u << bit);
            EXPECT_TRUE(unpack_command(bad, out) == CmdCheck::BadCrc);
        }
    }
    // A frame delivered under another node's ID is caught too (the CRC covers the ID).
    CanFrame wrong_id = f;
    wrong_id.id = 0x204;
    EXPECT_TRUE(unpack_command(wrong_id, out) == CmdCheck::BadCrc);
    CanFrame short_frame = f;
    short_frame.dlc = 7;
    EXPECT_TRUE(unpack_command(short_frame, out) == CmdCheck::BadLength);
}

TEST(Protocol, StatusScalingAndSaturation) {
    StatusFast s;
    s.position = 12.3456;
    s.velocity = -40.0;  // beyond the int16 range at 0.001 rad/s: saturates
    s.current = 1.234;
    CanFrame f = pack_status_fast(1, s);
    EXPECT_EQ(f.id, 0x181u);
    StatusFast out;
    ASSERT_TRUE(unpack_status_fast(f, out));
    EXPECT_NEAR(out.position, 12.3456, 1e-4);
    EXPECT_NEAR(out.velocity, -32.768, 1e-9);
    EXPECT_NEAR(out.current, 1.234, 1e-3);

    StatusSlow ss;
    ss.bus_voltage = 47.83;
    ss.motor_temp = -5.5;
    ss.fault_flags = fault::OVERTEMP | fault::STALL;
    ss.state = State::Fault;
    ss.mode = Mode::Velocity;
    ss.e2e_errors = 9;
    StatusSlow ss_out;
    ASSERT_TRUE(unpack_status_slow(pack_status_slow(2, ss), ss_out));
    EXPECT_NEAR(ss_out.bus_voltage, 47.83, 0.005);
    EXPECT_NEAR(ss_out.motor_temp, -5.5, 0.05);
    EXPECT_EQ(ss_out.fault_flags, ss.fault_flags);
    EXPECT_TRUE(ss_out.state == State::Fault);
    EXPECT_TRUE(ss_out.mode == Mode::Velocity);
    EXPECT_EQ(static_cast<int>(ss_out.e2e_errors), 9);
}

TEST(Protocol, ParamFloatAndEstop) {
    ParamMsg p;
    p.opcode = static_cast<uint8_t>(ParamOp::Write);
    p.index = 5;
    p.value = 3.14159f;
    ParamMsg out;
    ASSERT_TRUE(unpack_param(pack_param(FC_PARAM_REQ, 1, p), out));
    EXPECT_EQ(static_cast<int>(out.index), 5);
    EXPECT_EQ(out.value, 3.14159f);

    CanFrame e = pack_estop(4);
    EXPECT_EQ(e.id, 0x080u);
    EXPECT_TRUE(is_valid_estop(e));
    e.data[1] ^= 1;
    EXPECT_FALSE(is_valid_estop(e));
}

TEST(Slcan, FormatParseRoundTrip) {
    CanFrame f;
    f.id = 0x181;
    f.dlc = 8;
    const uint8_t data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x11, 0x22, 0x33};
    std::memcpy(f.data, data, 8);
    char buf[SlcanPort::kMaxFormatted];
    size_t n = SlcanPort::format(f, buf);
    EXPECT_EQ(std::string(buf, n), std::string("t1818DEADBEEF00112233\r"));
    CanFrame back;
    ASSERT_TRUE(SlcanPort::parse_frame(buf, n - 1, back));
    EXPECT_EQ(back.id, 0x181u);
    EXPECT_EQ(static_cast<int>(back.dlc), 8);
    EXPECT_EQ(std::memcmp(back.data, data, 8), 0);
}

TEST(Slcan, CommandSequence) {
    SlcanPort port;
    SlcanPort::Result r;
    auto feed = [&](const char* s) {
        bool done = false;
        for (const char* p = s; *p; ++p) done = port.feed(*p, r);
        return done;
    };
    // A frame before the channel is open is refused.
    ASSERT_TRUE(feed("t1230\r"));
    EXPECT_EQ(std::string(r.reply, r.reply_len), std::string("\a"));
    EXPECT_FALSE(r.has_frame);

    ASSERT_TRUE(feed("S6\r"));
    EXPECT_EQ(std::string(r.reply, r.reply_len), std::string("\r"));
    ASSERT_TRUE(feed("O\r"));
    EXPECT_TRUE(port.is_open());
    ASSERT_TRUE(feed("t20120102\r"));
    EXPECT_TRUE(r.has_frame);
    EXPECT_EQ(r.frame.id, 0x201u);
    EXPECT_EQ(static_cast<int>(r.frame.data[1]), 2);
    // Malformed: DLC says 2 bytes but only 1 given.
    ASSERT_TRUE(feed("t201201\r"));
    EXPECT_FALSE(r.has_frame);
    EXPECT_EQ(std::string(r.reply, r.reply_len), std::string("\a"));
    ASSERT_TRUE(feed("C\r"));
    EXPECT_FALSE(port.is_open());
}
