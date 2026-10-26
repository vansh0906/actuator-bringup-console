#include <cstring>

#include "test_common.hpp"

using namespace act;
using namespace act::proto;

namespace {
struct MemStorage : Storage {
    uint8_t blob[256];
    size_t n = 0;
    bool save(const uint8_t* data, size_t len) override {
        std::memcpy(blob, data, len);
        n = len;
        return true;
    }
    size_t load(uint8_t* data, size_t max_n) override {
        size_t k = n < max_n ? n : max_n;
        std::memcpy(data, blob, k);
        return k;
    }
};
}  // namespace

TEST(Params, DefaultsAndChecks) {
    ParamTable p(3);
    EXPECT_EQ(p.get(P_NODE_ID), 3.0f);
    EXPECT_EQ(p.get(P_CMD_TIMEOUT_MS), 100.0f);
    EXPECT_TRUE(p.write(P_FW_VERSION, 9.0f, false) == ParamStatus::ReadOnly);
    EXPECT_TRUE(p.write(P_COUNT, 1.0f, false) == ParamStatus::BadIndex);
    EXPECT_TRUE(p.write(P_CURRENT_LIMIT, 50.0f, false) == ParamStatus::OutOfRange);
    EXPECT_TRUE(p.write(P_CURRENT_LIMIT, 2.0f, false) == ParamStatus::Ok);
    EXPECT_EQ(p.get(P_CURRENT_LIMIT), 2.0f);
    // Polarity must be exactly +1 or -1, and only while disabled.
    EXPECT_TRUE(p.write(P_MOTOR_POLARITY, 0.5f, false) == ParamStatus::OutOfRange);
    EXPECT_TRUE(p.write(P_MOTOR_POLARITY, -1.0f, true) == ParamStatus::NotWhileEnabled);
    EXPECT_TRUE(p.write(P_MOTOR_POLARITY, -1.0f, false) == ParamStatus::Ok);
    // Soft limits cannot cross.
    EXPECT_TRUE(p.write(P_POSITION_MIN, 3.0f, false) == ParamStatus::OutOfRange);
}

TEST(Params, SaveAndLoadSkipsEncoderZero) {
    MemStorage storage;
    ParamTable a(1);
    a.write(P_VEL_KP, 2.5f, false);
    a.write(P_ENCODER_OFFSET, 1.25f, false);
    ASSERT_TRUE(a.save(storage));

    ParamTable b(1);
    ASSERT_TRUE(b.load(storage));
    EXPECT_EQ(b.get(P_VEL_KP), 2.5f);
    // An incremental encoder has to be homed again after power-up.
    EXPECT_EQ(b.get(P_ENCODER_OFFSET), 0.0f);

    // A corrupted blob is rejected and leaves the defaults alone.
    storage.blob[10] ^= 0xFF;
    ParamTable c(1);
    EXPECT_FALSE(c.load(storage));
    EXPECT_EQ(c.get(P_VEL_KP), 1.5f);
}

TEST(Params, OverCanWithStorage) {
    MemStorage storage;
    Harness h(PlantConfig(), &storage);
    h.run(300);
    ParamMsg r = h.param(ParamOp::Write, P_VEL_KI, 25.0f);
    EXPECT_EQ(static_cast<int>(r.status), static_cast<int>(ParamStatus::Ok));
    EXPECT_EQ(r.value, 25.0f);
    r = h.param(ParamOp::Save, 0);
    EXPECT_EQ(static_cast<int>(r.status), static_cast<int>(ParamStatus::Ok));
    r = h.param(ParamOp::Read, 200);
    EXPECT_EQ(static_cast<int>(r.status), static_cast<int>(ParamStatus::BadIndex));
    r = h.param(ParamOp::LoadDefaults, P_VEL_KI);
    EXPECT_EQ(r.value, 20.0f);

    Harness reboot(PlantConfig(), &storage);
    r = reboot.param(ParamOp::Read, P_VEL_KI);
    EXPECT_EQ(r.value, 25.0f);
}
