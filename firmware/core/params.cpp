#include "params.hpp"

#include <cmath>
#include <cstring>

namespace act {
namespace {

// name, unit, default, min, max, read_only, needs_disabled
const ParamDesc kParams[P_COUNT] = {
    {"fw_version", "", 1.0f, 0.0f, 255.0f, true, false},
    {"node_id", "", 1.0f, 1.0f, 127.0f, true, false},
    {"vel_kp", "A/(rad/s)", 1.5f, 0.0f, 20.0f, false, false},
    {"vel_ki", "A/rad", 20.0f, 0.0f, 500.0f, false, false},
    {"pos_kp", "1/s", 8.0f, 0.0f, 100.0f, false, false},
    {"current_limit", "A", 5.0f, 0.1f, 15.0f, false, false},
    {"velocity_limit", "rad/s", 3.0f, 0.1f, 10.0f, false, false},
    {"position_min", "rad", -2.4f, -10.0f, 10.0f, false, false},
    {"position_max", "rad", 2.4f, -10.0f, 10.0f, false, false},
    {"cmd_timeout_ms", "ms", 100.0f, 20.0f, 1000.0f, false, false},
    {"overtemp_c", "degC", 80.0f, 40.0f, 120.0f, false, false},
    {"undervoltage_v", "V", 40.0f, 20.0f, 48.0f, false, false},
    {"encoder_offset", "rad", 0.0f, -1000.0f, 1000.0f, false, true},
    {"current_offset", "A", 0.0f, -2.0f, 2.0f, false, true},
    {"motor_polarity", "", 1.0f, -1.0f, 1.0f, false, true},
    {"following_error", "rad/s", 0.5f, 0.05f, 10.0f, false, false},
    {"homing_position", "rad", -2.5f, -10.0f, 10.0f, false, false},
};

const uint8_t kMagic[4] = {'A', 'C', 'T', 'P'};
constexpr uint8_t kBlobVersion = 1;

}  // namespace

const ParamDesc& param_desc(uint8_t index) {
    static const ParamDesc kInvalid = {"invalid", "", 0.0f, 0.0f, 0.0f, true, false};
    return index < P_COUNT ? kParams[index] : kInvalid;
}

ParamTable::ParamTable(uint8_t node_id) : node_id_(node_id) { load_defaults(); }

void ParamTable::load_defaults() {
    for (uint8_t i = 0; i < P_COUNT; ++i) values_[i] = kParams[i].def;
    values_[P_FW_VERSION] = static_cast<float>(proto::FW_MAJOR) + static_cast<float>(proto::FW_MINOR) / 10.0f;
    values_[P_NODE_ID] = static_cast<float>(node_id_);
}

float ParamTable::get(uint8_t index) const { return index < P_COUNT ? values_[index] : 0.0f; }

void ParamTable::set_raw(uint8_t index, float value) {
    if (index < P_COUNT) values_[index] = value;
}

proto::ParamStatus ParamTable::write(uint8_t index, float value, bool drive_enabled) {
    using proto::ParamStatus;
    if (index >= P_COUNT) return ParamStatus::BadIndex;
    const ParamDesc& d = kParams[index];
    if (d.read_only) return ParamStatus::ReadOnly;
    if (d.needs_disabled && drive_enabled) return ParamStatus::NotWhileEnabled;
    if (std::isnan(value) || value < d.min || value > d.max) return ParamStatus::OutOfRange;
    if (index == P_MOTOR_POLARITY && value != 1.0f && value != -1.0f) return ParamStatus::OutOfRange;
    if (index == P_POSITION_MIN && value >= values_[P_POSITION_MAX]) return ParamStatus::OutOfRange;
    if (index == P_POSITION_MAX && value <= values_[P_POSITION_MIN]) return ParamStatus::OutOfRange;
    values_[index] = value;
    return ParamStatus::Ok;
}

bool ParamTable::save(Storage& storage) const {
    uint8_t blob[kBlobSize];
    std::memcpy(blob, kMagic, 4);
    blob[4] = kBlobVersion;
    std::memcpy(blob + 5, values_, P_COUNT * 4);
    blob[kBlobSize - 1] = proto::crc8_j1850(blob, kBlobSize - 1);
    return storage.save(blob, kBlobSize);
}

bool ParamTable::load(Storage& storage) {
    uint8_t blob[kBlobSize];
    if (storage.load(blob, kBlobSize) != kBlobSize) return false;
    if (std::memcmp(blob, kMagic, 4) != 0 || blob[4] != kBlobVersion) return false;
    if (proto::crc8_j1850(blob, kBlobSize - 1) != blob[kBlobSize - 1]) return false;
    float loaded[P_COUNT];
    std::memcpy(loaded, blob + 5, P_COUNT * 4);
    for (uint8_t i = 0; i < P_COUNT; ++i) {
        const ParamDesc& d = kParams[i];
        if (d.read_only) continue;
        // The encoder is incremental, so a saved zero is meaningless after a
        // power cycle: the joint must be homed again.
        if (i == P_ENCODER_OFFSET) continue;
        if (std::isnan(loaded[i]) || loaded[i] < d.min || loaded[i] > d.max) continue;
        values_[i] = loaded[i];
    }
    return true;
}

}  // namespace act
