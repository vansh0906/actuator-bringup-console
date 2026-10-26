// Parameter table: what the console reads, writes and saves to flash.
// The Python console mirrors the names, units and limits in
// console/bringup/params.py; an integration test checks both agree.
#pragma once

#include <cstddef>
#include <cstdint>

#include "protocol.hpp"

namespace act {

enum ParamIndex : uint8_t {
    P_FW_VERSION = 0,
    P_NODE_ID,
    P_VEL_KP,
    P_VEL_KI,
    P_POS_KP,
    P_CURRENT_LIMIT,
    P_VELOCITY_LIMIT,
    P_POSITION_MIN,
    P_POSITION_MAX,
    P_CMD_TIMEOUT_MS,
    P_OVERTEMP_C,
    P_UNDERVOLTAGE_V,
    P_ENCODER_OFFSET,
    P_CURRENT_OFFSET,
    P_MOTOR_POLARITY,
    P_FOLLOWING_ERROR,
    P_HOMING_POSITION,
    P_COUNT
};

struct ParamDesc {
    const char* name;
    const char* unit;
    float def;
    float min;
    float max;
    bool read_only;
    bool needs_disabled;  // refused while the drive is enabled
};

const ParamDesc& param_desc(uint8_t index);

// Flash storage hook. The host simulator stores a file; the ESP32 build
// stores to NVS (Preferences).
class Storage {
public:
    virtual ~Storage() = default;
    virtual bool save(const uint8_t* data, size_t n) = 0;
    virtual size_t load(uint8_t* data, size_t max_n) = 0;
};

class ParamTable {
public:
    explicit ParamTable(uint8_t node_id);

    float get(uint8_t index) const;
    // Internal set with no checks (used by the drive and tests).
    void set_raw(uint8_t index, float value);
    // Checked write, as a PARAM_REQ write would do it.
    proto::ParamStatus write(uint8_t index, float value, bool drive_enabled);
    void load_defaults();

    bool save(Storage& storage) const;
    bool load(Storage& storage);

    static constexpr size_t kBlobSize = 4 + 1 + P_COUNT * 4 + 1;

private:
    float values_[P_COUNT];
    uint8_t node_id_;
};

}  // namespace act
