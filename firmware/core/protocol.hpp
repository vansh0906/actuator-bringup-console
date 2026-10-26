// CAN protocol for the smart actuator. The single source of truth is
// protocol/actuator.dbc; this file mirrors it for the firmware. The Python
// console decodes frames from the DBC directly, and the integration tests
// check that both sides agree.
#pragma once

#include <cstddef>
#include <cstdint>

namespace act {

struct CanFrame {
    uint32_t id = 0;
    uint8_t dlc = 0;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

namespace proto {

// IDs: function code + node ID (1..127), except ESTOP and FAULT_INJECT.
constexpr uint32_t ID_ESTOP = 0x080;
constexpr uint32_t FC_EMCY = 0x080;
constexpr uint32_t FC_STATUS_FAST = 0x180;
constexpr uint32_t FC_COMMAND = 0x200;
constexpr uint32_t FC_STATUS_SLOW = 0x280;
constexpr uint32_t FC_PARAM_RESP = 0x580;
constexpr uint32_t FC_PARAM_REQ = 0x600;
constexpr uint32_t FC_HEARTBEAT = 0x700;
constexpr uint32_t ID_FAULT_INJECT = 0x7F0;

constexpr uint8_t ESTOP_MAGIC = 0xA5;
constexpr uint8_t FW_MAJOR = 1;
constexpr uint8_t FW_MINOR = 0;

enum class Mode : uint8_t { Idle = 0, Current = 1, Velocity = 2, Position = 3 };
enum class State : uint8_t { Boot = 0, Disabled = 1, Enabled = 2, Fault = 3 };

namespace fault {
constexpr uint16_t OVERCURRENT = 1u << 0;
constexpr uint16_t OVERTEMP = 1u << 1;
constexpr uint16_t UNDERVOLTAGE = 1u << 2;
constexpr uint16_t CMD_TIMEOUT = 1u << 3;
constexpr uint16_t FOLLOWING_ERROR = 1u << 4;
constexpr uint16_t OVERSPEED = 1u << 5;
constexpr uint16_t ENCODER = 1u << 6;
constexpr uint16_t STALL = 1u << 7;
constexpr uint16_t ESTOP = 1u << 8;
}  // namespace fault

enum class ParamOp : uint8_t { Read = 1, Write = 2, Save = 3, LoadDefaults = 4 };
enum class ParamStatus : uint8_t {
    Ok = 0,
    BadIndex = 1,
    ReadOnly = 2,
    OutOfRange = 3,
    NotWhileEnabled = 4,
    BadOpcode = 5,
    StorageError = 6,
};
enum class InjectKind : uint8_t {
    ClearAll = 0,
    Overheat = 1,
    Undervoltage = 2,
    EncoderGlitch = 3,
    Jam = 4,
    CurrentNoise = 5,
    DropStatus = 6,
};

// CRC-8 SAE J1850: poly 0x1D, init 0xFF, final XOR 0xFF.
uint8_t crc8_j1850(const uint8_t* data, size_t n);
// End-to-end CRC: covers the CAN ID (low byte, high byte) then the payload,
// so a frame delivered under the wrong ID is caught too.
uint8_t e2e_crc(uint32_t id, const uint8_t* payload, size_t n);

struct Command {
    Mode mode = Mode::Idle;
    bool enable = false;
    bool clear_faults = false;
    double setpoint = 0.0;  // A, rad/s or rad depending on mode
    uint8_t counter = 0;
};
struct StatusFast {
    double position = 0.0;  // rad
    double velocity = 0.0;  // rad/s
    double current = 0.0;   // A
};
struct StatusSlow {
    double bus_voltage = 0.0;  // V
    double motor_temp = 0.0;   // degC
    uint16_t fault_flags = 0;
    State state = State::Boot;
    Mode mode = Mode::Idle;
    uint8_t e2e_errors = 0;
};
struct Emcy {
    uint16_t fault_code = 0;
    uint16_t fault_flags = 0;
    State state = State::Fault;
    double motor_temp = 0.0;
    double bus_voltage = 0.0;
};
struct ParamMsg {
    uint8_t opcode = 0;
    uint8_t index = 0;
    uint8_t status = 0;
    float value = 0.0f;
};
struct Heartbeat {
    State state = State::Boot;
    uint32_t uptime_ms = 0;
    uint8_t fw_major = FW_MAJOR;
    uint8_t fw_minor = FW_MINOR;
    bool homed = false;
};
struct FaultInject {
    uint8_t node = 0;
    uint8_t kind = 0;
    float value = 0.0f;
};

enum class CmdCheck { Ok, BadLength, BadCrc };

inline uint32_t function_code(uint32_t id) { return id & 0x780u; }
inline uint8_t node_of(uint32_t id) { return static_cast<uint8_t>(id & 0x7Fu); }

CanFrame pack_command(uint8_t node, const Command& c);
CmdCheck unpack_command(const CanFrame& f, Command& out);
CanFrame pack_estop(uint8_t counter);
bool is_valid_estop(const CanFrame& f);

CanFrame pack_status_fast(uint8_t node, const StatusFast& s);
bool unpack_status_fast(const CanFrame& f, StatusFast& out);
CanFrame pack_status_slow(uint8_t node, const StatusSlow& s);
bool unpack_status_slow(const CanFrame& f, StatusSlow& out);
CanFrame pack_emcy(uint8_t node, const Emcy& e);
bool unpack_emcy(const CanFrame& f, Emcy& out);
CanFrame pack_param(uint32_t function, uint8_t node, const ParamMsg& p);
bool unpack_param(const CanFrame& f, ParamMsg& out);
CanFrame pack_heartbeat(uint8_t node, const Heartbeat& h);
bool unpack_heartbeat(const CanFrame& f, Heartbeat& out);
CanFrame pack_fault_inject(const FaultInject& fi);
bool unpack_fault_inject(const CanFrame& f, FaultInject& out);

}  // namespace proto
}  // namespace act
