#include "protocol.hpp"

#include <cmath>
#include <cstring>

namespace act {
namespace proto {
namespace {

int64_t scale_round(double value, double scale, int64_t lo, int64_t hi) {
    if (std::isnan(value)) return 0;
    double raw = std::round(value / scale);
    if (raw < static_cast<double>(lo)) return lo;
    if (raw > static_cast<double>(hi)) return hi;
    return static_cast<int64_t>(raw);
}

void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>(v >> 8);
}
void put_u32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
}
uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t get_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
int16_t get_i16(const uint8_t* p) { return static_cast<int16_t>(get_u16(p)); }
int32_t get_i32(const uint8_t* p) { return static_cast<int32_t>(get_u32(p)); }

void put_f32(uint8_t* p, float v) {
    uint32_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    put_u32(p, bits);
}
float get_f32(const uint8_t* p) {
    uint32_t bits = get_u32(p);
    float v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

CanFrame make_frame(uint32_t id, uint8_t dlc) {
    CanFrame f;
    f.id = id;
    f.dlc = dlc;
    return f;
}

}  // namespace

uint8_t crc8_j1850(const uint8_t* data, size_t n) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x1D) : static_cast<uint8_t>(crc << 1);
        }
    }
    return static_cast<uint8_t>(crc ^ 0xFF);
}

uint8_t e2e_crc(uint32_t id, const uint8_t* payload, size_t n) {
    uint8_t buf[2 + 8];
    buf[0] = static_cast<uint8_t>(id & 0xFF);
    buf[1] = static_cast<uint8_t>((id >> 8) & 0xFF);
    if (n > 8) n = 8;
    std::memcpy(buf + 2, payload, n);
    return crc8_j1850(buf, 2 + n);
}

CanFrame pack_command(uint8_t node, const Command& c) {
    CanFrame f = make_frame(FC_COMMAND + node, 8);
    f.data[0] = static_cast<uint8_t>((static_cast<uint8_t>(c.mode) & 0x0F) | (c.enable ? 0x10 : 0) |
                                     (c.clear_faults ? 0x20 : 0));
    put_u32(f.data + 1, static_cast<uint32_t>(static_cast<int32_t>(scale_round(c.setpoint, 1e-4, INT32_MIN, INT32_MAX))));
    f.data[5] = 0;
    f.data[6] = c.counter;
    f.data[7] = e2e_crc(f.id, f.data, 7);
    return f;
}

CmdCheck unpack_command(const CanFrame& f, Command& out) {
    if (f.dlc != 8) return CmdCheck::BadLength;
    if (e2e_crc(f.id, f.data, 7) != f.data[7]) return CmdCheck::BadCrc;
    uint8_t mode = f.data[0] & 0x0F;
    out.mode = mode <= 3 ? static_cast<Mode>(mode) : Mode::Idle;
    out.enable = (f.data[0] & 0x10) != 0;
    out.clear_faults = (f.data[0] & 0x20) != 0;
    out.setpoint = get_i32(f.data + 1) * 1e-4;
    out.counter = f.data[6];
    return CmdCheck::Ok;
}

CanFrame pack_estop(uint8_t counter) {
    CanFrame f = make_frame(ID_ESTOP, 3);
    f.data[0] = ESTOP_MAGIC;
    f.data[1] = counter;
    f.data[2] = e2e_crc(f.id, f.data, 2);
    return f;
}

bool is_valid_estop(const CanFrame& f) {
    return f.id == ID_ESTOP && f.dlc == 3 && f.data[0] == ESTOP_MAGIC && e2e_crc(f.id, f.data, 2) == f.data[2];
}

CanFrame pack_status_fast(uint8_t node, const StatusFast& s) {
    CanFrame f = make_frame(FC_STATUS_FAST + node, 8);
    put_u32(f.data, static_cast<uint32_t>(static_cast<int32_t>(scale_round(s.position, 1e-4, INT32_MIN, INT32_MAX))));
    put_u16(f.data + 4, static_cast<uint16_t>(static_cast<int16_t>(scale_round(s.velocity, 1e-3, INT16_MIN, INT16_MAX))));
    put_u16(f.data + 6, static_cast<uint16_t>(static_cast<int16_t>(scale_round(s.current, 1e-3, INT16_MIN, INT16_MAX))));
    return f;
}

bool unpack_status_fast(const CanFrame& f, StatusFast& out) {
    if (f.dlc != 8 || function_code(f.id) != FC_STATUS_FAST) return false;
    out.position = get_i32(f.data) * 1e-4;
    out.velocity = get_i16(f.data + 4) * 1e-3;
    out.current = get_i16(f.data + 6) * 1e-3;
    return true;
}

CanFrame pack_status_slow(uint8_t node, const StatusSlow& s) {
    CanFrame f = make_frame(FC_STATUS_SLOW + node, 8);
    put_u16(f.data, static_cast<uint16_t>(scale_round(s.bus_voltage, 0.01, 0, 65535)));
    put_u16(f.data + 2, static_cast<uint16_t>(static_cast<int16_t>(scale_round(s.motor_temp, 0.1, INT16_MIN, INT16_MAX))));
    put_u16(f.data + 4, s.fault_flags);
    f.data[6] = static_cast<uint8_t>((static_cast<uint8_t>(s.state) & 0x0F) | ((static_cast<uint8_t>(s.mode) & 0x0F) << 4));
    f.data[7] = s.e2e_errors;
    return f;
}

bool unpack_status_slow(const CanFrame& f, StatusSlow& out) {
    if (f.dlc != 8 || function_code(f.id) != FC_STATUS_SLOW) return false;
    out.bus_voltage = get_u16(f.data) * 0.01;
    out.motor_temp = get_i16(f.data + 2) * 0.1;
    out.fault_flags = get_u16(f.data + 4);
    out.state = static_cast<State>(f.data[6] & 0x0F);
    out.mode = static_cast<Mode>((f.data[6] >> 4) & 0x0F);
    out.e2e_errors = f.data[7];
    return true;
}

CanFrame pack_emcy(uint8_t node, const Emcy& e) {
    CanFrame f = make_frame(FC_EMCY + node, 8);
    put_u16(f.data, e.fault_code);
    put_u16(f.data + 2, e.fault_flags);
    f.data[4] = static_cast<uint8_t>(e.state);
    put_u16(f.data + 5, static_cast<uint16_t>(static_cast<int16_t>(scale_round(e.motor_temp, 0.1, INT16_MIN, INT16_MAX))));
    f.data[7] = static_cast<uint8_t>(scale_round(e.bus_voltage, 0.5, 0, 255));
    return f;
}

bool unpack_emcy(const CanFrame& f, Emcy& out) {
    if (f.dlc != 8 || function_code(f.id) != FC_EMCY || f.id == ID_ESTOP) return false;
    out.fault_code = get_u16(f.data);
    out.fault_flags = get_u16(f.data + 2);
    out.state = static_cast<State>(f.data[4]);
    out.motor_temp = get_i16(f.data + 5) * 0.1;
    out.bus_voltage = f.data[7] * 0.5;
    return true;
}

CanFrame pack_param(uint32_t function, uint8_t node, const ParamMsg& p) {
    CanFrame f = make_frame(function + node, 8);
    f.data[0] = p.opcode;
    f.data[1] = p.index;
    f.data[2] = p.status;
    f.data[3] = 0;
    put_f32(f.data + 4, p.value);
    return f;
}

bool unpack_param(const CanFrame& f, ParamMsg& out) {
    uint32_t fc = function_code(f.id);
    if (f.dlc != 8 || (fc != FC_PARAM_REQ && fc != FC_PARAM_RESP)) return false;
    out.opcode = f.data[0];
    out.index = f.data[1];
    out.status = f.data[2];
    out.value = get_f32(f.data + 4);
    return true;
}

CanFrame pack_heartbeat(uint8_t node, const Heartbeat& h) {
    CanFrame f = make_frame(FC_HEARTBEAT + node, 8);
    f.data[0] = static_cast<uint8_t>(h.state);
    put_u32(f.data + 1, h.uptime_ms);
    f.data[5] = h.fw_major;
    f.data[6] = h.fw_minor;
    f.data[7] = h.homed ? 1 : 0;
    return f;
}

bool unpack_heartbeat(const CanFrame& f, Heartbeat& out) {
    if (f.dlc != 8 || function_code(f.id) != FC_HEARTBEAT) return false;
    out.state = static_cast<State>(f.data[0]);
    out.uptime_ms = get_u32(f.data + 1);
    out.fw_major = f.data[5];
    out.fw_minor = f.data[6];
    out.homed = (f.data[7] & 1) != 0;
    return true;
}

CanFrame pack_fault_inject(const FaultInject& fi) {
    CanFrame f = make_frame(ID_FAULT_INJECT, 8);
    f.data[0] = fi.node;
    f.data[1] = fi.kind;
    put_f32(f.data + 4, fi.value);
    return f;
}

bool unpack_fault_inject(const CanFrame& f, FaultInject& out) {
    if (f.id != ID_FAULT_INJECT || f.dlc != 8) return false;
    out.node = f.data[0];
    out.kind = f.data[1];
    out.value = get_f32(f.data + 4);
    return true;
}

}  // namespace proto
}  // namespace act
