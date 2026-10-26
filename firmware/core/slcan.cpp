#include "slcan.hpp"

#include <cstring>

namespace act {
namespace {

int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

const char kHex[] = "0123456789ABCDEF";

}  // namespace

bool SlcanPort::parse_frame(const char* line, size_t n, CanFrame& f) {
    // t + 3 hex ID + 1 digit DLC + 2*DLC hex data
    if (n < 5 || line[0] != 't') return false;
    uint32_t id = 0;
    for (int i = 1; i <= 3; ++i) {
        int v = hex_value(line[i]);
        if (v < 0) return false;
        id = (id << 4) | static_cast<uint32_t>(v);
    }
    if (id > 0x7FF) return false;
    int dlc = hex_value(line[4]);
    if (dlc < 0 || dlc > 8) return false;
    if (n != static_cast<size_t>(5 + 2 * dlc)) return false;
    f.id = id;
    f.dlc = static_cast<uint8_t>(dlc);
    std::memset(f.data, 0, sizeof(f.data));
    for (int i = 0; i < dlc; ++i) {
        int hi = hex_value(line[5 + 2 * i]);
        int lo = hex_value(line[6 + 2 * i]);
        if (hi < 0 || lo < 0) return false;
        f.data[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return true;
}

size_t SlcanPort::format(const CanFrame& f, char* out) {
    size_t n = 0;
    out[n++] = 't';
    out[n++] = kHex[(f.id >> 8) & 0x7];
    out[n++] = kHex[(f.id >> 4) & 0xF];
    out[n++] = kHex[f.id & 0xF];
    const uint8_t dlc = f.dlc > 8 ? 8 : f.dlc;
    out[n++] = kHex[dlc];
    for (uint8_t i = 0; i < dlc; ++i) {
        out[n++] = kHex[f.data[i] >> 4];
        out[n++] = kHex[f.data[i] & 0xF];
    }
    out[n++] = '\r';
    return n;
}

void SlcanPort::reply(Result& out, const char* text) {
    size_t n = std::strlen(text);
    if (n > sizeof(out.reply)) n = sizeof(out.reply);
    std::memcpy(out.reply, text, n);
    out.reply_len = n;
}

bool SlcanPort::feed(char c, Result& out) {
    if (c == '\n') return false;  // tolerate CRLF
    if (c != '\r') {
        if (len_ < kMaxLine) {
            line_[len_++] = c;
        } else {
            overflow_ = true;
        }
        return false;
    }
    out = Result();
    if (overflow_) {
        ++errors_;
        reply(out, "\a");
    } else {
        handle_line(out);
    }
    len_ = 0;
    overflow_ = false;
    return true;
}

void SlcanPort::handle_line(Result& out) {
    if (len_ == 0) {  // bare CR: keep-alive
        reply(out, "\r");
        return;
    }
    switch (line_[0]) {
        case 'O':
        case 'L':  // listen-only: treated as open
            open_ = true;
            reply(out, "\r");
            return;
        case 'C':
            open_ = false;
            reply(out, "\r");
            return;
        case 'S':
            if (len_ == 2 && line_[1] >= '0' && line_[1] <= '8' && !open_) {
                reply(out, "\r");  // the simulated bus always runs at 500 kbit/s
            } else {
                ++errors_;
                reply(out, "\a");
            }
            return;
        case 'V':
            reply(out, "V1010\r");
            return;
        case 'v':
            reply(out, "v0100\r");
            return;
        case 'N':
            reply(out, "NACT1\r");
            return;
        case 'F':
            reply(out, "F00\r");
            return;
        case 'Z':
        case 'M':
        case 'm':
        case 'X':
        case 'W':
            reply(out, "\r");
            return;
        case 't': {
            CanFrame f;
            if (open_ && parse_frame(line_, len_, f)) {
                out.has_frame = true;
                out.frame = f;
                reply(out, "z\r");
            } else {
                ++errors_;
                reply(out, "\a");
            }
            return;
        }
        case 'T':
            // Extended (29-bit) frames are not used on this bus.
            reply(out, open_ ? "Z\r" : "\a");
            return;
        default:
            ++errors_;
            reply(out, "\a");
            return;
    }
}

}  // namespace act
