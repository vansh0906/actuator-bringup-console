// SLCAN (Lawicel ASCII) framing: carries CAN frames over a serial link,
// the protocol USB-CAN adapters such as CANable speak. The console talks
// SLCAN to the actuators over a TCP socket (host simulator) or USB serial
// (ESP32).
//
//   host -> device:  O\r  open      C\r  close     S6\r  500 kbit/s
//                    t1238DEADBEEF00112233\r      standard frame
//   device -> host:  \r  OK         \a  error      t...\r  received frame
#pragma once

#include <cstddef>
#include <cstdint>

#include "protocol.hpp"

namespace act {

class SlcanPort {
public:
    static constexpr size_t kMaxLine = 32;
    static constexpr size_t kMaxFormatted = 32;

    struct Result {
        bool has_frame = false;  // a frame for the bus
        CanFrame frame;
        char reply[16];          // bytes to send back to the host
        size_t reply_len = 0;
    };

    // Feed one byte. Returns true when a command line completed; `out`
    // then holds the reply and, for t/T commands, the frame.
    bool feed(char c, Result& out);
    bool is_open() const { return open_; }
    uint32_t errors() const { return errors_; }

    // Format a frame as "tIIILDD..\r". Returns the length written.
    static size_t format(const CanFrame& f, char* out);
    // Parse "tIIILDD.." (no terminator). Returns false if malformed.
    static bool parse_frame(const char* line, size_t n, CanFrame& f);

private:
    void handle_line(Result& out);
    void reply(Result& out, const char* text);

    char line_[kMaxLine];
    size_t len_ = 0;
    bool overflow_ = false;
    bool open_ = false;
    uint32_t errors_ = 0;
};

}  // namespace act
