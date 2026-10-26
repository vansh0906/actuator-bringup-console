// The actuator firmware on a real microcontroller: the same core as the
// host simulator (drive, protocol, parameters, fault handling, simulated
// motor), talking SLCAN over the ESP32's USB serial port at 921600 baud.
// The console connects to it exactly as it does to the simulator:
//
//   bringup-gui --url /dev/cu.usbserial-0001 --connect
//
// Before compiling, copy the core files in:  sh firmware/esp32/copy_core.sh
// Parameters are saved to NVS flash (Preferences), so a saved calibration
// survives a reset, like on a real drive.

#include <Arduino.h>
#include <Preferences.h>

#include "node.hpp"
#include "slcan.hpp"

#ifndef NODE_ID
#define NODE_ID 1
#endif
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 921600
#endif

namespace {

class NvsStorage : public act::Storage {
public:
    bool save(const uint8_t* data, size_t n) override {
        prefs_.begin("actuator", false);
        size_t written = prefs_.putBytes("params", data, n);
        prefs_.end();
        return written == n;
    }
    size_t load(uint8_t* data, size_t max_n) override {
        prefs_.begin("actuator", true);
        size_t read = prefs_.getBytes("params", data, max_n);
        prefs_.end();
        return read;
    }

private:
    Preferences prefs_;
};

act::PlantConfig plant_config() {
    act::PlantConfig c;
    c.seed = 1000 + NODE_ID;
    c.current_sensor_offset = 0.12;  // something for the calibration to find
    c.initial_position = 0.7;
    return c;
}

NvsStorage storage;
act::ActuatorNode* node = nullptr;
act::SlcanPort slcan;
uint32_t next_tick_us = 0;
uint32_t overruns = 0;

}  // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    static act::ActuatorNode actuator(NODE_ID, plant_config(), &storage);
    node = &actuator;
#ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
#endif
    next_tick_us = micros();
}

void loop() {
    // Host -> SLCAN -> actuator.
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        act::SlcanPort::Result r;
        if (slcan.feed(c, r)) {
            Serial.write(reinterpret_cast<const uint8_t*>(r.reply), r.reply_len);
            if (r.has_frame) node->on_frame(r.frame);
        }
    }

    // 1 kHz control tick, then actuator -> SLCAN -> host.
    const uint32_t now = micros();
    if (static_cast<int32_t>(now - next_tick_us) >= 0) {
        node->tick_1ms();
        next_tick_us += 1000;
        if (static_cast<int32_t>(now - next_tick_us) > 5000) {  // fell more than 5 ticks behind
            next_tick_us = now + 1000;
            ++overruns;
        }
        act::CanFrame f;
        char buf[act::SlcanPort::kMaxFormatted];
        while (node->pop_tx(f)) {
            if (slcan.is_open()) Serial.write(reinterpret_cast<const uint8_t*>(buf), act::SlcanPort::format(f, buf));
        }
#ifdef LED_BUILTIN
        // LED: off = disabled, on = enabled, fast blink = fault.
        const bool fault = node->drive().faults() != 0;
        digitalWrite(LED_BUILTIN, fault ? ((millis() / 100) % 2) : (node->drive().enabled() ? 1 : 0));
#endif
    }
}
