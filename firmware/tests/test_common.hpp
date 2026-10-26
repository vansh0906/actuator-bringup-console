#pragma once

#ifdef USE_GTEST_LITE
#include "gtest_lite.h"
#else
#include <gtest/gtest.h>
#endif

#include <cmath>
#include <vector>

#include "node.hpp"
#include "protocol.hpp"

// Drives one ActuatorNode the way the console would: a command every
// 10 ms (100 Hz) while `send_commands` is true, and every transmitted
// frame collected in `tx`.
struct Harness {
    act::ActuatorNode node;
    std::vector<act::CanFrame> tx;
    act::proto::Command cmd;
    bool send_commands = false;
    uint8_t counter = 0;

    explicit Harness(const act::PlantConfig& cfg = act::PlantConfig(), act::Storage* storage = nullptr)
        : node(1, cfg, storage) {}

    void send_command_now() {
        cmd.counter = ++counter;
        node.on_frame(act::proto::pack_command(1, cmd));
    }

    void run(int ms) {
        for (int i = 0; i < ms; ++i) {
            if (send_commands && node.uptime_ms() % 10 == 0) send_command_now();
            node.tick_1ms();
            act::CanFrame f;
            while (node.pop_tx(f)) tx.push_back(f);
        }
    }

    act::proto::ParamMsg param(act::proto::ParamOp op, uint8_t index, float value = 0.0f) {
        act::proto::ParamMsg req;
        req.opcode = static_cast<uint8_t>(op);
        req.index = index;
        req.value = value;
        node.on_frame(act::proto::pack_param(act::proto::FC_PARAM_REQ, 1, req));
        act::proto::ParamMsg resp;
        act::CanFrame f;
        while (node.pop_tx(f)) {
            if (act::proto::function_code(f.id) == act::proto::FC_PARAM_RESP) act::proto::unpack_param(f, resp);
            else tx.push_back(f);
        }
        return resp;
    }

    // Time (ms of node uptime) of the first EMCY carrying `bit`, or -1.
    int emcy_count(uint16_t bit) const {
        int n = 0;
        for (const auto& f : tx) {
            act::proto::Emcy e;
            if (act::proto::unpack_emcy(f, e) && (e.fault_code & bit)) ++n;
        }
        return n;
    }

    void enable(act::proto::Mode mode, double setpoint) {
        cmd.mode = mode;
        cmd.enable = true;
        cmd.clear_faults = false;
        cmd.setpoint = setpoint;
        send_commands = true;
    }
};
