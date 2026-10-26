// One actuator on the bus: parameter table + drive + (simulated) plant, the
// CAN message handling and the transmit schedule.
//
//   STATUS_FAST  100 Hz      STATUS_SLOW  10 Hz      HEARTBEAT  2 Hz
//   EMCY         once per new fault       PARAM_RESP  per request
#pragma once

#include <cstdint>

#include "controller.hpp"
#include "params.hpp"
#include "plant.hpp"
#include "protocol.hpp"

namespace act {

class ActuatorNode {
public:
    static constexpr int kPlantSubsteps = 10;  // 10 kHz plant integration
    static constexpr uint32_t kFastPeriodMs = 10;
    static constexpr uint32_t kSlowPeriodMs = 100;
    static constexpr uint32_t kHeartbeatPeriodMs = 500;

    ActuatorNode(uint8_t node_id, const PlantConfig& plant_cfg, Storage* storage);

    void on_frame(const CanFrame& f);
    void tick_1ms();
    bool pop_tx(CanFrame& out);

    uint8_t node_id() const { return node_id_; }
    uint32_t uptime_ms() const { return now_ms_; }
    uint8_t e2e_errors() const { return e2e_errors_; }
    Plant& plant() { return plant_; }
    Drive& drive() { return drive_; }
    ParamTable& params() { return params_; }

private:
    void push_tx(const CanFrame& f);
    void handle_param(const proto::ParamMsg& req);

    uint8_t node_id_;
    ParamTable params_;
    Plant plant_;
    Drive drive_;
    Storage* storage_;

    uint32_t now_ms_ = 0;
    double terminal_cmd_ = 0.0;
    uint32_t drop_fast_until_ms_ = 0;
    uint8_t e2e_errors_ = 0;
    bool have_counter_ = false;
    uint8_t last_counter_ = 0;

    static constexpr int kTxDepth = 32;
    CanFrame tx_[kTxDepth];
    int tx_head_ = 0;
    int tx_count_ = 0;
};

}  // namespace act
