#include "node.hpp"

namespace act {

ActuatorNode::ActuatorNode(uint8_t node_id, const PlantConfig& plant_cfg, Storage* storage)
    : node_id_(node_id), params_(node_id), plant_(plant_cfg), drive_(params_, plant_cfg.gear, plant_cfg.enc_cpr),
      storage_(storage) {
    if (storage_) params_.load(*storage_);
}

void ActuatorNode::push_tx(const CanFrame& f) {
    if (tx_count_ == kTxDepth) {  // full: drop the oldest, as a CAN controller mailbox would overrun
        tx_head_ = (tx_head_ + 1) % kTxDepth;
        --tx_count_;
    }
    tx_[(tx_head_ + tx_count_) % kTxDepth] = f;
    ++tx_count_;
}

bool ActuatorNode::pop_tx(CanFrame& out) {
    if (tx_count_ == 0) return false;
    out = tx_[tx_head_];
    tx_head_ = (tx_head_ + 1) % kTxDepth;
    --tx_count_;
    return true;
}

void ActuatorNode::on_frame(const CanFrame& f) {
    using namespace proto;
    if (f.id == ID_ESTOP) {
        if (is_valid_estop(f)) drive_.on_estop();
        return;
    }
    if (f.id == ID_FAULT_INJECT) {
        FaultInject fi;
        if (unpack_fault_inject(f, fi) && (fi.node == node_id_ || fi.node == 0)) {
            const auto kind = static_cast<InjectKind>(fi.kind);
            if (kind == InjectKind::DropStatus) {
                drop_fast_until_ms_ = now_ms_ + static_cast<uint32_t>(fi.value * 1000.0f);
            } else {
                if (kind == InjectKind::ClearAll) drop_fast_until_ms_ = 0;
                plant_.inject(kind, fi.value);
            }
        }
        return;
    }
    if (node_of(f.id) != node_id_) return;

    const uint32_t fc = function_code(f.id);
    if (fc == FC_COMMAND) {
        Command c;
        const CmdCheck check = unpack_command(f, c);
        if (check != CmdCheck::Ok || (have_counter_ && c.counter == last_counter_)) {
            if (e2e_errors_ < 255) ++e2e_errors_;
            return;
        }
        have_counter_ = true;
        last_counter_ = c.counter;
        drive_.on_command(c, now_ms_);
    } else if (fc == FC_PARAM_REQ) {
        ParamMsg req;
        if (unpack_param(f, req)) handle_param(req);
    }
}

void ActuatorNode::handle_param(const proto::ParamMsg& req) {
    using namespace proto;
    ParamMsg resp;
    resp.opcode = req.opcode;
    resp.index = req.index;
    ParamStatus status = ParamStatus::Ok;

    switch (static_cast<ParamOp>(req.opcode)) {
        case ParamOp::Read:
            if (req.index >= P_COUNT) status = ParamStatus::BadIndex;
            break;
        case ParamOp::Write:
            status = params_.write(req.index, req.value, drive_.enabled());
            if (status == ParamStatus::Ok && req.index == P_ENCODER_OFFSET) drive_.mark_homed();
            break;
        case ParamOp::Save:
            status = (storage_ && params_.save(*storage_)) ? ParamStatus::Ok : ParamStatus::StorageError;
            break;
        case ParamOp::LoadDefaults:
            if (drive_.enabled()) {
                status = ParamStatus::NotWhileEnabled;
            } else {
                params_.load_defaults();
                drive_.clear_homed();
            }
            break;
        default:
            status = ParamStatus::BadOpcode;
            break;
    }
    resp.status = static_cast<uint8_t>(status);
    resp.value = params_.get(req.index);
    push_tx(pack_param(FC_PARAM_RESP, node_id_, resp));
}

void ActuatorNode::tick_1ms() {
    using namespace proto;

    Sensors s;
    s.encoder_counts = plant_.encoder_counts();
    s.current_raw = plant_.read_current_sensor();
    s.bus_voltage = plant_.bus_voltage();
    s.motor_temp = plant_.motor_temp();
    terminal_cmd_ = drive_.update(s, now_ms_);

    const uint16_t fresh = drive_.take_new_faults();
    if (fresh) {
        Emcy e;
        e.fault_code = fresh;
        e.fault_flags = drive_.faults();
        e.state = drive_.state();
        e.motor_temp = drive_.motor_temp();
        e.bus_voltage = drive_.bus_voltage();
        push_tx(pack_emcy(node_id_, e));
    }

    for (int i = 0; i < kPlantSubsteps; ++i) plant_.step(Drive::kDt / kPlantSubsteps, terminal_cmd_);

    // Stagger the schedules by node ID so nodes do not all transmit in the
    // same millisecond.
    const uint32_t t = now_ms_ + node_id_;
    if (t % kFastPeriodMs == 0 && now_ms_ >= drop_fast_until_ms_) {
        StatusFast sf;
        sf.position = drive_.position();
        sf.velocity = drive_.velocity();
        sf.current = drive_.current();
        push_tx(pack_status_fast(node_id_, sf));
    }
    if ((t + 3) % kSlowPeriodMs == 0) {
        StatusSlow ss;
        ss.bus_voltage = drive_.bus_voltage();
        ss.motor_temp = drive_.motor_temp();
        ss.fault_flags = drive_.faults();
        ss.state = drive_.state();
        ss.mode = drive_.mode();
        ss.e2e_errors = e2e_errors_;
        push_tx(pack_status_slow(node_id_, ss));
    }
    if ((t + 7) % kHeartbeatPeriodMs == 0) {
        Heartbeat hb;
        hb.state = drive_.state();
        hb.uptime_ms = now_ms_;
        hb.homed = drive_.homed();
        push_tx(pack_heartbeat(node_id_, hb));
    }
    ++now_ms_;
}

}  // namespace act
