"""Parameter access over CAN (read, write, save to flash, defaults) and
YAML import/export. PARAMS mirrors firmware/core/params.cpp; the
integration tests check the two agree."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from .protocol import PARAM_STATUS_TEXT, ParamOp


@dataclass(frozen=True)
class ParamInfo:
    index: int
    name: str
    unit: str
    default: float
    minimum: float
    maximum: float
    read_only: bool = False
    needs_disabled: bool = False
    description: str = ""


PARAMS: list[ParamInfo] = [
    ParamInfo(0, "fw_version", "", 1.0, 0, 255, True, description="Firmware version"),
    ParamInfo(1, "node_id", "", 1.0, 1, 127, True, description="CAN node ID"),
    ParamInfo(2, "vel_kp", "A/(rad/s)", 1.5, 0, 20, description="Velocity loop proportional gain"),
    ParamInfo(3, "vel_ki", "A/rad", 20.0, 0, 500, description="Velocity loop integral gain"),
    ParamInfo(4, "pos_kp", "1/s", 8.0, 0, 100, description="Position loop gain"),
    ParamInfo(5, "current_limit", "A", 5.0, 0.1, 15, description="Motor current limit"),
    ParamInfo(6, "velocity_limit", "rad/s", 3.0, 0.1, 10, description="Joint speed limit"),
    ParamInfo(7, "position_min", "rad", -2.4, -10, 10, description="Soft limit, active once homed"),
    ParamInfo(8, "position_max", "rad", 2.4, -10, 10, description="Soft limit, active once homed"),
    ParamInfo(9, "cmd_timeout_ms", "ms", 100.0, 20, 1000, description="Command watchdog"),
    ParamInfo(10, "overtemp_c", "degC", 80.0, 40, 120, description="Winding over-temperature trip"),
    ParamInfo(11, "undervoltage_v", "V", 40.0, 20, 48, description="Bus under-voltage trip"),
    ParamInfo(12, "encoder_offset", "rad", 0.0, -1000, 1000, needs_disabled=True,
              description="Joint zero (set by homing)"),
    ParamInfo(13, "current_offset", "A", 0.0, -2, 2, needs_disabled=True,
              description="Current-sensor zero (set by calibration)"),
    ParamInfo(14, "motor_polarity", "", 1.0, -1, 1, needs_disabled=True,
              description="+1 or -1: flips the motor phase order"),
    ParamInfo(15, "following_error", "rad/s", 0.5, 0.05, 10, description="Velocity tracking error trip"),
    ParamInfo(16, "homing_position", "rad", -2.5, -10, 10, description="Joint angle at the homing hard stop"),
]
BY_NAME = {p.name: p for p in PARAMS}
BY_INDEX = {p.index: p for p in PARAMS}

# Not exported to YAML: identity, and the joint zero (an incremental
# encoder must be homed after every power-up).
NOT_EXPORTED = {"fw_version", "node_id", "encoder_offset"}


class ParamError(RuntimeError):
    pass


def resolve(name_or_index) -> ParamInfo:
    if isinstance(name_or_index, ParamInfo):
        return name_or_index
    if isinstance(name_or_index, int):
        if name_or_index not in BY_INDEX:
            raise ParamError(f"no parameter with index {name_or_index}")
        return BY_INDEX[name_or_index]
    if name_or_index not in BY_NAME:
        raise ParamError(f"no parameter named {name_or_index!r}")
    return BY_NAME[name_or_index]


class ParamClient:
    def __init__(self, dm, timeout: float = 0.3, retries: int = 2):
        self.dm = dm
        self.timeout = timeout
        self.retries = retries

    def request(self, node: int, op: ParamOp, index: int, value: float = 0.0) -> tuple[int, float]:
        frame = self.dm.codec.param_request(node, op, index, value)
        for _ in range(self.retries + 1):
            result = self.dm.param_transaction(frame, node, int(op), index, self.timeout)
            if result is not None:
                return result
        raise ParamError(f"node {node}: no response to {op.name} of parameter {index}")

    def read(self, node: int, name) -> float:
        p = resolve(name)
        status, value = self.request(node, ParamOp.READ, p.index)
        if status != 0:
            raise ParamError(f"node {node}: read {p.name} failed: {PARAM_STATUS_TEXT.get(status, status)}")
        return value

    def write(self, node: int, name, value: float) -> float:
        p = resolve(name)
        status, stored = self.request(node, ParamOp.WRITE, p.index, float(value))
        if status != 0:
            raise ParamError(f"node {node}: write {p.name}={value:g} refused: {PARAM_STATUS_TEXT.get(status, status)}")
        return stored

    def save(self, node: int) -> None:
        status, _ = self.request(node, ParamOp.SAVE, 0)
        if status != 0:
            raise ParamError(f"node {node}: save failed: {PARAM_STATUS_TEXT.get(status, status)}")

    def load_defaults(self, node: int) -> None:
        status, _ = self.request(node, ParamOp.LOAD_DEFAULTS, 0)
        if status != 0:
            raise ParamError(f"node {node}: load defaults failed: {PARAM_STATUS_TEXT.get(status, status)}")

    def read_all(self, node: int) -> dict[str, float]:
        return {p.name: self.read(node, p) for p in PARAMS}

    def export_yaml(self, node: int, path: str | Path) -> dict:
        values = self.read_all(node)
        doc = {
            "node": node,
            "firmware": values["fw_version"],
            "params": {k: round(float(v), 6) for k, v in values.items() if k not in NOT_EXPORTED},
        }
        Path(path).write_text(yaml.safe_dump(doc, sort_keys=False))
        return doc

    def import_yaml(self, node: int, path: str | Path) -> list[str]:
        doc = yaml.safe_load(Path(path).read_text())
        params = doc.get("params", doc)
        written = []
        # Soft limits must not cross mid-update: widen first, then narrow.
        order = sorted(params.items(), key=lambda kv: (kv[0] not in ("position_min",), kv[0]))
        for name, value in order:
            if name in NOT_EXPORTED:
                continue
            p = resolve(name)
            if p.read_only:
                continue
            self.write(node, p, float(value))
            written.append(name)
        return written
