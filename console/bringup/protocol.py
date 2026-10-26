"""Node-aware view of the actuator CAN protocol, built on the DBC file.

The DBC (protocol/actuator.dbc) is the interface spec shared with the
firmware. It lists every message for node 1; node n uses the same layout at
ID = function code + n, CANopen style.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path

from .dbc import Database, load_dbc

ID_ESTOP = 0x080
FC_EMCY = 0x080
FC_STATUS_FAST = 0x180
FC_COMMAND = 0x200
FC_STATUS_SLOW = 0x280
FC_PARAM_RESP = 0x580
FC_PARAM_REQ = 0x600
FC_HEARTBEAT = 0x700
ID_FAULT_INJECT = 0x7F0
ESTOP_MAGIC = 0xA5
BITRATE = 500_000


class Mode(IntEnum):
    IDLE = 0
    CURRENT = 1
    VELOCITY = 2
    POSITION = 3


class State(IntEnum):
    BOOT = 0
    DISABLED = 1
    ENABLED = 2
    FAULT = 3


class ParamOp(IntEnum):
    READ = 1
    WRITE = 2
    SAVE = 3
    LOAD_DEFAULTS = 4


class InjectKind(IntEnum):
    CLEAR_ALL = 0
    OVERHEAT = 1
    UNDERVOLTAGE = 2
    ENCODER_GLITCH = 3
    JAM = 4
    CURRENT_NOISE = 5
    DROP_STATUS = 6


FAULTS = [
    (1 << 0, "OVERCURRENT"),
    (1 << 1, "OVERTEMP"),
    (1 << 2, "UNDERVOLTAGE"),
    (1 << 3, "CMD_TIMEOUT"),
    (1 << 4, "FOLLOWING_ERROR"),
    (1 << 5, "OVERSPEED"),
    (1 << 6, "ENCODER"),
    (1 << 7, "STALL"),
    (1 << 8, "ESTOP"),
]
FAULT_BIT = {name: bit for bit, name in FAULTS}

PARAM_STATUS_TEXT = {
    0: "OK",
    1: "bad index",
    2: "read-only",
    3: "out of range",
    4: "not allowed while enabled",
    5: "bad opcode",
    6: "storage error",
}

# Templates in the DBC (node 1) by function code.
_TEMPLATES = {
    FC_EMCY: "EMCY",
    FC_STATUS_FAST: "STATUS_FAST",
    FC_COMMAND: "COMMAND",
    FC_STATUS_SLOW: "STATUS_SLOW",
    FC_PARAM_RESP: "PARAM_RESP",
    FC_PARAM_REQ: "PARAM_REQ",
    FC_HEARTBEAT: "HEARTBEAT",
}


def fault_names(flags: int) -> list[str]:
    return [name for bit, name in FAULTS if flags & bit]


def fault_text(flags: int) -> str:
    return ", ".join(fault_names(flags)) or "none"


def crc8_j1850(data: bytes) -> int:
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def e2e_crc(can_id: int, payload: bytes) -> int:
    return crc8_j1850(bytes([can_id & 0xFF, (can_id >> 8) & 0xFF]) + bytes(payload))


@dataclass
class Frame:
    can_id: int
    data: bytes
    timestamp: float = field(default_factory=time.time)

    @property
    def dlc(self) -> int:
        return len(self.data)

    def hex(self) -> str:
        return self.data.hex().upper()

    def __str__(self) -> str:
        return f"{self.can_id:03X}#{self.hex()}"


@dataclass
class Decoded:
    name: str
    node: int
    signals: dict


def default_dbc_path() -> Path:
    env = os.environ.get("BRINGUP_DBC")
    if env:
        return Path(env)
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "protocol" / "actuator.dbc"
        if candidate.exists():
            return candidate
    raise FileNotFoundError("protocol/actuator.dbc not found; set BRINGUP_DBC to its path")


class Codec:
    def __init__(self, dbc_path: str | Path | None = None):
        self.db: Database = load_dbc(dbc_path or default_dbc_path())

    # --- decoding -----------------------------------------------------------
    def message_for(self, can_id: int):
        """Return (DBC message, node) for a frame ID, or (None, 0)."""
        if can_id == ID_ESTOP:
            return self.db.by_name("ESTOP"), 0
        if can_id == ID_FAULT_INJECT:
            return self.db.by_name("FAULT_INJECT"), 0
        fc, node = can_id & 0x780, can_id & 0x7F
        name = _TEMPLATES.get(fc)
        if name is None or node == 0:
            return None, 0
        return self.db.by_name(name), node

    def decode(self, frame: Frame) -> Decoded | None:
        msg, node = self.message_for(frame.can_id)
        if msg is None or len(frame.data) != msg.length:
            return None
        return Decoded(msg.name, node, msg.decode(frame.data))

    def name_of(self, can_id: int) -> str:
        msg, node = self.message_for(can_id)
        if msg is None:
            return "?"
        return msg.name if node == 0 else f"{msg.name}[{node}]"

    # --- encoding -----------------------------------------------------------
    def encode(self, name: str, node_id: int, /, **signals) -> Frame:
        msg = self.db.by_name(name)
        can_id = msg.frame_id if node_id == 0 else (msg.frame_id - 1) + node_id
        return Frame(can_id, msg.encode(signals))

    def command(self, node: int, mode: Mode, setpoint: float, enable: bool, clear_faults: bool,
                counter: int) -> Frame:
        f = self.encode("COMMAND", node, mode=int(mode), enable=int(enable), clear_faults=int(clear_faults),
                        setpoint=setpoint, counter=counter & 0xFF, crc=0)
        data = bytearray(f.data)
        data[7] = e2e_crc(f.can_id, data[:7])
        return Frame(f.can_id, bytes(data))

    def estop(self, counter: int) -> Frame:
        f = self.encode("ESTOP", 0, magic=ESTOP_MAGIC, counter=counter & 0xFF, crc=0)
        data = bytearray(f.data)
        data[2] = e2e_crc(f.can_id, data[:2])
        return Frame(f.can_id, bytes(data))

    def param_request(self, node: int, op: ParamOp, index: int, value: float = 0.0) -> Frame:
        return self.encode("PARAM_REQ", node, opcode=int(op), index=index, status=0, value=value)

    def fault_inject(self, node: int, kind: InjectKind, value: float = 0.0) -> Frame:
        return self.encode("FAULT_INJECT", 0, node=node, kind=int(kind), value=value)


# --- Bus timing ---------------------------------------------------------------

def _crc15(bits: list[int]) -> int:
    crc = 0
    for b in bits:
        nxt = b ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if nxt:
            crc ^= 0x4599
    return crc


def frame_bits(can_id: int, data: bytes) -> int:
    """Exact on-wire length of a classic CAN frame with an 11-bit ID,
    including stuff bits, the EOF and the 3-bit interframe space."""
    dlc = len(data)
    bits = [0]  # SOF
    bits += [(can_id >> (10 - i)) & 1 for i in range(11)]
    bits += [0, 0, 0]  # RTR, IDE, r0
    bits += [(dlc >> (3 - i)) & 1 for i in range(4)]
    for byte in data:
        bits += [(byte >> (7 - i)) & 1 for i in range(8)]
    crc = _crc15(bits)
    bits += [(crc >> (14 - i)) & 1 for i in range(15)]
    stuffed = 0
    run_bit, run_len = None, 0
    for b in bits:
        if b == run_bit:
            run_len += 1
        else:
            run_bit, run_len = b, 1
        if run_len == 5:
            stuffed += 1
            run_bit, run_len = 1 - b, 1  # the stuff bit starts a new run
    return len(bits) + stuffed + 1 + 2 + 7 + 3  # CRC delim, ACK slot+delim, EOF, IFS
