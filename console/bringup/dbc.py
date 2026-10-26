"""A small DBC reader/encoder for the subset this project uses.

It reads BO_ (messages), SG_ (signals), SIG_VALTYPE_ (float signals),
VAL_ (value names) and CM_ (comments), and encodes/decodes little-endian
(Intel) signals. That keeps the console dependency-light. The tests
cross-check every message against cantools when it is installed.
"""

from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass, field
from pathlib import Path

_BO = re.compile(r"^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+)")
_SG = re.compile(
    r"^SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(([^,]+),([^)]+)\)\s*\[([^|]*)\|([^\]]*)\]\s*\"([^\"]*)\""
)
_VALTYPE = re.compile(r"^SIG_VALTYPE_\s+(\d+)\s+(\w+)\s*:\s*(\d)\s*;")
_VAL = re.compile(r"^VAL_\s+(\d+)\s+(\w+)\s+(.*);")
_CM_BO = re.compile(r'^CM_\s+BO_\s+(\d+)\s+"(.*)"\s*;', re.S)
_CM_SG = re.compile(r'^CM_\s+SG_\s+(\d+)\s+(\w+)\s+"(.*)"\s*;', re.S)


@dataclass
class Signal:
    name: str
    start: int
    length: int
    little_endian: bool
    signed: bool
    factor: float
    offset: float
    minimum: float
    maximum: float
    unit: str
    is_float: bool = False
    choices: dict[int, str] = field(default_factory=dict)
    comment: str = ""

    def decode(self, raw_int: int):
        raw = (raw_int >> self.start) & ((1 << self.length) - 1)
        if self.is_float:
            if self.length != 32:
                raise ValueError(f"{self.name}: only 32-bit floats are supported")
            return struct.unpack("<f", raw.to_bytes(4, "little"))[0]
        if self.signed and raw & (1 << (self.length - 1)):
            raw -= 1 << self.length
        if self.factor == 1 and self.offset == 0:
            return raw
        return raw * self.factor + self.offset

    def encode(self, value) -> int:
        if self.is_float:
            return int.from_bytes(struct.pack("<f", float(value)), "little") << self.start
        if isinstance(value, str):
            names = {v: k for k, v in self.choices.items()}
            value = names[value]
        raw = round((float(value) - self.offset) / self.factor)
        if self.signed:
            lo, hi = -(1 << (self.length - 1)), (1 << (self.length - 1)) - 1
        else:
            lo, hi = 0, (1 << self.length) - 1
        raw = max(lo, min(hi, raw))
        return (raw & ((1 << self.length) - 1)) << self.start


@dataclass
class Message:
    frame_id: int
    name: str
    length: int
    sender: str
    signals: list[Signal] = field(default_factory=list)
    comment: str = ""

    def signal(self, name: str) -> Signal:
        for s in self.signals:
            if s.name == name:
                return s
        raise KeyError(f"{self.name} has no signal {name}")

    def decode(self, data: bytes) -> dict:
        raw = int.from_bytes(bytes(data).ljust(8, b"\0"), "little")
        return {s.name: s.decode(raw) for s in self.signals}

    def encode(self, values: dict) -> bytes:
        raw = 0
        for s in self.signals:
            raw |= s.encode(values.get(s.name, 0))
        return raw.to_bytes(8, "little")[: self.length]


class Database:
    def __init__(self, messages: list[Message]):
        self.messages = messages
        self._by_id = {m.frame_id: m for m in messages}
        self._by_name = {m.name: m for m in messages}

    def by_id(self, frame_id: int) -> Message | None:
        return self._by_id.get(frame_id)

    def by_name(self, name: str) -> Message:
        return self._by_name[name]


def _number(text: str) -> float:
    text = text.strip()
    try:
        return float(text)
    except ValueError:
        return math.nan


def load_dbc(path: str | Path) -> Database:
    text = Path(path).read_text(encoding="utf-8")
    messages: list[Message] = []
    by_id: dict[int, Message] = {}
    current: Message | None = None

    # CM_ entries can span lines; join each statement up to its ';'.
    statements: list[str] = []
    buf = ""
    for line in text.splitlines():
        stripped = line.strip()
        if buf:
            buf += "\n" + line
            if stripped.endswith(";"):
                statements.append(buf.strip())
                buf = ""
            continue
        if stripped.startswith("CM_ ") and not stripped.endswith(";"):
            buf = stripped
            continue
        statements.append(stripped)

    for st in statements:
        if m := _BO.match(st):
            current = Message(int(m.group(1)), m.group(2), int(m.group(3)), m.group(4))
            messages.append(current)
            by_id[current.frame_id] = current
        elif (m := _SG.match(st)) and current is not None:
            (name, start, length, order, sign, factor, offset, lo, hi, unit) = m.groups()
            if order != "1":
                raise ValueError(f"signal {name}: big-endian signals are not supported")
            current.signals.append(
                Signal(name, int(start), int(length), True, sign == "-", float(factor), float(offset),
                       _number(lo), _number(hi), unit)
            )
        elif m := _VALTYPE.match(st):
            msg = by_id.get(int(m.group(1)))
            if msg and m.group(3) == "1":
                msg.signal(m.group(2)).is_float = True
        elif m := _VAL.match(st):
            msg = by_id.get(int(m.group(1)))
            if msg:
                pairs = re.findall(r'(-?\d+)\s+"([^"]*)"', m.group(3))
                msg.signal(m.group(2)).choices = {int(k): v for k, v in pairs}
        elif m := _CM_SG.match(st):
            msg = by_id.get(int(m.group(1)))
            if msg:
                msg.signal(m.group(2)).comment = m.group(3)
        elif m := _CM_BO.match(st):
            msg = by_id.get(int(m.group(1)))
            if msg:
                msg.comment = m.group(2)
        else:
            current = current if st.startswith("SG_") else None
    return Database(messages)
