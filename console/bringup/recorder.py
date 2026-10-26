"""Data collection and log replay.

Recording writes two files side by side:
  <name>.log  every frame on the bus in candump format, readable by
              can-utils and `cantools decode`:  (1730000000.123456) can0 181#00000000...
  <name>.csv  decoded fast status per node, one row per STATUS_FAST frame

analyze_log() replays a .log through an offline DeviceManager and
summarises it: rates, faults, ranges, rejected commands.
"""

from __future__ import annotations

import csv
import math
import re
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from .devices import DeviceManager
from .protocol import Codec, Decoded, Frame, fault_text

_CANDUMP = re.compile(r"^\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]{3,8})#([0-9A-Fa-f]*)")


def format_candump(frame: Frame, channel: str = "can0") -> str:
    return f"({frame.timestamp:.6f}) {channel} {frame.can_id:03X}#{frame.data.hex().upper()}"


def read_candump(path: str | Path):
    with open(path, encoding="ascii", errors="replace") as fh:
        for line in fh:
            m = _CANDUMP.match(line.strip())
            if m:
                yield Frame(int(m.group(3), 16), bytes.fromhex(m.group(4)), float(m.group(1)))


class Recorder:
    CSV_FIELDS = ["time", "node", "position_rad", "velocity_rad_s", "current_a", "setpoint", "state", "mode",
                  "fault_flags", "motor_temp_c", "bus_voltage_v"]

    def __init__(self, dm: DeviceManager):
        self.dm = dm
        self._lock = threading.Lock()
        self._log = None
        self._csv = None
        self._writer = None
        self.frames = 0
        self.rows = 0
        self.paths: tuple[Path, Path] | None = None
        self.started = 0.0

    @property
    def active(self) -> bool:
        return self._log is not None

    def start(self, directory: str | Path = "recordings", name: str | None = None) -> tuple[Path, Path]:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        name = name or datetime.now().strftime("bus_%Y%m%d_%H%M%S")
        log_path, csv_path = directory / f"{name}.log", directory / f"{name}.csv"
        with self._lock:
            self._log = open(log_path, "w", encoding="ascii")
            self._csv = open(csv_path, "w", newline="")
            self._writer = csv.writer(self._csv)
            self._writer.writerow(self.CSV_FIELDS)
            self.frames = self.rows = 0
            self.started = time.time()
        self.paths = (log_path, csv_path)
        self.dm.add_listener(self._on_frame)
        return self.paths

    def stop(self) -> tuple[Path, Path] | None:
        self.dm.remove_listener(self._on_frame)
        with self._lock:
            for fh in (self._log, self._csv):
                if fh:
                    fh.close()
            self._log = self._csv = self._writer = None
        return self.paths

    def _on_frame(self, frame: Frame, direction: str, decoded: Decoded | None) -> None:
        with self._lock:
            if self._log is None:
                return
            self._log.write(format_candump(frame) + "\n")
            self.frames += 1
            if decoded is not None and decoded.name == "STATUS_FAST":
                ns = self.dm.nodes.get(decoded.node)
                s = decoded.signals
                self._writer.writerow([
                    f"{frame.timestamp:.6f}", decoded.node, s["position"], s["velocity"], s["current"],
                    "" if ns is None or not ns.cmd_enable else ns.cmd_setpoint,
                    ns.state_name if ns else "", ns.mode_name if ns else "",
                    ns.fault_flags if ns else "", ns.motor_temp if ns else "", ns.bus_voltage if ns else "",
                ])
                self.rows += 1


@dataclass
class LogSummary:
    path: str
    frames: int
    duration_s: float
    rates: dict
    nodes: dict
    faults: list
    dm: DeviceManager

    def to_text(self) -> str:
        lines = [f"{self.path}: {self.frames} frames over {self.duration_s:.1f} s"]
        lines.append("Message rates:")
        for name, hz in sorted(self.rates.items()):
            lines.append(f"  {name:<22} {hz:7.1f} Hz")
        for n, info in sorted(self.nodes.items()):
            lines.append(f"Node {n}: position {info['pos_min']:+.3f} .. {info['pos_max']:+.3f} rad, "
                         f"|velocity| max {info['vel_absmax']:.3f} rad/s, |current| max {info['cur_absmax']:.3f} A, "
                         f"winding max {info['temp_max']:.1f} C, rejected commands {info['e2e_errors']}")
        if self.faults:
            lines.append("Faults:")
            for t, node, text in self.faults:
                lines.append(f"  +{t:8.3f} s  node {node}: {text}")
        else:
            lines.append("Faults: none")
        return "\n".join(lines)


def analyze_log(path: str | Path, codec: Codec | None = None) -> LogSummary:
    dm = DeviceManager(link=None, codec=codec)
    frames = list(read_candump(path))
    if not frames:
        raise ValueError(f"{path}: no candump frames found")
    for f in frames:
        dm.feed(f, "rx")
    t0, t1 = frames[0].timestamp, frames[-1].timestamp
    duration = max(t1 - t0, 1e-9)
    counts: dict[str, int] = {}
    for f in frames:
        name = dm.codec.name_of(f.can_id)
        counts[name] = counts.get(name, 0) + 1
    nodes = {}
    for n, ns in dm.nodes.items():
        if not ns.pos:
            continue
        nodes[n] = {
            "pos_min": min(ns.pos), "pos_max": max(ns.pos),
            "vel_absmax": max(abs(v) for v in ns.vel), "cur_absmax": max(abs(c) for c in ns.cur),
            "temp_max": max(ns.temp) if ns.temp else math.nan, "e2e_errors": ns.e2e_errors,
        }
    faults = [(e.t - t0, e.node, e.text) for e in dm.events if e.kind == "fault"]
    return LogSummary(str(path), len(frames), duration, {k: v / duration for k, v in counts.items()}, nodes, faults, dm)


__all__ = ["Recorder", "analyze_log", "read_candump", "format_candump", "LogSummary", "fault_text"]
