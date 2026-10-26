"""DeviceManager: live state of every actuator on the bus, plus the
command stream that keeps enabled actuators fed.

- A reader thread (in the link) delivers frames to on_frame(), which decodes
  them and updates each node's state, history, event log and bus statistics.
- A command thread sends COMMAND frames at 100 Hz to every armed node, with a
  fresh alive counter and CRC each time. If the host stops (crash, frozen
  GUI), the actuator's watchdog disables it within cmd_timeout_ms.
- The same class replays a recorded log offline (no link): feed() frames
  with their original timestamps.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

from .protocol import (
    FC_COMMAND,
    FC_EMCY,
    ID_ESTOP,
    Codec,
    Decoded,
    Frame,
    InjectKind,
    Mode,
    State,
    fault_text,
    frame_bits,
)

HISTORY_FAST = 6000  # 60 s at 100 Hz
HISTORY_SLOW = 600   # 60 s at 10 Hz
HEARTBEAT_TIMEOUT = 1.5  # s


class RateMeter:
    """Frames per second over the last `window` seconds."""

    def __init__(self, window: float = 1.0):
        self.window = window
        self.times: deque[float] = deque()

    def add(self, t: float) -> None:
        self.times.append(t)
        self._trim(t)

    def _trim(self, now: float) -> None:
        while self.times and self.times[0] < now - self.window:
            self.times.popleft()

    def rate(self, now: float) -> float:
        self._trim(now)
        return len(self.times) / self.window


@dataclass
class NodeState:
    node: int
    # Latest telemetry
    position: float = math.nan
    velocity: float = math.nan
    current: float = math.nan
    bus_voltage: float = math.nan
    motor_temp: float = math.nan
    fault_flags: int = 0
    state: State | None = None
    mode: Mode | None = None
    e2e_errors: int = 0
    homed: bool = False
    fw_version: str = ""
    uptime: float = 0.0
    last_fast: float = 0.0
    last_slow: float = 0.0
    last_heartbeat: float = 0.0
    online: bool = False
    fast_rate: RateMeter = field(default_factory=RateMeter)
    slow_rate: RateMeter = field(default_factory=lambda: RateMeter(2.0))
    # History for plots
    t: deque = field(default_factory=lambda: deque(maxlen=HISTORY_FAST))
    pos: deque = field(default_factory=lambda: deque(maxlen=HISTORY_FAST))
    vel: deque = field(default_factory=lambda: deque(maxlen=HISTORY_FAST))
    cur: deque = field(default_factory=lambda: deque(maxlen=HISTORY_FAST))
    sp: deque = field(default_factory=lambda: deque(maxlen=HISTORY_FAST))
    slow_t: deque = field(default_factory=lambda: deque(maxlen=HISTORY_SLOW))
    temp: deque = field(default_factory=lambda: deque(maxlen=HISTORY_SLOW))
    vbus: deque = field(default_factory=lambda: deque(maxlen=HISTORY_SLOW))
    # Host-side command
    armed: bool = False
    suspended: bool = False
    cmd_enable: bool = False
    cmd_mode: Mode = Mode.IDLE
    cmd_setpoint: float = 0.0
    clear_pending: int = 0
    counter: int = 0
    last_cmd_tx: float = 0.0

    @property
    def state_name(self) -> str:
        return self.state.name if self.state is not None else "?"

    @property
    def mode_name(self) -> str:
        return self.mode.name if self.mode is not None else "?"


@dataclass
class IdStat:
    can_id: int
    name: str
    direction: str
    count: int = 0
    rate: RateMeter = field(default_factory=RateMeter)
    last_data: bytes = b""
    last_t: float = 0.0
    bits: int = 0
    decoded: dict | None = None


@dataclass
class Event:
    t: float
    node: int
    kind: str  # "fault", "state", "info", "warn", "action"
    text: str


Listener = Callable[[Frame, str, "Decoded | None"], None]


class DeviceManager:
    def __init__(self, link=None, codec: Codec | None = None, command_rate_hz: float = 100.0):
        self.codec = codec or Codec()
        self.link = link
        if link is not None:
            link.on_frame = self.on_frame
        self.command_period = 1.0 / command_rate_hz
        self.nodes: dict[int, NodeState] = {}
        self.events: deque[Event] = deque(maxlen=1000)
        self.id_stats: dict[tuple[int, str], IdStat] = {}
        self.lock = threading.RLock()
        self._listeners: list[Listener] = []
        self._param_waiters: dict[tuple[int, int, int], list] = {}
        self._emcy_log: deque[tuple[float, int, int]] = deque(maxlen=500)  # (t, node, fault_code)
        self._thread: threading.Thread | None = None
        self._running = False
        self._estop_counter = 0
        self.tx_errors = 0
        self.frame_meter = RateMeter()
        self.bits_meter: deque[tuple[float, int]] = deque()

    # --- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._command_loop, name="command-loop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def add_listener(self, fn: Listener) -> None:
        self._listeners.append(fn)

    def remove_listener(self, fn: Listener) -> None:
        if fn in self._listeners:
            self._listeners.remove(fn)

    # --- frames in/out ------------------------------------------------------
    def on_frame(self, frame: Frame) -> None:
        self.feed(frame, "rx")

    def send(self, frame: Frame) -> None:
        if self.link is None:
            raise RuntimeError("no link: this DeviceManager is offline")
        frame.timestamp = time.time()
        try:
            self.link.send(frame)
        except Exception:
            self.tx_errors += 1
            raise
        self.feed(frame, "tx")

    def feed(self, frame: Frame, direction: str = "rx") -> Decoded | None:
        decoded = self.codec.decode(frame)
        t = frame.timestamp
        with self.lock:
            key = (frame.can_id, direction)
            stat = self.id_stats.get(key)
            if stat is None:
                stat = IdStat(frame.can_id, self.codec.name_of(frame.can_id), direction,
                              bits=frame_bits(frame.can_id, frame.data))
                self.id_stats[key] = stat
            stat.count += 1
            stat.rate.add(t)
            stat.last_data = frame.data
            stat.last_t = t
            stat.decoded = decoded.signals if decoded else None
            self.frame_meter.add(t)
            self.bits_meter.append((t, stat.bits))
            while self.bits_meter and self.bits_meter[0][0] < t - 1.0:
                self.bits_meter.popleft()
            if decoded is not None and direction == "rx":
                self._apply(decoded, t)
        for fn in list(self._listeners):
            try:
                fn(frame, direction, decoded)
            except Exception as exc:  # a broken listener must not kill the reader thread
                self.log(0, "warn", f"listener error: {exc}")
        return decoded

    # --- decoding into state -----------------------------------------------
    def _node(self, n: int) -> NodeState:
        ns = self.nodes.get(n)
        if ns is None:
            ns = NodeState(n)
            self.nodes[n] = ns
        return ns

    def log(self, node: int, kind: str, text: str, t: float | None = None) -> None:
        with self.lock:
            self.events.append(Event(t if t is not None else time.time(), node, kind, text))

    def _apply(self, d: Decoded, t: float) -> None:
        s = d.signals
        if d.name == "STATUS_FAST":
            ns = self._node(d.node)
            ns.position, ns.velocity, ns.current = s["position"], s["velocity"], s["current"]
            ns.last_fast = t
            ns.fast_rate.add(t)
            ns.t.append(t)
            ns.pos.append(ns.position)
            ns.vel.append(ns.velocity)
            ns.cur.append(ns.current)
            ns.sp.append(ns.cmd_setpoint if (ns.cmd_enable and ns.armed) else math.nan)
        elif d.name == "STATUS_SLOW":
            ns = self._node(d.node)
            state, mode = State(s["state"]) if s["state"] <= 3 else None, Mode(s["mode"]) if s["mode"] <= 3 else None
            if state != ns.state and ns.state is not None:
                self.log(d.node, "state", f"{ns.state_name} -> {state.name if state else '?'}", t)
            if s["fault_flags"] != ns.fault_flags and s["fault_flags"] == 0 and ns.fault_flags:
                self.log(d.node, "info", "faults cleared", t)
            if s["e2e_errors"] > ns.e2e_errors:
                self.log(d.node, "warn", f"node dropped {s['e2e_errors'] - ns.e2e_errors} bad command(s) "
                                         f"(total {s['e2e_errors']})", t)
            ns.state, ns.mode = state, mode
            ns.bus_voltage, ns.motor_temp = s["bus_voltage"], s["motor_temp"]
            ns.fault_flags, ns.e2e_errors = s["fault_flags"], s["e2e_errors"]
            ns.last_slow = t
            ns.slow_rate.add(t)
            ns.slow_t.append(t)
            ns.temp.append(ns.motor_temp)
            ns.vbus.append(ns.bus_voltage)
        elif d.name == "HEARTBEAT":
            ns = self._node(d.node)
            if not ns.online:
                self.log(d.node, "info", f"node {d.node} online (firmware {s['fw_major']}.{s['fw_minor']})", t)
            ns.online = True
            ns.state = State(s["state"]) if s["state"] <= 3 else ns.state
            ns.uptime = s["uptime"]
            ns.fw_version = f"{s['fw_major']}.{s['fw_minor']}"
            ns.homed = bool(s["homed"])
            ns.last_heartbeat = t
        elif d.name == "EMCY":
            ns = self._node(d.node)
            ns.fault_flags = s["fault_flags"]
            ns.state = State.FAULT
            self._emcy_log.append((t, d.node, s["fault_code"]))
            self.log(d.node, "fault", f"FAULT {fault_text(s['fault_code'])} "
                                      f"(temp {s['motor_temp']:.1f} C, bus {s['bus_voltage']:.1f} V)", t)
        elif d.name == "PARAM_RESP":
            key = (d.node, s["opcode"], s["index"])
            waiters = self._param_waiters.get(key)
            if waiters:
                event, holder = waiters.pop(0)
                holder.append((s["status"], s["value"]))
                event.set()

    # --- parameter requests (used by ParamClient) ---------------------------
    def param_transaction(self, frame: Frame, node: int, opcode: int, index: int, timeout: float):
        event = threading.Event()
        holder: list = []
        key = (node, opcode, index)
        with self.lock:
            self._param_waiters.setdefault(key, []).append((event, holder))
        self.send(frame)
        if not event.wait(timeout):
            with self.lock:
                waiters = self._param_waiters.get(key, [])
                if (event, holder) in waiters:
                    waiters.remove((event, holder))
            return None
        return holder[0]

    # --- commands -------------------------------------------------------------
    def node(self, n: int) -> NodeState:
        with self.lock:
            return self._node(n)

    def node_ids(self) -> list[int]:
        with self.lock:
            return sorted(n for n, ns in self.nodes.items() if ns.online or ns.last_fast)

    def arm(self, n: int) -> None:
        with self.lock:
            self._node(n).armed = True

    def release(self, n: int) -> None:
        with self.lock:
            ns = self._node(n)
            ns.armed = False
            ns.cmd_enable = False

    def set_command(self, n: int, mode: Mode | None = None, setpoint: float | None = None) -> None:
        with self.lock:
            ns = self._node(n)
            ns.armed = True
            if mode is not None:
                ns.cmd_mode = Mode(mode)
            if setpoint is not None:
                ns.cmd_setpoint = float(setpoint)

    def enable(self, n: int, mode: Mode | None = None, setpoint: float | None = None) -> None:
        with self.lock:
            ns = self._node(n)
            if mode is not None:
                ns.cmd_mode = Mode(mode)
            if setpoint is not None:
                ns.cmd_setpoint = float(setpoint)
            elif ns.cmd_mode == Mode.POSITION and not math.isnan(ns.position):
                ns.cmd_setpoint = ns.position  # hold where it is: no jump on enable
            elif ns.cmd_mode in (Mode.VELOCITY, Mode.CURRENT):
                ns.cmd_setpoint = 0.0
            ns.armed = True
            ns.cmd_enable = True
            ns.clear_pending = 0
        self.log(n, "action", f"enable {ns.cmd_mode.name} setpoint {ns.cmd_setpoint:+.3f}")

    def disable(self, n: int) -> None:
        with self.lock:
            ns = self._node(n)
            was = ns.cmd_enable
            ns.cmd_enable = False
        if was:
            self.log(n, "action", "disable")

    def clear_faults(self, n: int) -> None:
        with self.lock:
            ns = self._node(n)
            ns.armed = True
            ns.cmd_enable = False
            ns.clear_pending = 5  # 50 ms of clear requests
        self.log(n, "action", "clear faults")

    def suspend_commands(self, n: int, suspended: bool = True) -> None:
        with self.lock:
            self._node(n).suspended = suspended

    def estop(self) -> None:
        with self.lock:
            for ns in self.nodes.values():
                ns.cmd_enable = False
            self._estop_counter = (self._estop_counter + 1) & 0xFF
            counter = self._estop_counter
        self.send(self.codec.estop(counter))
        self.log(0, "action", "E-STOP sent")

    def inject(self, n: int, kind: InjectKind, value: float = 0.0) -> None:
        self.send(self.codec.fault_inject(n, kind, value))
        self.log(n, "action", f"inject {kind.name} {value:g}")

    def send_command_now(self, n: int) -> Frame:
        """Send one COMMAND for node n immediately (outside the 100 Hz loop)."""
        with self.lock:
            ns = self._node(n)
            ns.counter = (ns.counter + 1) & 0xFF
            clear = ns.clear_pending > 0
            if clear:
                ns.clear_pending -= 1
            frame = self.codec.command(n, ns.cmd_mode, ns.cmd_setpoint, ns.cmd_enable and not clear, clear,
                                       ns.counter)
            ns.last_cmd_tx = time.time()
        self.send(frame)
        return frame

    def _command_loop(self) -> None:
        next_t = time.monotonic()
        last_check = 0.0
        while self._running:
            with self.lock:
                targets = [n for n, ns in self.nodes.items() if ns.armed and not ns.suspended]
            for n in targets:
                try:
                    self.send_command_now(n)
                except Exception as exc:
                    self.log(n, "warn", f"command send failed: {exc}")
                    time.sleep(0.2)
            now = time.time()
            if now - last_check > 0.1:
                last_check = now
                self._check_timeouts(now)
            next_t += self.command_period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()  # fell behind: do not burst

    def _check_timeouts(self, now: float) -> None:
        with self.lock:
            for ns in self.nodes.values():
                if ns.online and now - ns.last_heartbeat > HEARTBEAT_TIMEOUT:
                    ns.online = False
                    self.log(ns.node, "fault", f"node {ns.node} lost: no heartbeat for {HEARTBEAT_TIMEOUT:.1f} s")

    # --- queries ----------------------------------------------------------------
    def wait_until(self, predicate: Callable[[], bool], timeout: float, poll: float = 0.005) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                if predicate():
                    return True
            time.sleep(poll)
        with self.lock:
            return predicate()

    def wait_emcy(self, n: int, bit: int, timeout: float, since: float) -> float | None:
        """Timestamp of the first EMCY from node n carrying `bit` after `since`."""
        def find():
            for t, node, code in self._emcy_log:
                if node == n and code & bit and t >= since:
                    return t
            return None

        found = [None]

        def pred():
            found[0] = find()
            return found[0] is not None

        self.wait_until(pred, timeout)
        return found[0]

    def history(self, n: int, seconds: float | None = None):
        """Copy of node n's fast history: (t, pos, vel, cur, sp) lists."""
        with self.lock:
            ns = self._node(n)
            t, pos, vel, cur, sp = list(ns.t), list(ns.pos), list(ns.vel), list(ns.cur), list(ns.sp)
        if seconds is not None and t:
            cutoff = t[-1] - seconds
            i = next((k for k, v in enumerate(t) if v >= cutoff), len(t))
            t, pos, vel, cur, sp = t[i:], pos[i:], vel[i:], cur[i:], sp[i:]
        return t, pos, vel, cur, sp

    def samples_since(self, n: int, since: float):
        """Fast-status samples received after `since`: (t, pos, vel, cur)."""
        with self.lock:
            ns = self._node(n)
            rows = [(t, p, v, c) for t, p, v, c in zip(ns.t, ns.pos, ns.vel, ns.cur) if t > since]
        return rows

    def bus_stats(self, now: float | None = None) -> dict:
        now = now if now is not None else time.time()
        with self.lock:
            while self.bits_meter and self.bits_meter[0][0] < now - 1.0:
                self.bits_meter.popleft()
            bits = sum(b for _, b in self.bits_meter)
            rows = sorted(self.id_stats.values(), key=lambda s: (s.can_id, s.direction))
            table = [
                {
                    "id": s.can_id,
                    "name": s.name,
                    "dir": s.direction,
                    "count": s.count,
                    "rate": s.rate.rate(now),
                    "data": s.last_data.hex().upper(),
                    "decoded": s.decoded,
                    "age": now - s.last_t,
                }
                for s in rows
            ]
            return {
                "frames_per_s": self.frame_meter.rate(now),
                "bus_load_pct": 100.0 * bits / 500_000,
                "link_errors": getattr(self.link, "errors", 0) if self.link else 0,
                "tx_errors": self.tx_errors,
                "ids": table,
            }

    def is_command_frame(self, frame: Frame) -> bool:
        return (frame.can_id & 0x780) == FC_COMMAND or frame.can_id == ID_ESTOP

    def is_emcy_frame(self, frame: Frame) -> bool:
        return (frame.can_id & 0x780) == FC_EMCY and frame.can_id != ID_ESTOP
