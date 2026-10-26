"""Display formatting for the GUI, kept free of Qt so it can be unit tested."""

from __future__ import annotations

import math
import time

from .devices import Event, NodeState
from .protocol import InjectKind, Mode, State, fault_text

NODE_COLUMNS = ["Node", "Link", "State", "Mode", "Homed", "Faults", "Position (rad)", "Velocity (rad/s)",
                "Current (A)", "Winding (C)", "Bus (V)", "Rejected cmds"]

SEVERITY_COLORS = {
    "ok": "#2e7d32",
    "warn": "#b26a00",
    "fault": "#c62828",
    "muted": "#757575",
    "info": "#1565c0",
    "action": "#6a1b9a",
    "state": "#37474f",
}

MODE_RANGES = {
    Mode.CURRENT: (-5.0, 5.0, 0.05, "A"),
    Mode.VELOCITY: (-3.0, 3.0, 0.1, "rad/s"),
    Mode.POSITION: (-2.4, 2.4, 0.05, "rad"),
}

INJECT_HELP = {
    InjectKind.OVERHEAT: ("Extra heat (W)", 60.0,
                          "Adds heat to the winding, as a blocked cooling path would. Watch the winding temperature "
                          "climb; at overtemp_c the node trips OVERTEMP and cannot be cleared until it cools 5 C."),
    InjectKind.UNDERVOLTAGE: ("Supply (V)", 36.0,
                              "Drops the 48 V supply. Below undervoltage_v for 10 ms the node trips UNDERVOLTAGE."),
    InjectKind.ENCODER_GLITCH: ("Jump (rad)", 0.2,
                                "Makes the encoder jump, as lost counts or a noisy cable would. The node trips "
                                "ENCODER and forgets its homing: re-home before using position mode."),
    InjectKind.JAM: ("Friction (Nm)", 40.0,
                     "Jams the joint. Enabled in velocity or position mode, it trips FOLLOWING_ERROR (or STALL)."),
    InjectKind.CURRENT_NOISE: ("Noise (A rms)", 0.2,
                               "Adds current-sensor noise. Run the current-offset calibration to see its effect."),
    InjectKind.DROP_STATUS: ("Seconds", 3.0,
                             "The node stops sending fast status for a while. The bus monitor shows the rate drop "
                             "and the plots go flat."),
    InjectKind.CLEAR_ALL: ("-", 0.0, "Removes every injected condition (an encoder jump stays: counts are lost)."),
}


def fmt(value: float, digits: int = 3) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "-"
    return f"{value:+.{digits}f}"


def node_row(ns: NodeState, now: float | None = None) -> tuple[list[str], str]:
    """Cells for the node table and a severity for the row colour."""
    now = now if now is not None else time.time()
    alive = ns.online and now - ns.last_heartbeat < 1.5
    cells = [
        str(ns.node),
        "online" if alive else "LOST" if ns.last_heartbeat else "-",
        ns.state_name,
        ns.mode_name,
        "yes" if ns.homed else "no",
        fault_text(ns.fault_flags) if ns.fault_flags else "none",
        fmt(ns.position, 4),
        fmt(ns.velocity, 3),
        fmt(ns.current, 3),
        "-" if math.isnan(ns.motor_temp) else f"{ns.motor_temp:.1f}",
        "-" if math.isnan(ns.bus_voltage) else f"{ns.bus_voltage:.2f}",
        str(ns.e2e_errors),
    ]
    if not alive and ns.last_heartbeat:
        severity = "fault"
    elif ns.state == State.FAULT or ns.fault_flags:
        severity = "fault"
    elif ns.state == State.ENABLED:
        severity = "ok"
    elif ns.e2e_errors:
        severity = "warn"
    else:
        severity = "muted"
    return cells, severity


def event_line(e: Event, t0: float | None = None) -> tuple[str, str]:
    stamp = time.strftime("%H:%M:%S", time.localtime(e.t)) + f".{int((e.t % 1) * 1000):03d}"
    who = f"node {e.node}" if e.node else "host"
    return f"{stamp}  {who:<7} {e.text}", SEVERITY_COLORS.get(e.kind, "#000000")


def bus_row(row: dict) -> list[str]:
    decoded = ""
    if row.get("decoded"):
        parts = []
        for k, v in row["decoded"].items():
            parts.append(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}")
        decoded = " ".join(parts)
    return [f"0x{row['id']:03X}", row["name"], row["dir"], str(row["count"]), f"{row['rate']:.1f}",
            row["data"], decoded]


def check_color(passed: bool) -> str:
    return "#e8f5e9" if passed else "#ffebee"
