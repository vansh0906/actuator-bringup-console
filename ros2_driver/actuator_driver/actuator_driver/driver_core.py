"""ROS-independent logic of the driver: joint states, command handling,
the host-side command timeout and diagnostics. driver_node.py wraps it in
rclpy; tests drive it directly against the simulator, no ROS needed."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from bringup.protocol import Mode, State, fault_text

LEVEL_OK, LEVEL_WARN, LEVEL_ERROR, LEVEL_STALE = 0, 1, 2, 3


@dataclass
class DiagStatus:
    level: int
    name: str
    message: str
    hardware_id: str
    values: dict = field(default_factory=dict)


class DriverCore:
    def __init__(self, dm, pc, node_ids: list[int], joint_names: list[str], mode: str = "velocity",
                 torque_constant: float = 4.0, command_timeout: float = 0.5):
        if len(node_ids) != len(joint_names):
            raise ValueError("node_ids and joint_names must have the same length")
        self.dm = dm
        self.pc = pc
        self.node_ids = list(node_ids)
        self.joint_names = list(joint_names)
        self.mode = {"velocity": Mode.VELOCITY, "position": Mode.POSITION, "current": Mode.CURRENT}[mode]
        self.torque_constant = torque_constant  # Nm per A at the joint (Kt x gear ratio)
        self.command_timeout = command_timeout
        self.last_command_time = 0.0
        self.timed_out = True

    # --- state out ------------------------------------------------------------------
    def joint_state(self):
        names, pos, vel, eff = [], [], [], []
        with self.dm.lock:
            for n, name in zip(self.node_ids, self.joint_names):
                ns = self.dm.nodes.get(n)
                names.append(name)
                if ns is None:
                    pos.append(math.nan)
                    vel.append(math.nan)
                    eff.append(math.nan)
                    continue
                pos.append(ns.position)
                vel.append(ns.velocity)
                eff.append(ns.current * self.torque_constant)
        return names, pos, vel, eff

    def diagnostics(self, now: float | None = None) -> list[DiagStatus]:
        now = now if now is not None else time.time()
        out = []
        with self.dm.lock:
            for n, name in zip(self.node_ids, self.joint_names):
                ns = self.dm.nodes.get(n)
                hw = f"can node {n}"
                if ns is None or not ns.last_heartbeat:
                    out.append(DiagStatus(LEVEL_STALE, f"actuator_driver: {name}", "no data from actuator", hw))
                    continue
                age = now - ns.last_heartbeat
                if age > 1.5:
                    level, msg = LEVEL_STALE, f"no heartbeat for {age:.1f} s"
                elif ns.fault_flags or ns.state == State.FAULT:
                    level, msg = LEVEL_ERROR, f"FAULT: {fault_text(ns.fault_flags)}"
                elif not ns.homed:
                    level, msg = LEVEL_WARN, f"{ns.state_name}, not homed"
                elif ns.motor_temp > 70.0:
                    level, msg = LEVEL_WARN, f"{ns.state_name}, winding hot ({ns.motor_temp:.0f} C)"
                else:
                    level, msg = LEVEL_OK, ns.state_name
                out.append(DiagStatus(level, f"actuator_driver: {name}", msg, hw, {
                    "state": ns.state_name,
                    "mode": ns.mode_name,
                    "position_rad": f"{ns.position:.4f}",
                    "velocity_rad_s": f"{ns.velocity:.3f}",
                    "current_a": f"{ns.current:.3f}",
                    "winding_temp_c": f"{ns.motor_temp:.1f}",
                    "bus_voltage_v": f"{ns.bus_voltage:.2f}",
                    "faults": fault_text(ns.fault_flags),
                    "homed": str(ns.homed),
                    "rejected_commands": str(ns.e2e_errors),
                    "status_rate_hz": f"{ns.fast_rate.rate(now):.0f}",
                }))
        return out

    # --- commands in ----------------------------------------------------------------
    def apply_commands(self, values: list[float], now: float | None = None) -> str | None:
        """Forward one setpoint per joint (ros2_control forward-controller
        style). Returns an error message, or None when applied."""
        if len(values) != len(self.node_ids):
            return f"expected {len(self.node_ids)} values, got {len(values)}"
        if any(math.isnan(v) or math.isinf(v) for v in values):
            return "command contains NaN or inf"
        for n, v in zip(self.node_ids, values):
            self.dm.set_command(n, self.mode, float(v))
        self.last_command_time = now if now is not None else time.time()
        self.timed_out = False
        return None

    def check_command_timeout(self, now: float | None = None) -> bool:
        """If ROS stops sending commands, stop the joints (velocity/current go
        to zero, position holds where it is). Returns True when it fired."""
        now = now if now is not None else time.time()
        if self.timed_out or now - self.last_command_time < self.command_timeout:
            return False
        self.timed_out = True
        for n in self.node_ids:
            ns = self.dm.node(n)
            hold = ns.position if self.mode == Mode.POSITION and not math.isnan(ns.position) else 0.0
            self.dm.set_command(n, self.mode, hold)
        return True

    # --- services ---------------------------------------------------------------------
    def enable(self) -> tuple[bool, str]:
        for n in self.node_ids:
            ns = self.dm.node(n)
            if ns.fault_flags:
                return False, f"node {n} has faults ({fault_text(ns.fault_flags)}): call clear_faults first"
            hold = ns.position if self.mode == Mode.POSITION and not math.isnan(ns.position) else 0.0
            self.dm.enable(n, self.mode, hold)
        ok = self.dm.wait_until(lambda: all(self.dm.nodes[n].state == State.ENABLED for n in self.node_ids), 1.0)
        return ok, "enabled" if ok else "not all joints enabled: " + ", ".join(
            f"node {n} {self.dm.node(n).state_name}" for n in self.node_ids)

    def disable(self) -> tuple[bool, str]:
        for n in self.node_ids:
            self.dm.disable(n)
        ok = self.dm.wait_until(lambda: all(self.dm.nodes[n].state != State.ENABLED for n in self.node_ids), 1.0)
        return ok, "disabled" if ok else "a joint did not disable"

    def clear_faults(self) -> tuple[bool, str]:
        for n in self.node_ids:
            self.dm.clear_faults(n)
        ok = self.dm.wait_until(lambda: all(self.dm.nodes[n].fault_flags == 0 for n in self.node_ids), 1.5)
        if ok:
            return True, "faults cleared"
        return False, "faults remain: " + ", ".join(
            f"node {n} {fault_text(self.dm.node(n).fault_flags)}" for n in self.node_ids if self.dm.node(n).fault_flags)

    def estop(self) -> tuple[bool, str]:
        self.dm.estop()
        return True, "E-stop sent"
