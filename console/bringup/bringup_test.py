"""Automated bring-up test for one actuator.

Runs the checks an engineer would do on a new or repaired joint, fixes what
it safely can (current-sensor zero, motor polarity, joint zero), and writes
a pass/fail report with every measured value next to its limit.
"""

from __future__ import annotations

import json
import statistics
import time
import traceback
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable

from . import calibration as cal
from .protocol import FAULT_BIT, Mode, State, fault_text


@dataclass
class Check:
    name: str
    passed: bool
    measured: str
    limit: str
    note: str = ""
    seconds: float = 0.0


@dataclass
class Report:
    node: int
    started: str
    checks: list[Check] = field(default_factory=list)
    firmware: str = ""
    duration_s: float = 0.0

    @property
    def passed(self) -> bool:
        return bool(self.checks) and all(c.passed for c in self.checks)

    @property
    def summary(self) -> str:
        n_pass = sum(c.passed for c in self.checks)
        return f"node {self.node}: {n_pass}/{len(self.checks)} checks passed"

    def to_markdown(self) -> str:
        lines = [
            f"# Bring-up report: node {self.node}",
            "",
            f"Started {self.started}, firmware {self.firmware or '?'}, took {self.duration_s:.1f} s.",
            f"**Result: {'PASS' if self.passed else 'FAIL'}** ({self.summary}).",
            "",
            "| # | Check | Result | Measured | Limit | Note |",
            "| --- | --- | --- | --- | --- | --- |",
        ]
        for i, c in enumerate(self.checks, 1):
            lines.append(f"| {i} | {c.name} | {'PASS' if c.passed else 'FAIL'} | {c.measured} | {c.limit} | {c.note} |")
        return "\n".join(lines) + "\n"

    def to_dict(self) -> dict:
        d = asdict(self)
        d["passed"] = self.passed
        return d

    def save(self, directory: str | Path) -> tuple[Path, Path]:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = self.started.replace(":", "").replace("-", "").replace(" ", "_")
        md = directory / f"bringup_node{self.node}_{stamp}.md"
        js = directory / f"bringup_node{self.node}_{stamp}.json"
        md.write_text(self.to_markdown())
        js.write_text(json.dumps(self.to_dict(), indent=2))
        return md, js


class BringupTest:
    STEP_OVERSHOOT_MAX = 10.0      # %
    STEP_SETTLING_MAX = 1.0        # s
    STEP_SS_ERROR_MAX = 0.005      # rad
    VELOCITY_RMS_MAX = 0.05        # rad/s
    ESTOP_LATENCY_MAX = 0.020      # s
    WATCHDOG_MARGIN = 0.050        # s past cmd_timeout_ms

    def __init__(self, dm, pc, node: int, fix: bool = True, log: Callable[[str], None] = print,
                 stop_flag: Callable[[], bool] | None = None):
        self.dm = dm
        self.pc = pc
        self.node = node
        self.fix = fix
        self.log = log
        self.stop_flag = stop_flag or (lambda: False)
        self.report = Report(node=node, started=datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    def checks(self) -> list[tuple[str, Callable[[], Check]]]:
        return [
            ("Heartbeat and firmware", self.check_heartbeat),
            ("Status message rates", self.check_rates),
            ("Parameter write/read-back", self.check_params),
            ("Supply voltage and temperature", self.check_supply),
            ("Current-sensor offset", self.check_current_offset),
            ("Motor direction", self.check_direction),
            ("Homing to hard stop", self.check_homing),
            ("Velocity tracking", self.check_velocity),
            ("Position step response", self.check_step),
            ("Command watchdog", self.check_watchdog),
            ("E-stop", self.check_estop),
            ("Corrupted commands rejected", self.check_e2e),
            ("Faults clear, calibration saved", self.check_finish),
        ]

    def run(self) -> Report:
        t_start = time.monotonic()
        self.dm.arm(self.node)
        for name, fn in self.checks():
            if self.stop_flag():
                self.report.checks.append(Check(name, False, "-", "-", "stopped by user"))
                break
            self.log(f"[node {self.node}] {name} ...")
            t0 = time.monotonic()
            try:
                check = fn()
            except Exception as exc:  # a check that crashes is a failed check
                check = Check(name, False, "-", "-", f"error: {exc}")
                self.log(traceback.format_exc(limit=2))
            check.name = name
            check.seconds = round(time.monotonic() - t0, 2)
            self.report.checks.append(check)
            self.log(f"[node {self.node}]   {'PASS' if check.passed else 'FAIL'}: {check.measured}"
                     + (f"  ({check.note})" if check.note else ""))
            if name in ("Heartbeat and firmware",) and not check.passed:
                self.log(f"[node {self.node}] node not responding: stopping")
                break
        self.dm.disable(self.node)
        self.report.duration_s = round(time.monotonic() - t_start, 1)
        self.log(f"[node {self.node}] {self.report.summary}")
        return self.report

    # --- checks ------------------------------------------------------------
    def check_heartbeat(self) -> Check:
        seen = self.dm.wait_until(lambda: self.dm.nodes.get(self.node) is not None
                                  and time.time() - self.dm.nodes[self.node].last_heartbeat < 1.0, 1.5)
        if not seen:
            return Check("", False, "no heartbeat in 1.5 s", "heartbeat every 0.5 s")
        fw = self.pc.read(self.node, "fw_version")
        node_id = self.pc.read(self.node, "node_id")
        self.report.firmware = f"{fw:.1f}"
        ok = int(node_id) == self.node
        return Check("", ok, f"firmware {fw:.1f}, node_id {int(node_id)}", "heartbeat + node_id matches")

    def check_rates(self) -> Check:
        ns = self.dm.node(self.node)
        time.sleep(2.0)
        now = time.time()
        with self.dm.lock:
            fast = ns.fast_rate.rate(now)
            slow = ns.slow_rate.rate(now)
        ok = 90 <= fast <= 110 and 8 <= slow <= 12
        return Check("", ok, f"STATUS_FAST {fast:.0f} Hz, STATUS_SLOW {slow:.1f} Hz", "100 +-10 Hz, 10 +-2 Hz")

    def check_params(self) -> Check:
        original = self.pc.read(self.node, "following_error")
        test_value = round(original + 0.1, 3)
        self.pc.write(self.node, "following_error", test_value)
        back = self.pc.read(self.node, "following_error")
        self.pc.write(self.node, "following_error", original)
        restored = self.pc.read(self.node, "following_error")
        ok = abs(back - test_value) < 1e-5 and abs(restored - original) < 1e-5
        # A write outside the limits must be refused.
        refused = False
        try:
            self.pc.write(self.node, "current_limit", 99.0)
        except Exception:
            refused = True
        return Check("", ok and refused, f"wrote {test_value}, read {back:.3f}; out-of-range write "
                     f"{'refused' if refused else 'ACCEPTED'}", "exact read-back, bad write refused")

    def check_supply(self) -> Check:
        ns = self.dm.node(self.node)
        v, t = ns.bus_voltage, ns.motor_temp
        ok = 44.0 <= v <= 52.0 and 0.0 <= t <= 60.0
        return Check("", ok, f"bus {v:.2f} V, winding {t:.1f} C", "44-52 V, 0-60 C")

    def check_current_offset(self) -> Check:
        r = cal.current_offset(self.dm, self.pc, self.node)
        return Check("", r.ok, f"found {r.values.get('offset_a', 0) * 1000:+.1f} mA, residual "
                     f"{r.values.get('residual_a', 0) * 1000:+.1f} mA", "|residual| < 10 mA",
                     "offset written to current_offset")

    def check_direction(self) -> Check:
        r = cal.direction_check(self.dm, self.pc, self.node, fix=self.fix)
        note = "motor phases swapped: motor_polarity set to -1" if r.values.get("fixed") else ""
        return Check("", r.ok, r.summary, "+current -> +motion", note)

    def check_homing(self) -> Check:
        r = cal.home(self.dm, self.pc, self.node)
        return Check("", r.ok, r.summary, "stall detected, reads homing_position +-0.01 rad")

    def check_velocity(self) -> Check:
        if not cal.move_to(self.dm, self.node, -1.0):
            return Check("", False, "could not reach the start position", "-")
        err = cal.velocity_tracking(self.dm, self.node, speed=1.0, seconds=1.2)
        cal.move_to(self.dm, self.node, 0.0)
        return Check("", err < self.VELOCITY_RMS_MAX, f"RMS error {err:.4f} rad/s at 1.0 rad/s",
                     f"< {self.VELOCITY_RMS_MAX} rad/s")

    def check_step(self) -> Check:
        m, _ = cal.step_response(self.dm, self.node, amplitude=0.5, start=-0.25)
        ok = (m.overshoot_pct < self.STEP_OVERSHOOT_MAX and m.settling_time < self.STEP_SETTLING_MAX
              and m.steady_state_error < self.STEP_SS_ERROR_MAX)
        cal.move_to(self.dm, self.node, 0.0)
        return Check("", ok, f"0.5 rad step: rise {m.rise_time * 1000:.0f} ms, overshoot {m.overshoot_pct:.1f}%, "
                     f"settling {m.settling_time * 1000:.0f} ms, error {m.steady_state_error * 1000:.2f} mrad",
                     f"overshoot < {self.STEP_OVERSHOOT_MAX:.0f}%, settling < {self.STEP_SETTLING_MAX:.1f} s, "
                     f"error < {self.STEP_SS_ERROR_MAX * 1000:.0f} mrad")

    def check_watchdog(self) -> Check:
        timeout_ms = self.pc.read(self.node, "cmd_timeout_ms")
        cal.enable_and_wait(self.dm, self.node, Mode.VELOCITY, 0.0)
        time.sleep(0.2)
        self.dm.suspend_commands(self.node, True)
        try:
            time.sleep(0.02)
            last_tx = self.dm.node(self.node).last_cmd_tx
            t_fault = self.dm.wait_emcy(self.node, FAULT_BIT["CMD_TIMEOUT"], 1.0, last_tx)
        finally:
            self.dm.disable(self.node)
            self.dm.suspend_commands(self.node, False)
        if t_fault is None:
            return Check("", False, "no CMD_TIMEOUT fault within 1 s", f"{timeout_ms:.0f} ms")
        latency = t_fault - last_tx
        cal.clear_faults_if_any(self.dm, self.node)
        ok = timeout_ms / 1000 <= latency + 0.005 and latency <= timeout_ms / 1000 + self.WATCHDOG_MARGIN
        return Check("", ok, f"faulted {latency * 1000:.0f} ms after the last command",
                     f"{timeout_ms:.0f}-{timeout_ms + self.WATCHDOG_MARGIN * 1000:.0f} ms")

    def check_estop(self) -> Check:
        cal.enable_and_wait(self.dm, self.node, Mode.VELOCITY, 0.0)
        time.sleep(0.2)
        t_send = time.time()
        self.dm.estop()
        t_fault = self.dm.wait_emcy(self.node, FAULT_BIT["ESTOP"], 1.0, t_send)
        disabled = self.dm.wait_until(lambda: self.dm.nodes[self.node].state == State.FAULT, 0.5)
        if t_fault is None:
            return Check("", False, "no ESTOP fault reported", f"< {self.ESTOP_LATENCY_MAX * 1000:.0f} ms")
        latency = t_fault - t_send
        cal.clear_faults_if_any(self.dm, self.node)
        ok = disabled and latency < self.ESTOP_LATENCY_MAX
        return Check("", ok, f"output off, fault reported {latency * 1000:.1f} ms after the E-stop frame",
                     f"< {self.ESTOP_LATENCY_MAX * 1000:.0f} ms")

    def check_e2e(self) -> Check:
        ns = self.dm.node(self.node)
        before = ns.e2e_errors
        good = self.dm.codec.command(self.node, Mode.IDLE, 0.0, False, False, (ns.counter + 100) & 0xFF)
        bad = type(good)(good.can_id, bytes([good.data[0], good.data[1] ^ 0x04]) + good.data[2:])
        self.dm.suspend_commands(self.node, True)  # keep the 100 Hz stream from interleaving
        try:
            time.sleep(0.02)
            self.dm.send(bad)    # flipped bit: CRC fails
            self.dm.send(good)   # accepted
            self.dm.send(good)   # same counter again: stale
        finally:
            self.dm.suspend_commands(self.node, False)
        ok = self.dm.wait_until(lambda: self.dm.nodes[self.node].e2e_errors >= before + 2, 0.5)
        after = self.dm.node(self.node).e2e_errors
        return Check("", ok and after == before + 2, f"node counted {after - before} rejected frame(s)",
                     "exactly 2 (bad CRC + repeated counter)")

    def check_finish(self) -> Check:
        cal.clear_faults_if_any(self.dm, self.node)
        cal.ensure_disabled(self.dm, self.node)
        self.pc.save(self.node)
        ns = self.dm.node(self.node)
        ok = ns.fault_flags == 0 and ns.state == State.DISABLED
        temps = list(ns.temp)[-20:]
        peak_temp = max(temps) if temps else float("nan")
        return Check("", ok, f"state {ns.state_name}, faults {fault_text(ns.fault_flags)}, winding "
                     f"{statistics.fmean(temps) if temps else float('nan'):.1f} C (peak {peak_temp:.1f} C)",
                     "DISABLED, no faults, saved to flash", "parameters saved to flash")
