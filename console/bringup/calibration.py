"""Calibration and characterisation routines.

Each routine drives one actuator through the DeviceManager and ParamClient,
measures from the live telemetry and writes the result to the actuator's
parameters. They are what a bring-up engineer does by hand, made
repeatable:

  current_offset   zero the current sensor with the motor disabled
  direction_check  ramp current until the joint just moves; a joint that
                   moves backwards has its motor phases swapped
  home             drive slowly into the hard stop and set the joint zero
  step_response    position step: rise time, overshoot, settling, error
  velocity_tracking  RMS velocity error at a constant speed
"""

from __future__ import annotations

import math
import statistics
import time
from dataclasses import dataclass, field

from .metrics import StepMetrics, rms, step_metrics
from .protocol import Mode, State, fault_text


@dataclass
class CalResult:
    name: str
    ok: bool
    summary: str
    values: dict = field(default_factory=dict)


class CalibrationError(RuntimeError):
    pass


def _wait_state(dm, node: int, states: set, timeout: float = 1.0) -> bool:
    return dm.wait_until(lambda: dm.nodes[node].state in states, timeout)


def ensure_disabled(dm, node: int) -> None:
    dm.disable(node)
    if not _wait_state(dm, node, {State.DISABLED, State.FAULT}, 1.0):
        raise CalibrationError(f"node {node} did not disable")


def clear_faults_if_any(dm, node: int) -> None:
    if dm.node(node).fault_flags:
        dm.clear_faults(node)
        if not dm.wait_until(lambda: dm.nodes[node].fault_flags == 0 and dm.nodes[node].state == State.DISABLED,
                             1.5):
            raise CalibrationError(f"node {node}: faults did not clear ({fault_text(dm.node(node).fault_flags)})")


def enable_and_wait(dm, node: int, mode: Mode, setpoint: float | None = None) -> None:
    dm.enable(node, mode, setpoint)
    dm.send_command_now(node)
    if not _wait_state(dm, node, {State.ENABLED}, 1.0):
        raise CalibrationError(f"node {node} did not enable (state {dm.node(node).state_name}, "
                               f"faults {fault_text(dm.node(node).fault_flags)})")


def fresh_position(dm, node: int, settle: float = 0.03) -> float:
    """Position from a status frame received after now (+settle)."""
    t0 = time.time()
    time.sleep(settle)
    dm.wait_until(lambda: dm.nodes[node].last_fast > t0, 0.5)
    return dm.node(node).position


# --- current sensor ------------------------------------------------------------

def current_offset(dm, pc, node: int, seconds: float = 1.0, tolerance: float = 0.01) -> CalResult:
    clear_faults_if_any(dm, node)
    ensure_disabled(dm, node)
    pc.write(node, "current_offset", 0.0)
    time.sleep(0.05)
    t0 = time.time()
    time.sleep(seconds)
    samples = [c for _, _, _, c in dm.samples_since(node, t0)]
    if len(samples) < seconds * 50:
        return CalResult("Current-sensor offset", False, f"only {len(samples)} samples in {seconds:.1f} s")
    offset = statistics.fmean(samples)
    noise = statistics.pstdev(samples)
    pc.write(node, "current_offset", round(offset, 4))
    time.sleep(0.05)
    t1 = time.time()
    time.sleep(0.5)
    check = [c for _, _, _, c in dm.samples_since(node, t1)]
    residual = statistics.fmean(check) if check else math.nan
    ok = abs(residual) < tolerance
    return CalResult(
        "Current-sensor offset",
        ok,
        f"offset {offset * 1000:+.1f} mA (noise {noise * 1000:.1f} mA rms), residual after zeroing "
        f"{residual * 1000:+.1f} mA",
        {"offset_a": round(offset, 4), "noise_a": round(noise, 4), "residual_a": round(residual, 4)},
    )


# --- motor direction -------------------------------------------------------------

def _breakaway_probe(dm, node: int, sign: float, ramp: float, max_current: float, move: float):
    start = fresh_position(dm, node)
    enable_and_wait(dm, node, Mode.CURRENT, 0.0)
    t0 = time.monotonic()
    moved, current = 0.0, 0.0
    try:
        while True:
            current = ramp * (time.monotonic() - t0)
            if current > max_current or dm.node(node).state != State.ENABLED:
                break
            dm.set_command(node, setpoint=sign * current)
            time.sleep(0.01)
            moved = dm.node(node).position - start
            if abs(moved) > move:
                break
    finally:
        dm.set_command(node, setpoint=0.0)
        dm.disable(node)
    time.sleep(0.4)  # let the joint coast to a stop
    return moved, current


def direction_check(dm, pc, node: int, fix: bool = True, ramp: float = 1.0, max_current: float = 1.5,
                    move: float = 0.02) -> CalResult:
    """Positive current must move the joint in the positive encoder direction.

    The current is ramped from zero until the joint moves `move` rad, so it
    barely moves; the current where it breaks away is reported too. If the
    joint is against a stop on the positive side, the probe repeats in the
    negative direction."""
    clear_faults_if_any(dm, node)
    ensure_disabled(dm, node)
    polarity = pc.read(node, "motor_polarity")

    def probe():
        for sign in (1.0, -1.0):
            moved, i_break = _breakaway_probe(dm, node, sign, ramp, max_current, move)
            if abs(moved) > move:
                return sign, moved, i_break
        return None, 0.0, max_current

    sign, moved, i_break = probe()
    if sign is None:
        return CalResult("Motor direction", False,
                         f"no motion up to {max_current:.1f} A in either direction: check for a brake, "
                         "a jam or an open motor phase")
    agrees = (moved > 0) == (sign > 0)
    values = {"breakaway_current_a": round(i_break, 3), "moved_rad": round(moved, 4)}
    if agrees:
        values["motor_polarity"] = polarity
        return CalResult("Motor direction", True,
                         f"{'+' if sign > 0 else '-'}current moves the joint {'+' if moved > 0 else '-'}: correct "
                         f"(breakaway at {i_break:.2f} A)", values)
    if not fix:
        return CalResult("Motor direction", False,
                         "motor runs backwards (phases swapped). Set motor_polarity to "
                         f"{-polarity:+.0f} before enabling closed-loop control", values)
    pc.write(node, "motor_polarity", -polarity)
    sign2, moved2, i_break2 = probe()
    ok = sign2 is not None and (moved2 > 0) == (sign2 > 0)
    values.update({"motor_polarity": -polarity, "fixed": True, "breakaway_current_a": round(i_break2, 3)})
    return CalResult("Motor direction", ok,
                     f"motor ran backwards (phases swapped): set motor_polarity to {-polarity:+.0f}; "
                     f"re-test {'passes' if ok else 'still fails'} (breakaway at {i_break2:.2f} A)", values)


# --- homing ---------------------------------------------------------------------------

def home(dm, pc, node: int, direction: float = -1.0, speed: float = 0.4, current_limit: float = 1.5,
         timeout: float = 25.0) -> CalResult:
    """Drive slowly into the hard stop with a low current limit, detect the
    stall, and set the joint zero so that the stop reads homing_position."""
    clear_faults_if_any(dm, node)
    ensure_disabled(dm, node)
    saved = {name: pc.read(node, name) for name in ("current_limit", "position_min", "position_max")}
    homing_position = pc.read(node, "homing_position")
    start = fresh_position(dm, node)
    stall_hold = 0.15
    try:
        # Soft limits would stop the joint short of the stop: open them up.
        pc.write(node, "position_min", -10.0)
        pc.write(node, "position_max", 10.0)
        pc.write(node, "current_limit", current_limit)
        enable_and_wait(dm, node, Mode.VELOCITY, direction * speed)
        t0 = time.monotonic()
        stall_since = None
        while True:
            if time.monotonic() - t0 > timeout:
                return CalResult("Homing", False, f"no hard stop found within {timeout:.0f} s")
            ns = dm.node(node)
            if ns.state == State.FAULT:
                return CalResult("Homing", False, f"faulted while homing: {fault_text(ns.fault_flags)}")
            stalled = abs(ns.velocity) < 0.03 and abs(ns.current) > 0.8 * current_limit
            now = time.monotonic()
            if stalled:
                stall_since = stall_since or now
                if now - stall_since >= stall_hold:
                    break
            else:
                stall_since = None
            time.sleep(0.01)
        dm.disable(node)
        _wait_state(dm, node, {State.DISABLED}, 1.0)
        time.sleep(0.15)  # stop spring relaxes
        at_stop = fresh_position(dm, node)
        old_offset = pc.read(node, "encoder_offset")
        new_offset = (at_stop + old_offset) - homing_position
        pc.write(node, "encoder_offset", new_offset)
        check = fresh_position(dm, node, 0.05)
        ok = abs(check - homing_position) < 0.01 and dm.wait_until(lambda: dm.nodes[node].homed, 1.0)
        return CalResult(
            "Homing",
            ok,
            f"hard stop found after {abs(at_stop - start):.2f} rad of travel; zero set "
            f"(encoder_offset {new_offset:+.4f} rad, joint now reads {check:+.4f} rad)",
            {"travel_rad": round(abs(at_stop - start), 4), "encoder_offset": round(new_offset, 5),
             "reads_rad": round(check, 4)},
        )
    finally:
        dm.disable(node)
        _wait_state(dm, node, {State.DISABLED, State.FAULT}, 1.0)
        pc.write(node, "current_limit", saved["current_limit"])
        pc.write(node, "position_min", saved["position_min"])
        pc.write(node, "position_max", saved["position_max"])


# --- motion characterisation ---------------------------------------------------------------

def move_to(dm, node: int, position: float, timeout: float = 6.0, tolerance: float = 0.01) -> bool:
    dm.set_command(node, Mode.POSITION, position)
    if dm.node(node).state != State.ENABLED:
        enable_and_wait(dm, node, Mode.POSITION, position)
    ok = dm.wait_until(lambda: abs(dm.nodes[node].position - position) < tolerance
                       and abs(dm.nodes[node].velocity) < 0.02, timeout)
    time.sleep(0.2)
    return ok


def step_response(dm, node: int, amplitude: float = 0.5, hold: float = 1.5, start: float | None = None):
    """Position step from `start` (default: where the joint is). Returns
    (StepMetrics, (t, pos) samples relative to the step time)."""
    if start is None:
        start = fresh_position(dm, node)
    if not move_to(dm, node, start):
        raise CalibrationError(f"node {node} could not settle at {start:+.3f} rad")
    target = start + amplitude
    dm.set_command(node, Mode.POSITION, target)
    dm.send_command_now(node)
    t_step = time.time()
    time.sleep(hold)
    rows = dm.samples_since(node, t_step - 0.1)
    t = [r[0] for r in rows]
    pos = [r[1] for r in rows]
    metrics: StepMetrics = step_metrics(t, pos, start, target, t_step)
    return metrics, ([ti - t_step for ti in t], pos)


def velocity_tracking(dm, node: int, speed: float = 1.0, seconds: float = 1.2, settle: float = 0.4) -> float:
    dm.set_command(node, Mode.VELOCITY, speed)
    if dm.node(node).state != State.ENABLED:
        enable_and_wait(dm, node, Mode.VELOCITY, speed)
    dm.send_command_now(node)
    t0 = time.time()
    time.sleep(seconds)
    rows = dm.samples_since(node, t0 + settle)
    dm.set_command(node, Mode.VELOCITY, 0.0)
    return rms([speed - r[2] for r in rows])
