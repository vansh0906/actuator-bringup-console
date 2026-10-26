"""Step-response and tracking metrics."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class StepMetrics:
    rise_time: float        # s, 10% -> 90%
    overshoot_pct: float    # % of the step size
    settling_time: float    # s, last time outside the +-2% band
    steady_state_error: float  # same unit as the signal
    peak: float

    def as_dict(self) -> dict:
        return {
            "rise_time_s": round(self.rise_time, 4),
            "overshoot_pct": round(self.overshoot_pct, 2),
            "settling_time_s": round(self.settling_time, 4),
            "steady_state_error": round(self.steady_state_error, 5),
        }


def step_metrics(t: list[float], y: list[float], y0: float, y1: float, t_step: float,
                 band: float = 0.02) -> StepMetrics:
    if len(t) < 5:
        raise ValueError("not enough samples for step metrics")
    step = y1 - y0
    if step == 0:
        raise ValueError("zero step")
    rel = [(ti - t_step, (yi - y0) / step) for ti, yi in zip(t, y) if ti >= t_step]

    def first_crossing(level: float) -> float:
        for ti, r in rel:
            if r >= level:
                return ti
        return math.nan

    rise = first_crossing(0.9) - first_crossing(0.1)
    peak_rel = max(r for _, r in rel)
    overshoot = max(0.0, (peak_rel - 1.0) * 100.0)
    settling = 0.0
    for ti, r in rel:
        if abs(r - 1.0) > band:
            settling = ti
    tail = [r for _, r in rel[-max(3, len(rel) // 10):]]
    ss_error = abs(sum(tail) / len(tail) - 1.0) * abs(step)
    return StepMetrics(rise, overshoot, settling, ss_error, y0 + step * peak_rel)


def rms(values: list[float]) -> float:
    vals = [v for v in values if not math.isnan(v)]
    if not vals:
        return math.nan
    return math.sqrt(sum(v * v for v in vals) / len(vals))
