"""Unit tests for the parts that need no bus: metrics, parameter table,
recorder/replay and display formatting."""

import math
import re
from pathlib import Path

import pytest

from bringup.devices import DeviceManager, NodeState
from bringup.metrics import rms, step_metrics
from bringup.params import PARAMS
from bringup.protocol import Codec, Frame, Mode, State
from bringup.recorder import analyze_log, format_candump, read_candump
from bringup.ui_model import bus_row, node_row

REPO = Path(__file__).resolve().parents[2]


def test_first_order_step_metrics():
    tau = 0.1
    t = [i * 0.001 for i in range(2000)]
    y = [1.0 - math.exp(-ti / tau) for ti in t]
    m = step_metrics(t, y, 0.0, 1.0, 0.0)
    assert m.overshoot_pct == 0.0
    assert m.rise_time == pytest.approx(2.197 * tau, rel=0.02)    # ln(9) tau
    assert m.settling_time == pytest.approx(3.912 * tau, rel=0.02)  # -ln(0.02) tau
    assert m.steady_state_error < 1e-3


def test_second_order_overshoot_and_negative_step():
    zeta, wn = 0.5, 20.0
    wd = wn * math.sqrt(1 - zeta ** 2)
    t = [i * 0.0005 for i in range(4000)]
    y = [1 - math.exp(-zeta * wn * ti) * (math.cos(wd * ti) + zeta / math.sqrt(1 - zeta ** 2) * math.sin(wd * ti))
         for ti in t]
    expected = 100 * math.exp(-zeta * math.pi / math.sqrt(1 - zeta ** 2))  # 16.3 %
    m = step_metrics(t, y, 0.0, 1.0, 0.0)
    assert m.overshoot_pct == pytest.approx(expected, rel=0.01)
    # The same response stepping down from 2 to 1.
    m2 = step_metrics(t, [2 - v for v in y], 2.0, 1.0, 0.0)
    assert m2.overshoot_pct == pytest.approx(expected, rel=0.01)
    assert rms([3, 4]) == pytest.approx(math.sqrt(12.5))


def test_param_table_matches_firmware_source():
    """console/bringup/params.py must mirror firmware/core/params.cpp."""
    src = (REPO / "firmware" / "core" / "params.cpp").read_text()
    rows = re.findall(r'\{"(\w+)", "([^"]*)", (-?[\d.]+)f, (-?[\d.]+)f, (-?[\d.]+)f, (true|false), (true|false)\}', src)
    rows = [r for r in rows if r[0] != "invalid"]  # the out-of-range placeholder
    assert len(rows) == len(PARAMS)
    for p, (name, unit, d, lo, hi, ro, nd) in zip(PARAMS, rows):
        assert p.name == name
        assert p.unit == unit
        assert p.default == pytest.approx(float(d))
        assert (p.minimum, p.maximum) == (pytest.approx(float(lo)), pytest.approx(float(hi)))
        assert p.read_only == (ro == "true") and p.needs_disabled == (nd == "true"), name


def _synthetic_log(path: Path) -> Path:
    codec = Codec()
    lines = []
    t0 = 1_700_000_000.0
    for i in range(300):  # 3 s of node 1 moving at 0.5 rad/s
        t = t0 + i * 0.01
        f = codec.encode("STATUS_FAST", 1, position=0.005 * i, velocity=0.5, current=0.3)
        f.timestamp = t
        lines.append(format_candump(f))
        if i % 10 == 0:
            s = codec.encode("STATUS_SLOW", 1, bus_voltage=48.0, motor_temp=30.0 + i / 100, fault_flags=0,
                             state=int(State.ENABLED), mode=int(Mode.VELOCITY), e2e_errors=0)
            s.timestamp = t
            lines.append(format_candump(s))
    e = codec.encode("EMCY", 1, fault_code=1 << 4, fault_flags=1 << 4, state=3, motor_temp=33.0, bus_voltage=48.0)
    e.timestamp = t0 + 3.0
    lines.append(format_candump(e))
    path.write_text("\n".join(lines) + "\n")
    return path


def test_candump_round_trip_and_analysis(tmp_path):
    log = _synthetic_log(tmp_path / "bus.log")
    frames = list(read_candump(log))
    assert len(frames) == 300 + 30 + 1
    assert frames[0].can_id == 0x181 and frames[0].timestamp == pytest.approx(1_700_000_000.0)
    summary = analyze_log(log)
    assert summary.rates["STATUS_FAST[1]"] == pytest.approx(100, rel=0.02)
    assert summary.nodes[1]["pos_max"] == pytest.approx(1.495, abs=1e-3)
    assert summary.nodes[1]["temp_max"] == pytest.approx(32.9, abs=0.05)  # last slow frame at i=290
    assert len(summary.faults) == 1 and "FOLLOWING_ERROR" in summary.faults[0][2]
    assert "Faults:" in summary.to_text()


def test_device_manager_offline_state_and_bus_stats():
    dm = DeviceManager(link=None)
    codec = dm.codec
    t = 1000.0
    for i in range(100):
        f = codec.encode("STATUS_FAST", 2, position=0.1, velocity=0.0, current=0.01)
        f.timestamp = t + i * 0.01
        dm.feed(f)
    hb = codec.encode("HEARTBEAT", 2, state=1, uptime=5.0, fw_major=1, fw_minor=0, homed=1)
    hb.timestamp = t + 1.0
    dm.feed(hb)
    ns = dm.node(2)
    assert ns.position == pytest.approx(0.1) and ns.homed and ns.fw_version == "1.0"
    stats = dm.bus_stats(now=t + 1.0)
    assert stats["frames_per_s"] == pytest.approx(101, abs=2)
    assert 2.0 < stats["bus_load_pct"] < 4.0  # ~100 frames x ~120 bits of 500 kbit/s
    row = [r for r in stats["ids"] if r["id"] == 0x182][0]
    assert bus_row(row)[1] == "STATUS_FAST[2]"


def test_node_row_severity():
    ns = NodeState(1)
    ns.online, ns.last_heartbeat, ns.state = True, 100.0, State.ENABLED
    cells, sev = node_row(ns, now=100.5)
    assert cells[1] == "online" and sev == "ok"
    ns.fault_flags = 1 << 1
    assert node_row(ns, now=100.5)[1] == "fault"
    ns.fault_flags = 0
    assert node_row(ns, now=105.0)[0][1] == "LOST"


def test_frame_str():
    assert str(Frame(0x181, bytes([1, 2]))) == "181#0102"
