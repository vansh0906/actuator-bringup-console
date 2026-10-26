"""Console against the real C++ firmware simulator over SLCAN/TCP."""

import time

import pytest

from bringup import SlcanLink
from bringup import calibration as cal
from bringup.bringup_test import BringupTest
from bringup.cli import main as cli_main
from bringup.params import PARAMS, ParamError
from bringup.protocol import FAULT_BIT, InjectKind, Mode, State
from bringup.recorder import Recorder, analyze_log


def test_parameter_defaults_match_firmware(bus):
    _, dm, pc = bus
    pc.load_defaults(1)
    values = pc.read_all(1)
    for p in PARAMS:
        if p.name in ("fw_version", "node_id"):
            continue
        assert values[p.name] == pytest.approx(p.default, abs=1e-6), p.name
    assert values["node_id"] == 1


def test_bad_parameter_writes_are_refused(bus):
    _, dm, pc = bus
    with pytest.raises(ParamError, match="out of range"):
        pc.write(1, "current_limit", 99)
    with pytest.raises(ParamError, match="read-only"):
        pc.write(1, "fw_version", 2)
    with pytest.raises(ParamError, match="out of range"):
        pc.write(1, "motor_polarity", 0.5)


def test_velocity_mode_tracks(bus):
    _, dm, pc = bus
    cal.clear_faults_if_any(dm, 1)
    cal.enable_and_wait(dm, 1, Mode.VELOCITY, 0.4)
    time.sleep(1.0)
    assert dm.node(1).state == State.ENABLED
    assert dm.node(1).velocity == pytest.approx(0.4, abs=0.02)
    dm.disable(1)


def test_watchdog_trips_when_host_stops(bus):
    _, dm, pc = bus
    cal.clear_faults_if_any(dm, 1)
    cal.enable_and_wait(dm, 1, Mode.VELOCITY, 0.0)
    dm.suspend_commands(1, True)
    time.sleep(0.02)
    t_last = dm.node(1).last_cmd_tx
    t_fault = dm.wait_emcy(1, FAULT_BIT["CMD_TIMEOUT"], 1.0, t_last)
    dm.suspend_commands(1, False)
    dm.disable(1)
    assert t_fault is not None
    assert 0.095 < t_fault - t_last < 0.150
    cal.clear_faults_if_any(dm, 1)


def test_jam_is_detected_and_cleared(bus):
    _, dm, pc = bus
    cal.clear_faults_if_any(dm, 2)
    pc.write(2, "motor_polarity", -1.0)  # node 2's motor is wired backwards
    cal.enable_and_wait(dm, 2, Mode.VELOCITY, 0.8)
    time.sleep(0.3)
    dm.inject(2, InjectKind.JAM, 40.0)
    assert dm.wait_until(lambda: dm.nodes[2].state == State.FAULT, 2.0)
    assert dm.node(2).fault_flags & (FAULT_BIT["FOLLOWING_ERROR"] | FAULT_BIT["STALL"])
    dm.inject(2, InjectKind.CLEAR_ALL)
    cal.clear_faults_if_any(dm, 2)
    assert dm.node(2).state == State.DISABLED


def test_second_client_sees_the_bus(bus, sim):
    _, dm, _ = bus
    seen = []
    other = SlcanLink(sim, lambda f: seen.append(f.can_id)).open()
    try:
        dm.arm(1)  # stream COMMAND frames from the first client
        time.sleep(0.3)
    finally:
        other.close()
        dm.release(1)
    assert 0x201 in seen, "a second client must see the first client's commands"
    assert 0x181 in seen and 0x182 in seen


def test_recording_and_replay(bus, tmp_path):
    _, dm, _ = bus
    rec = Recorder(dm)
    log_path, csv_path = rec.start(tmp_path)
    time.sleep(1.0)
    rec.stop()
    summary = analyze_log(log_path)
    assert summary.rates["STATUS_FAST[1]"] == pytest.approx(100, rel=0.15)
    assert summary.rates["STATUS_SLOW[2]"] == pytest.approx(10, rel=0.3)
    assert csv_path.read_text().count("\n") > 150


def test_cli_monitor_and_params(sim, capsys):
    assert cli_main(["--url", sim, "monitor", "--seconds", "0.5", "--only-new"]) == 0
    out = capsys.readouterr().out
    assert "STATUS_FAST" in out and "bus load" in out
    assert cli_main(["--url", sim, "params", "--node", "2", "--set", "vel_ki=22"]) == 0
    assert "vel_ki = 22" in capsys.readouterr().out


@pytest.mark.parametrize("node", [1, 2])
def test_full_bringup_passes_and_fixes_node2(bus, node, tmp_path):
    _, dm, pc = bus
    pc.load_defaults(node)  # undo what earlier tests changed
    report = BringupTest(dm, pc, node, log=lambda s: None).run()
    md, _ = report.save(tmp_path)
    assert report.passed, md.read_text()
    direction = [c for c in report.checks if c.name == "Motor direction"][0]
    assert ("swapped" in direction.note) == (node == 2)
