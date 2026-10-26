"""The ROS 2 driver's logic, tested against the simulator without ROS."""

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_driver" / "actuator_driver"))
from actuator_driver.driver_core import LEVEL_ERROR, LEVEL_OK, LEVEL_WARN, DriverCore  # noqa: E402

from bringup import calibration as cal  # noqa: E402
from bringup.protocol import InjectKind, State  # noqa: E402


@pytest.fixture
def core(bus):
    _, dm, pc = bus
    for n in (1, 2):
        cal.clear_faults_if_any(dm, n)
    pc.write(2, "motor_polarity", -1.0)  # node 2 is wired backwards (the bring-up test finds this)
    return DriverCore(dm, pc, [1, 2], ["joint1", "joint2"], "velocity", torque_constant=4.0, command_timeout=0.3)


def test_commands_move_joints_and_timeout_stops_them(core):
    ok, msg = core.enable()
    assert ok, msg
    assert core.apply_commands([0.4, -0.2]) is None
    time.sleep(0.8)
    core.apply_commands([0.4, -0.2])
    names, pos, vel, eff = core.joint_state()
    assert names == ["joint1", "joint2"]
    assert vel[0] == pytest.approx(0.4, abs=0.03) and vel[1] == pytest.approx(-0.2, abs=0.03)
    # Nothing more from "ROS": the host-side timeout stops the joints.
    time.sleep(0.35)
    assert core.check_command_timeout()
    time.sleep(0.5)
    _, _, vel, _ = core.joint_state()
    assert abs(vel[0]) < 0.02 and abs(vel[1]) < 0.02
    assert core.dm.node(1).state == State.ENABLED  # stopped, not faulted
    core.disable()


def test_bad_commands_are_rejected(core):
    assert "expected 2 values" in core.apply_commands([1.0])
    assert "NaN" in core.apply_commands([float("nan"), 0.0])


def test_diagnostics_levels(core):
    # Heartbeats come at 2 Hz: until the first one, a joint reports STALE.
    assert core.dm.wait_until(lambda: all(core.dm.nodes[n].last_heartbeat for n in (1, 2)), 1.0)
    diags = core.diagnostics()
    assert [d.name for d in diags] == ["actuator_driver: joint1", "actuator_driver: joint2"]
    assert all(d.level in (LEVEL_OK, LEVEL_WARN) for d in diags), [(d.level, d.message) for d in diags]
    core.dm.inject(1, InjectKind.ENCODER_GLITCH, 0.3)
    assert core.dm.wait_until(lambda: core.dm.nodes[1].fault_flags != 0, 1.0)
    time.sleep(0.15)
    d1 = core.diagnostics()[0]
    assert d1.level == LEVEL_ERROR and "ENCODER" in d1.message
    assert d1.values["homed"] == "False"
    ok, msg = core.enable()
    assert not ok and "clear_faults" in msg
    ok, msg = core.clear_faults()
    assert ok, msg
