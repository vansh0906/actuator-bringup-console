"""Shared fixtures. Integration tests start the real C++ simulator
(firmware/build/actuator_sim, or $ACTUATOR_SIM) on a free port."""

import os
import socket
import subprocess
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]


def sim_binary() -> Path | None:
    env = os.environ.get("ACTUATOR_SIM")
    candidates = [Path(env)] if env else []
    candidates += [REPO / "firmware" / "build" / "actuator_sim"]
    for c in candidates:
        if c.exists():
            return c
    return None


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    binary = sim_binary()
    if binary is None:
        pytest.skip("simulator not built: cmake -S firmware -B firmware/build && cmake --build firmware/build")
    port = free_port()
    state = tmp_path_factory.mktemp("simstate")
    proc = subprocess.Popen([str(binary), "--port", str(port), "--state-dir", str(state), "--quiet"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    url = f"tcp://127.0.0.1:{port}"
    deadline = time.time() + 5
    while time.time() < deadline:
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
            break
        except OSError:
            time.sleep(0.05)
    else:
        proc.kill()
        pytest.fail("simulator did not start")
    yield url
    proc.terminate()
    proc.wait(timeout=5)


@pytest.fixture
def bus(sim):
    """(link, dm, pc) connected to the simulator, closed after the test."""
    from bringup import connect

    link, dm, pc = connect(sim)
    assert dm.wait_until(lambda: {1, 2} <= set(dm.node_ids()), 3.0), "nodes did not appear"
    yield link, dm, pc
    for n in dm.node_ids():
        dm.disable(n)
    time.sleep(0.05)
    dm.stop()
    link.close()
