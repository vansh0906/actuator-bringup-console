"""GUI smoke test with the real Qt in offscreen mode (runs in CI)."""

import os
import time

import pytest

pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from bringup.gui.app import MainWindow  # noqa: E402
from bringup.protocol import State  # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


def pump(app, win, seconds):
    end = time.time() + seconds
    while time.time() < end:
        app.processEvents()
        time.sleep(0.01)


def wait_task(app, win, timeout=60):
    end = time.time() + timeout
    while time.time() < end:
        pump(app, win, 0.05)
        if not win.runner.busy:
            pump(app, win, 0.1)
            return
    raise TimeoutError("GUI task did not finish")


def test_gui_drives_the_simulator(qapp, sim, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    win = MainWindow(sim)
    win.show()
    win.connect_link()
    assert win.connected
    pump(qapp, win, 1.0)
    assert win.node_table.rowCount() == 2
    assert win.active_node == 1

    # Enable in velocity mode from the control tab and watch the plots fill.
    win.control_tab.mode.setCurrentIndex(1)
    win.control_tab.btn_enable.click()
    win.control_tab.setpoint.setValue(0.5)
    pump(qapp, win, 1.2)
    assert win.dm.node(1).state == State.ENABLED
    assert win.dm.node(1).velocity == pytest.approx(0.5, abs=0.03)
    x, y = win.control_tab.c_pos.getData()
    assert x is not None and len(x) > 50

    # E-stop from the toolbar.
    win.btn_estop.click()
    pump(qapp, win, 0.3)
    assert win.dm.node(1).state == State.FAULT
    win.control_tab.btn_clear.click()
    pump(qapp, win, 0.5)
    assert win.dm.node(1).state == State.DISABLED

    # Parameters tab: read all.
    win.tabs.setCurrentWidget(win.params_tab)
    win.params_tab.btn_read.click()
    wait_task(qapp, win)
    assert win.params_tab.table.item(2, 2).text() == "1.5"

    # Calibration tab: zero the current sensor.
    win.tabs.setCurrentWidget(win.cal_tab)
    win.cal_tab.btn_offset.click()
    wait_task(qapp, win)
    assert any("OK: offset" in line for line in win.cal_tab.results.toPlainText().splitlines())

    # Bus monitor shows every message.
    win.tabs.setCurrentWidget(win.bus_tab)
    pump(qapp, win, 1.0)
    assert win.bus_tab.table.rowCount() >= 6

    # Record, then replay the log.
    win.btn_record.setChecked(True)
    pump(qapp, win, 1.0)
    win.btn_record.setChecked(False)
    log_path = win.recorder.paths[0]
    win.tabs.setCurrentWidget(win.replay_tab)
    win.replay_tab.load(str(log_path))
    wait_task(qapp, win)
    assert "STATUS_FAST[1]" in win.replay_tab.summary.toPlainText()

    win.close()
    pump(qapp, win, 0.2)
    assert win.link is None
