"""The console's tabs. Each tab gets the main window (`win`) for the device
manager, the parameter client, the task runner and the active node, and
has a refresh() the main window calls ~20 times a second while visible."""

from __future__ import annotations

import math
from pathlib import Path

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .. import calibration as cal
from ..bringup_test import BringupTest
from ..params import PARAMS
from ..protocol import InjectKind, Mode, State
from ..recorder import analyze_log
from ..ui_model import INJECT_HELP, MODE_RANGES, bus_row, check_color

PEN = {
    "pos": pg.mkPen("#1565c0", width=1.6),
    "vel": pg.mkPen("#2e7d32", width=1.6),
    "cur": pg.mkPen("#c62828", width=1.6),
    "temp": pg.mkPen("#ef6c00", width=1.6),
    "sp": pg.mkPen("#424242", width=1.2, style=Qt.PenStyle.DashLine),
}


def _item(text: str, editable: bool = False) -> QTableWidgetItem:
    it = QTableWidgetItem(text)
    if not editable:
        it.setFlags(it.flags() & ~Qt.ItemFlag.ItemIsEditable)
    return it


def _make_plot(layout: pg.GraphicsLayoutWidget, row: int, label: str, unit: str):
    p = layout.addPlot(row=row, col=0)
    p.showGrid(x=True, y=True, alpha=0.25)
    p.setLabel("left", label, units=unit)
    p.setMouseEnabled(x=False, y=True)
    return p


# ---------------------------------------------------------------------------------------------
class ControlTab(QWidget):
    """Enable/disable, mode and setpoint, jog, and live plots of the active node."""

    def __init__(self, win):
        super().__init__()
        self.win = win
        self._syncing = False
        outer = QHBoxLayout(self)

        panel = QGroupBox("Command (active node)")
        form = QFormLayout(panel)
        self.mode = QComboBox()
        for m in (Mode.CURRENT, Mode.VELOCITY, Mode.POSITION):
            self.mode.addItem(m.name.title(), int(m))
        self.mode.setCurrentIndex(1)
        self.setpoint = QDoubleSpinBox()
        self.setpoint.setDecimals(3)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(-1000, 1000)
        self.unit = QLabel()
        sp_row = QHBoxLayout()
        sp_row.addWidget(self.setpoint)
        sp_row.addWidget(self.unit)
        form.addRow("Mode", self.mode)
        form.addRow("Setpoint", sp_row)
        form.addRow("", self.slider)

        self.btn_enable = QPushButton("Enable")
        self.btn_disable = QPushButton("Disable")
        self.btn_clear = QPushButton("Clear faults")
        row = QHBoxLayout()
        for b in (self.btn_enable, self.btn_disable, self.btn_clear):
            row.addWidget(b)
        form.addRow(row)

        self.jog_speed = QDoubleSpinBox()
        self.jog_speed.setRange(0.05, 2.0)
        self.jog_speed.setSingleStep(0.05)
        self.jog_speed.setValue(0.3)
        self.jog_speed.setSuffix(" rad/s")
        self.btn_jog_neg = QPushButton("◀ Jog")
        self.btn_jog_pos = QPushButton("Jog ▶")
        self.btn_zero = QPushButton("Go to 0 rad")
        jog = QHBoxLayout()
        jog.addWidget(self.btn_jog_neg)
        jog.addWidget(self.btn_jog_pos)
        form.addRow("Jog speed", self.jog_speed)
        form.addRow(jog)
        form.addRow(self.btn_zero)

        self.window_s = QDoubleSpinBox()
        self.window_s.setRange(2, 60)
        self.window_s.setValue(10)
        self.window_s.setSuffix(" s")
        self.pause = QCheckBox("Pause plots")
        form.addRow("Plot window", self.window_s)
        form.addRow(self.pause)
        hint = QLabel("Esc = E-STOP (all nodes).\nHold a jog button to move; release to stop.\n"
                      "Velocity and position modes run the drive's own loops; current mode has no speed loop.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #666;")
        form.addRow(hint)
        panel.setMaximumWidth(330)
        outer.addWidget(panel)

        self.plots = pg.GraphicsLayoutWidget()
        self.p_pos = _make_plot(self.plots, 0, "Position", "rad")
        self.p_vel = _make_plot(self.plots, 1, "Velocity", "rad/s")
        self.p_cur = _make_plot(self.plots, 2, "Current", "A")
        self.p_temp = _make_plot(self.plots, 3, "Winding", "°C")
        for p in (self.p_vel, self.p_cur, self.p_temp):
            p.setXLink(self.p_pos)
        self.p_temp.setLabel("bottom", "time", units="s")
        self.c_pos = self.p_pos.plot(pen=PEN["pos"])
        self.c_vel = self.p_vel.plot(pen=PEN["vel"])
        self.c_cur = self.p_cur.plot(pen=PEN["cur"])
        self.c_temp = self.p_temp.plot(pen=PEN["temp"])
        self.c_sp = {
            Mode.POSITION: self.p_pos.plot(pen=PEN["sp"]),
            Mode.VELOCITY: self.p_vel.plot(pen=PEN["sp"]),
            Mode.CURRENT: self.p_cur.plot(pen=PEN["sp"]),
        }
        outer.addWidget(self.plots, 1)

        self.mode.currentIndexChanged.connect(self._mode_changed)
        self.setpoint.valueChanged.connect(self._setpoint_changed)
        self.slider.valueChanged.connect(self._slider_changed)
        self.btn_enable.clicked.connect(self._enable)
        self.btn_disable.clicked.connect(self._disable)
        self.btn_clear.clicked.connect(self._clear)
        self.btn_jog_pos.pressed.connect(lambda: self._jog(+1))
        self.btn_jog_neg.pressed.connect(lambda: self._jog(-1))
        self.btn_jog_pos.released.connect(self._jog_stop)
        self.btn_jog_neg.released.connect(self._jog_stop)
        self.btn_zero.clicked.connect(self._go_zero)
        self._mode_changed()

    # --- helpers --------------------------------------------------------------
    def current_mode(self) -> Mode:
        return Mode(self.mode.currentData())

    def _ready(self) -> bool:
        return self.win.connected and self.win.active_node is not None

    def _mode_changed(self, *_):
        lo, hi, step, unit = MODE_RANGES[self.current_mode()]
        self._syncing = True
        self.setpoint.setRange(lo, hi)
        self.setpoint.setSingleStep(step)
        self.setpoint.setValue(0.0)
        self.slider.setValue(0)
        self.unit.setText(unit)
        self._syncing = False
        if self._ready():
            node = self.win.active_node
            ns = self.win.dm.node(node)
            sp = ns.position if (self.current_mode() == Mode.POSITION and not math.isnan(ns.position)) else 0.0
            self._set_spin(sp)
            self.win.dm.set_command(node, self.current_mode(), sp)

    def _set_spin(self, value: float) -> None:
        self._syncing = True
        self.setpoint.setValue(value)
        lo, hi, _, _ = MODE_RANGES[self.current_mode()]
        self.slider.setValue(int(round(1000 * (2 * (value - lo) / (hi - lo) - 1))))
        self._syncing = False

    def _setpoint_changed(self, value: float) -> None:
        if self._syncing:
            return
        self._set_spin(value)
        if self._ready():
            self.win.dm.set_command(self.win.active_node, self.current_mode(), value)

    def _slider_changed(self, pos: int) -> None:
        if self._syncing:
            return
        lo, hi, _, _ = MODE_RANGES[self.current_mode()]
        value = lo + (pos / 1000 + 1) / 2 * (hi - lo)
        self._syncing = True
        self.setpoint.setValue(value)
        self._syncing = False
        if self._ready():
            self.win.dm.set_command(self.win.active_node, self.current_mode(), value)

    def _enable(self) -> None:
        if not self._ready():
            return
        node = self.win.active_node
        mode = self.current_mode()
        ns = self.win.dm.node(node)
        if mode == Mode.POSITION:
            sp = ns.position if not math.isnan(ns.position) else 0.0
        else:
            sp = 0.0
        self._set_spin(sp)
        self.win.dm.enable(node, mode, sp)

    def _disable(self) -> None:
        if self._ready():
            self.win.dm.disable(self.win.active_node)

    def _clear(self) -> None:
        if self._ready():
            self.win.dm.clear_faults(self.win.active_node)

    def _jog(self, direction: int) -> None:
        if not self._ready():
            return
        node = self.win.active_node
        speed = direction * self.jog_speed.value()
        if self.win.dm.node(node).state != State.ENABLED:
            self.win.dm.enable(node, Mode.VELOCITY, speed)
        else:
            self.win.dm.set_command(node, Mode.VELOCITY, speed)

    def _jog_stop(self) -> None:
        if self._ready():
            self.win.dm.set_command(self.win.active_node, Mode.VELOCITY, 0.0)
            self.mode.setCurrentIndex(1)

    def _go_zero(self) -> None:
        if not self._ready():
            return
        node = self.win.active_node
        self.mode.setCurrentIndex(2)
        self._set_spin(0.0)
        if self.win.dm.node(node).state != State.ENABLED:
            self.win.dm.enable(node, Mode.POSITION, 0.0)
        else:
            self.win.dm.set_command(node, Mode.POSITION, 0.0)

    def refresh(self) -> None:
        if self.pause.isChecked() or not self._ready():
            return
        node = self.win.active_node
        t, pos, vel, cur, sp = self.win.dm.history(node, self.window_s.value())
        if not t:
            return
        t_end = t[-1]
        x = [ti - t_end for ti in t]
        self.c_pos.setData(x, pos)
        self.c_vel.setData(x, vel)
        self.c_cur.setData(x, cur)
        mode = self.win.dm.node(node).cmd_mode
        for m, curve in self.c_sp.items():
            if m == mode:
                curve.setData(x, sp, connect="finite")
            else:
                curve.setData([], [])
        ns = self.win.dm.node(node)
        with self.win.dm.lock:
            st, temp = list(ns.slow_t), list(ns.temp)
        if st:
            self.c_temp.setData([ti - t_end for ti in st], temp)
        self.p_pos.setXRange(-self.window_s.value(), 0, padding=0.01)


# ---------------------------------------------------------------------------------------------
class ParamsTab(QWidget):
    COLS = ["#", "Name", "Value", "Unit", "Min", "Max", "Notes"]

    def __init__(self, win):
        super().__init__()
        self.win = win
        self.last_read: dict[str, float] = {}
        lay = QVBoxLayout(self)
        buttons = QHBoxLayout()
        self.btn_read = QPushButton("Read all")
        self.btn_write = QPushButton("Write changed")
        self.btn_save = QPushButton("Save to flash")
        self.btn_defaults = QPushButton("Load defaults")
        self.btn_export = QPushButton("Export YAML…")
        self.btn_import = QPushButton("Import YAML…")
        for b in (self.btn_read, self.btn_write, self.btn_save, self.btn_defaults, self.btn_export, self.btn_import):
            buttons.addWidget(b)
        buttons.addStretch(1)
        lay.addLayout(buttons)
        self.table = QTableWidget(len(PARAMS), len(self.COLS))
        self.table.setHorizontalHeaderLabels(self.COLS)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        for r, p in enumerate(PARAMS):
            notes = p.description + (" (read-only)" if p.read_only else "") + \
                (" (only while disabled)" if p.needs_disabled else "")
            cells = [str(p.index), p.name, "", p.unit, f"{p.minimum:g}", f"{p.maximum:g}", notes]
            for c, text in enumerate(cells):
                self.table.setItem(r, c, _item(text, editable=(c == 2 and not p.read_only)))
        lay.addWidget(self.table)
        self.status = QLabel("Select a node and press Read all.")
        lay.addWidget(self.status)

        self.btn_read.clicked.connect(self.read_all)
        self.btn_write.clicked.connect(self.write_changed)
        self.btn_save.clicked.connect(lambda: self._simple("Save to flash", lambda n: self.win.pc.save(n)))
        self.btn_defaults.clicked.connect(self._defaults)
        self.btn_export.clicked.connect(self._export)
        self.btn_import.clicked.connect(self._import)

    def _node(self):
        if not self.win.connected or self.win.active_node is None:
            self.status.setText("Connect and select a node first.")
            return None
        return self.win.active_node

    def read_all(self) -> None:
        node = self._node()
        if node is None:
            return
        self.status.setText(f"Reading node {node}…")
        self.win.runner.run("read parameters", self.win.pc.read_all, node, on_done=self._show_values)

    def _show_values(self, values: dict) -> None:
        self.last_read = dict(values)
        for r, p in enumerate(PARAMS):
            self.table.item(r, 2).setText(f"{values[p.name]:.6g}")
            self.table.item(r, 2).setBackground(QBrush())
        self.status.setText(f"Read {len(values)} parameters from node {self.win.active_node}.")

    def write_changed(self) -> None:
        node = self._node()
        if node is None or not self.last_read:
            self.status.setText("Read the parameters first.")
            return
        changes = {}
        for r, p in enumerate(PARAMS):
            if p.read_only:
                continue
            text = self.table.item(r, 2).text().strip()
            try:
                value = float(text)
            except ValueError:
                self.status.setText(f"{p.name}: '{text}' is not a number")
                return
            # Compare as displayed: values come back as float32 (-2.4 reads -2.4000001).
            if text != f"{self.last_read.get(p.name, value):.6g}":
                changes[p.name] = value

        def work():
            done = []
            for name, value in changes.items():
                done.append((name, self.win.pc.write(node, name, value)))
            return done, self.win.pc.read_all(node)

        def finished(result):
            done, values = result
            self._show_values(values)
            self.status.setText("Wrote " + (", ".join(f"{n}={v:g}" for n, v in done) or "nothing (no changes)"))

        self.win.runner.run("write parameters", work, on_done=finished,
                            on_error=lambda e: self.status.setText(e))

    def _simple(self, label: str, fn) -> None:
        node = self._node()
        if node is None:
            return
        self.win.runner.run(label, fn, node, on_done=lambda _: self.status.setText(f"{label}: done on node {node}"),
                            on_error=lambda e: self.status.setText(e))

    def _defaults(self) -> None:
        node = self._node()
        if node is None:
            return

        def work():
            self.win.pc.load_defaults(node)
            return self.win.pc.read_all(node)

        self.win.runner.run("load defaults", work, on_done=self._show_values, on_error=lambda e: self.status.setText(e))

    def _export(self) -> None:
        node = self._node()
        if node is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export parameters", f"node{node}_params.yaml", "YAML (*.yaml)")
        if path:
            self._simple(f"Export to {Path(path).name}", lambda n: self.win.pc.export_yaml(n, path))

    def _import(self) -> None:
        node = self._node()
        if node is None:
            return
        path, _ = QFileDialog.getOpenFileName(self, "Import parameters", "", "YAML (*.yaml *.yml)")
        if not path:
            return

        def work():
            self.win.pc.import_yaml(node, path)
            return self.win.pc.read_all(node)

        self.win.runner.run("import parameters", work, on_done=self._show_values,
                            on_error=lambda e: self.status.setText(e))

    def refresh(self) -> None:
        pass


# ---------------------------------------------------------------------------------------------
class CalibrationTab(QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        lay = QHBoxLayout(self)
        left = QVBoxLayout()
        box = QGroupBox("Calibrate the active node")
        form = QFormLayout(box)
        self.btn_offset = QPushButton("Zero current sensor")
        self.btn_direction = QPushButton("Check motor direction")
        self.autofix = QCheckBox("Fix polarity automatically")
        self.autofix.setChecked(True)
        self.btn_home = QPushButton("Home to hard stop")
        self.amplitude = QDoubleSpinBox()
        self.amplitude.setRange(0.05, 1.5)
        self.amplitude.setValue(0.5)
        self.amplitude.setSuffix(" rad")
        self.btn_step = QPushButton("Run step response")
        form.addRow(self.btn_offset)
        form.addRow(self.btn_direction)
        form.addRow(self.autofix)
        form.addRow(self.btn_home)
        form.addRow("Step size", self.amplitude)
        form.addRow(self.btn_step)
        order = QLabel("Order on a new joint: zero the current sensor, check direction, home, then step-test. "
                       "Results are written to the node; use Parameters → Save to flash to keep them.")
        order.setWordWrap(True)
        order.setStyleSheet("color: #666;")
        form.addRow(order)
        left.addWidget(box)
        self.results = QPlainTextEdit()
        self.results.setReadOnly(True)
        left.addWidget(self.results, 1)
        lay.addLayout(left, 1)

        self.plot = pg.PlotWidget(title="Step response")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("left", "position", units="rad")
        self.plot.setLabel("bottom", "time after step", units="s")
        self.c_resp = self.plot.plot(pen=PEN["pos"])
        self.c_target = self.plot.plot(pen=PEN["sp"])
        lay.addWidget(self.plot, 2)

        self.btn_offset.clicked.connect(lambda: self._run("Current offset", cal.current_offset))
        self.btn_direction.clicked.connect(
            lambda: self._run("Direction check", lambda dm, pc, n: cal.direction_check(dm, pc, n,
                                                                                         fix=self.autofix.isChecked())))
        self.btn_home.clicked.connect(lambda: self._run("Homing", cal.home))
        self.btn_step.clicked.connect(self._step)

    def _node(self):
        if not self.win.connected or self.win.active_node is None:
            self.results.appendPlainText("Connect and select a node first.")
            return None
        return self.win.active_node

    def _run(self, label: str, fn) -> None:
        node = self._node()
        if node is None:
            return
        self.results.appendPlainText(f"{label} on node {node}…")

        def done(r):
            self.results.appendPlainText(f"  {'OK' if r.ok else 'FAILED'}: {r.summary}")

        self.win.runner.run(label, fn, self.win.dm, self.win.pc, node, on_done=done,
                            on_error=lambda e: self.results.appendPlainText(f"  error: {e}"))

    def _step(self) -> None:
        node = self._node()
        if node is None:
            return
        amp = self.amplitude.value()
        self.results.appendPlainText(f"Step response on node {node} ({amp:.2f} rad)…")

        def work():
            start = cal.fresh_position(self.win.dm, node)
            if self.win.dm.node(node).homed:
                # Keep the whole step inside the soft limits.
                lo = self.win.pc.read(node, "position_min") + 0.05
                hi = self.win.pc.read(node, "position_max") - 0.05 - amp
                if lo > hi:
                    raise cal.CalibrationError(f"a {amp:.2f} rad step does not fit inside the soft limits")
                clamped = min(max(start, lo), hi)
                if abs(clamped - start) > 1e-3:
                    self.win.runner.log(f"moving to {clamped:+.3f} rad first so the step stays inside the soft limits")
                start = clamped
            else:
                self.win.runner.log("note: node is not homed, so soft limits are off")
            metrics, (t, pos) = cal.step_response(self.win.dm, node, amplitude=amp, start=start)
            return metrics, t, pos, start + amp

        def done(result):
            m, t, pos, target = result
            self.c_resp.setData(t, pos)
            self.c_target.setData([t[0], t[-1]], [target, target])
            self.results.appendPlainText(
                f"  rise {m.rise_time * 1000:.0f} ms, overshoot {m.overshoot_pct:.1f}%, settling "
                f"{m.settling_time * 1000:.0f} ms, steady-state error {m.steady_state_error * 1000:.2f} mrad")

        self.win.runner.run("Step response", work, on_done=done,
                            on_error=lambda e: self.results.appendPlainText(f"  error: {e}"))

    def refresh(self) -> None:
        busy = self.win.runner.busy
        for b in (self.btn_offset, self.btn_direction, self.btn_home, self.btn_step):
            b.setEnabled(not busy)


# ---------------------------------------------------------------------------------------------
class TestTab(QWidget):
    COLS = ["Node", "#", "Check", "Result", "Measured", "Limit", "Note", "Time (s)"]

    def __init__(self, win):
        super().__init__()
        self.win = win
        self.report_dir = Path("reports")
        lay = QVBoxLayout(self)
        row = QHBoxLayout()
        self.btn_one = QPushButton("Test active node")
        self.btn_all = QPushButton("Test all nodes")
        self.btn_stop = QPushButton("Stop after this check")
        self.fix = QCheckBox("Fix what can be fixed (offsets, polarity, zero)")
        self.fix.setChecked(True)
        for w in (self.btn_one, self.btn_all, self.btn_stop, self.fix):
            row.addWidget(w)
        row.addStretch(1)
        lay.addLayout(row)
        self.table = QTableWidget(0, len(self.COLS))
        self.table.setHorizontalHeaderLabels(self.COLS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        lay.addWidget(self.table, 1)
        self.status = QLabel("Runs 13 checks per node (about 20 s each) and saves a Markdown + JSON report to "
                             f"{self.report_dir}/.")
        lay.addWidget(self.status)

        self.btn_one.clicked.connect(lambda: self._start([self.win.active_node] if self.win.active_node else []))
        self.btn_all.clicked.connect(lambda: self._start(self.win.dm.node_ids() if self.win.connected else []))
        self.btn_stop.clicked.connect(lambda: self.win.runner.stop_requested.set())

    def _start(self, nodes: list[int]) -> None:
        if not self.win.connected or not nodes:
            self.status.setText("Connect and select a node first.")
            return
        self.table.setRowCount(0)
        fix = self.fix.isChecked()
        runner = self.win.runner

        def work():
            reports = []
            for n in nodes:
                if runner.stop_requested.is_set():
                    break
                test = BringupTest(self.win.dm, self.win.pc, n, fix=fix, log=runner.log,
                                   stop_flag=runner.stop_requested.is_set)
                report = test.run()
                report.save(self.report_dir)
                reports.append(report)
            return reports

        def done(reports):
            for rep in reports:
                for i, c in enumerate(rep.checks, 1):
                    r = self.table.rowCount()
                    self.table.insertRow(r)
                    cells = [str(rep.node), str(i), c.name, "PASS" if c.passed else "FAIL", c.measured, c.limit,
                             c.note, f"{c.seconds:.1f}"]
                    for col, text in enumerate(cells):
                        it = _item(text)
                        it.setBackground(QBrush(QColor(check_color(c.passed))))
                        self.table.setItem(r, col, it)
            summary = "; ".join(rep.summary for rep in reports)
            self.status.setText(f"{summary}. Reports saved to {self.report_dir.resolve()}")

        self.status.setText(f"Testing node(s) {', '.join(map(str, nodes))}… progress is in the event log.")
        runner.run("bring-up test", work, on_done=done, on_error=lambda e: self.status.setText(e))

    def refresh(self) -> None:
        busy = self.win.runner.busy
        self.btn_one.setEnabled(not busy)
        self.btn_all.setEnabled(not busy)
        self.btn_stop.setEnabled(busy)


# ---------------------------------------------------------------------------------------------
class BusTab(QWidget):
    COLS = ["ID", "Message", "Dir", "Count", "Rate (Hz)", "Last data", "Decoded"]

    def __init__(self, win):
        super().__init__()
        self.win = win
        self.raw_lines: list[str] = []
        lay = QVBoxLayout(self)
        self.summary = QLabel("Not connected.")
        self.summary.setStyleSheet("font-weight: 600;")
        lay.addWidget(self.summary)
        split = QSplitter(Qt.Orientation.Vertical)
        self.table = QTableWidget(0, len(self.COLS))
        self.table.setHorizontalHeaderLabels(self.COLS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        split.addWidget(self.table)
        raw_box = QWidget()
        raw_lay = QVBoxLayout(raw_box)
        raw_lay.setContentsMargins(0, 0, 0, 0)
        controls = QHBoxLayout()
        self.pause = QCheckBox("Pause raw view")
        self.filter = QLineEdit()
        self.filter.setPlaceholderText("Filter by ID (hex), e.g. 181 or 08")
        controls.addWidget(QLabel("Raw frames"))
        controls.addWidget(self.filter)
        controls.addWidget(self.pause)
        raw_lay.addLayout(controls)
        self.raw = QPlainTextEdit()
        self.raw.setReadOnly(True)
        self.raw.setMaximumBlockCount(400)
        self.raw.setStyleSheet("font-family: Menlo, Consolas, monospace; font-size: 11px;")
        raw_lay.addWidget(self.raw)
        split.addWidget(raw_box)
        lay.addWidget(split, 1)
        self._pending: list[str] = []

    def attach(self, dm) -> None:
        dm.add_listener(self._on_frame)

    def _on_frame(self, frame, direction, decoded) -> None:
        # Called on the reader/command threads: only append to a list.
        if len(self._pending) < 5000:
            self._pending.append(f"{frame.timestamp:.3f} {direction} {frame.can_id:03X} [{len(frame.data)}] "
                                 f"{frame.data.hex(' ').upper()}")

    def refresh(self) -> None:
        if not self.win.connected:
            return
        stats = self.win.dm.bus_stats()
        link = self.win.link
        self.summary.setText(
            f"{stats['frames_per_s']:.0f} frames/s · bus load {stats['bus_load_pct']:.1f}% of 500 kbit/s · "
            f"SLCAN rx {link.rx_frames} / tx {link.tx_frames} · link errors {stats['link_errors']} · "
            f"send errors {stats['tx_errors']}")
        rows = stats["ids"]
        self.table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            for c, text in enumerate(bus_row(row)):
                it = self.table.item(r, c)
                if it is None:
                    self.table.setItem(r, c, _item(text))
                elif it.text() != text:
                    it.setText(text)
            stale = row["age"] > 1.0
            color = QColor("#9e9e9e") if stale else QColor("#000000")
            for c in range(len(self.COLS)):
                self.table.item(r, c).setForeground(QBrush(color))
        pending, self._pending = self._pending, []
        if self.pause.isChecked():
            return
        flt = self.filter.text().strip().upper()
        lines = [ln for ln in pending if not flt or f" {flt}" in ln[:24]][-300:]
        if lines:
            self.raw.appendPlainText("\n".join(lines))


# ---------------------------------------------------------------------------------------------
class FaultTab(QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        lay = QVBoxLayout(self)
        box = QGroupBox("Simulator fault injection (FAULT_INJECT frame, sim and ESP32 builds only)")
        form = QFormLayout(box)
        self.kind = QComboBox()
        for k in InjectKind:
            if k != InjectKind.CLEAR_ALL:
                self.kind.addItem(k.name.replace("_", " ").title(), int(k))
        self.value = QDoubleSpinBox()
        self.value.setRange(0, 500)
        self.value.setDecimals(2)
        self.value_label = QLabel()
        self.help = QLabel()
        self.help.setWordWrap(True)
        self.help.setStyleSheet("color: #444;")
        self.btn_inject = QPushButton("Inject on active node")
        self.btn_clear = QPushButton("Clear all injections")
        form.addRow("Fault", self.kind)
        form.addRow(self.value_label, self.value)
        form.addRow(self.help)
        row = QHBoxLayout()
        row.addWidget(self.btn_inject)
        row.addWidget(self.btn_clear)
        form.addRow(row)
        lay.addWidget(box)
        tips = QLabel(
            "Try: enable the node in velocity mode at 0.5 rad/s, then inject a jam (FOLLOWING_ERROR), "
            "an encoder jump (ENCODER, homing lost) or overheating (OVERTEMP). Watch the event log and the "
            "node table, then clear the injection and the fault. Record the bus first to replay it later.")
        tips.setWordWrap(True)
        lay.addWidget(tips)
        lay.addStretch(1)
        self.kind.currentIndexChanged.connect(self._kind_changed)
        self.btn_inject.clicked.connect(self._inject)
        self.btn_clear.clicked.connect(self._clear)
        self._kind_changed()

    def _kind_changed(self, *_):
        kind = InjectKind(self.kind.currentData())
        label, default, text = INJECT_HELP[kind]
        self.value_label.setText(label)
        self.value.setValue(default)
        self.help.setText(text)

    def _inject(self) -> None:
        if self.win.connected and self.win.active_node:
            self.win.dm.inject(self.win.active_node, InjectKind(self.kind.currentData()), self.value.value())

    def _clear(self) -> None:
        if self.win.connected and self.win.active_node:
            self.win.dm.inject(self.win.active_node, InjectKind.CLEAR_ALL, 0.0)

    def refresh(self) -> None:
        pass


# ---------------------------------------------------------------------------------------------
class ReplayTab(QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        lay = QVBoxLayout(self)
        row = QHBoxLayout()
        self.btn_open = QPushButton("Open candump log…")
        self.path = QLabel("No log loaded. Record one with the ● Record button.")
        row.addWidget(self.btn_open)
        row.addWidget(self.path, 1)
        lay.addLayout(row)
        split = QSplitter(Qt.Orientation.Horizontal)
        self.summary = QPlainTextEdit()
        self.summary.setReadOnly(True)
        split.addWidget(self.summary)
        self.plots = pg.GraphicsLayoutWidget()
        self.p_pos = _make_plot(self.plots, 0, "Position", "rad")
        self.p_vel = _make_plot(self.plots, 1, "Velocity", "rad/s")
        self.p_cur = _make_plot(self.plots, 2, "Current", "A")
        for p in (self.p_vel, self.p_cur):
            p.setXLink(self.p_pos)
        self.p_cur.setLabel("bottom", "time since start of log", units="s")
        self.p_pos.addLegend()
        split.addWidget(self.plots)
        split.setSizes([350, 650])
        lay.addWidget(split, 1)
        self.btn_open.clicked.connect(self._open)

    def _open(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open candump log", "recordings", "candump logs (*.log)")
        if path:
            self.load(path)

    def load(self, path: str) -> None:
        self.path.setText(f"Loading {path}…")
        self.win.runner.run("analyse log", analyze_log, path, on_done=self._show,
                            on_error=lambda e: self.path.setText(e))

    def _show(self, summary) -> None:
        self.path.setText(summary.path)
        self.summary.setPlainText(summary.to_text())
        for p in (self.p_pos, self.p_vel, self.p_cur):
            p.clear()
        colors = ["#1565c0", "#c62828", "#2e7d32", "#6a1b9a"]
        t0 = None
        for i, n in enumerate(sorted(summary.dm.nodes)):
            ns = summary.dm.nodes[n]
            if not ns.t:
                continue
            t0 = ns.t[0] if t0 is None else min(t0, ns.t[0])
        for i, n in enumerate(sorted(summary.dm.nodes)):
            ns = summary.dm.nodes[n]
            if not ns.t:
                continue
            x = [t - t0 for t in ns.t]
            pen = pg.mkPen(colors[i % len(colors)], width=1.4)
            self.p_pos.plot(x, list(ns.pos), pen=pen, name=f"node {n}")
            self.p_vel.plot(x, list(ns.vel), pen=pen)
            self.p_cur.plot(x, list(ns.cur), pen=pen)
        if t0 is not None:
            for f_t, node, text in summary.faults:
                for p in (self.p_pos, self.p_vel, self.p_cur):
                    line = pg.InfiniteLine(pos=f_t, angle=90, pen=pg.mkPen("#c62828", style=Qt.PenStyle.DotLine))
                    p.addItem(line)

    def refresh(self) -> None:
        pass


__all__ = ["ControlTab", "ParamsTab", "CalibrationTab", "TestTab", "BusTab", "FaultTab", "ReplayTab"]
