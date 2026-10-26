"""Main window: connection bar, E-STOP, node table, tabs, event log.

    bringup-gui                      # connects to tcp://localhost:29536
    bringup-gui --url /dev/cu.usbserial-0001
"""

from __future__ import annotations

import argparse
import sys
import time

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QDockWidget,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QToolBar,
)

from ..devices import DeviceManager
from ..params import ParamClient
from ..recorder import Recorder
from ..transport import LinkError, SlcanLink
from ..ui_model import NODE_COLUMNS, SEVERITY_COLORS, event_line, node_row
from .tabs import BusTab, CalibrationTab, ControlTab, FaultTab, ParamsTab, ReplayTab, TestTab
from .worker import TaskRunner

ROW_TINT = {"fault": "#ffebee", "ok": "#e8f5e9", "warn": "#fff8e1", "muted": "#ffffff"}


class MainWindow(QMainWindow):
    def __init__(self, url: str = "tcp://localhost:29536", autoconnect: bool = False):
        super().__init__()
        self.setWindowTitle("Actuator Bring-Up Console")
        self.resize(1400, 860)
        self.link: SlcanLink | None = None
        self.dm: DeviceManager | None = None
        self.pc: ParamClient | None = None
        self.recorder: Recorder | None = None
        self.runner = TaskRunner()
        self._active_node: int | None = None
        self._last_event = None

        # --- toolbar ------------------------------------------------------------
        bar = QToolBar("Connection")
        bar.setMovable(False)
        self.addToolBar(bar)
        bar.addWidget(QLabel(" Link "))
        self.url = QLineEdit(url)
        self.url.setMinimumWidth(260)
        bar.addWidget(self.url)
        self.btn_connect = QPushButton("Connect")
        bar.addWidget(self.btn_connect)
        bar.addSeparator()
        self.btn_record = QPushButton("● Record")
        self.btn_record.setCheckable(True)
        bar.addWidget(self.btn_record)
        bar.addSeparator()
        self.btn_estop = QPushButton("E-STOP  (Esc)")
        self.btn_estop.setMinimumHeight(34)
        self.btn_estop.setStyleSheet(
            "QPushButton { background: #c62828; color: white; font-weight: 700; padding: 4px 18px; border-radius: 4px; }"
            "QPushButton:pressed { background: #8e0000; }")
        bar.addWidget(self.btn_estop)
        self.active_label = QLabel("  no node selected")
        bar.addWidget(self.active_label)

        # --- node table (left dock) ------------------------------------------------
        self.node_table = QTableWidget(0, len(NODE_COLUMNS))
        self.node_table.setHorizontalHeaderLabels(NODE_COLUMNS)
        self.node_table.verticalHeader().setVisible(False)
        self.node_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.node_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.node_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.node_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        dock = QDockWidget("Actuators on the bus", self)
        dock.setWidget(self.node_table)
        dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable | QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        self.addDockWidget(Qt.DockWidgetArea.TopDockWidgetArea, dock)

        # --- tabs -------------------------------------------------------------------
        self.tabs = QTabWidget()
        self.control_tab = ControlTab(self)
        self.params_tab = ParamsTab(self)
        self.cal_tab = CalibrationTab(self)
        self.test_tab = TestTab(self)
        self.bus_tab = BusTab(self)
        self.fault_tab = FaultTab(self)
        self.replay_tab = ReplayTab(self)
        for tab, name in ((self.control_tab, "Control && plots"), (self.params_tab, "Parameters"),
                          (self.cal_tab, "Calibration"), (self.test_tab, "Bring-up test"),
                          (self.bus_tab, "Bus monitor"), (self.fault_tab, "Fault injection"),
                          (self.replay_tab, "Log replay")):
            self.tabs.addTab(tab, name)
        self.setCentralWidget(self.tabs)

        # --- event log (bottom dock) ------------------------------------------------
        self.events = QPlainTextEdit()
        self.events.setReadOnly(True)
        self.events.setMaximumBlockCount(2000)
        self.events.setStyleSheet("font-family: Menlo, Consolas, monospace; font-size: 11px;")
        log_dock = QDockWidget("Events and faults", self)
        log_dock.setWidget(self.events)
        log_dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable | QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, log_dock)

        self.status_label = QLabel("Not connected")
        self.statusBar().addPermanentWidget(self.status_label, 1)

        # --- wiring -------------------------------------------------------------------
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.btn_record.toggled.connect(self.toggle_recording)
        self.btn_estop.clicked.connect(self.estop)
        self.node_table.itemSelectionChanged.connect(self._selection_changed)
        self.estop_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        self.estop_shortcut.setContext(Qt.ShortcutContext.ApplicationShortcut)
        self.estop_shortcut.activated.connect(self.estop)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)
        if autoconnect:
            QTimer.singleShot(100, self.connect_link)

    # --- state ------------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self.link is not None and self.link.connected and self.dm is not None

    @property
    def active_node(self) -> int | None:
        return self._active_node

    def log(self, text: str, kind: str = "info") -> None:
        color = SEVERITY_COLORS.get(kind, "#000000")
        stamp = time.strftime("%H:%M:%S")
        self.events.appendHtml(f'<span style="color:{color}">{stamp}  host    {_escape(text)}</span>')

    # --- connection -----------------------------------------------------------------------
    def toggle_connection(self) -> None:
        if self.link is not None:
            self.disconnect_link()
        else:
            self.connect_link()

    def connect_link(self) -> None:
        url = self.url.text().strip()
        dm = DeviceManager()
        link = SlcanLink(url, dm.on_frame)
        try:
            link.open()
        except LinkError as exc:
            QMessageBox.warning(self, "Cannot connect", str(exc))
            return
        dm.link = link
        dm.start()
        self.link, self.dm, self.pc = link, dm, ParamClient(dm)
        self.bus_tab.attach(dm)
        self.btn_connect.setText("Disconnect")
        self.url.setEnabled(False)
        self.log(f"connected to {url}", "info")

    def disconnect_link(self) -> None:
        if self.recorder and self.recorder.active:
            self.btn_record.setChecked(False)
        if self.dm:
            for n in self.dm.node_ids():
                self.dm.disable(n)
            time.sleep(0.05)  # let the disable go out
            self.dm.stop()
        if self.link:
            self.link.close()
        self.link = self.dm = self.pc = None
        self._active_node = None
        self.node_table.setRowCount(0)
        self.btn_connect.setText("Connect")
        self.url.setEnabled(True)
        self.log("disconnected", "warn")

    def toggle_recording(self, on: bool) -> None:
        if on:
            if not self.connected:
                self.btn_record.setChecked(False)
                return
            self.recorder = Recorder(self.dm)
            log_path, csv_path = self.recorder.start("recordings")
            self.btn_record.setText("■ Stop recording")
            self.log(f"recording to {log_path} and {csv_path.name}", "action")
        elif self.recorder:
            paths = self.recorder.stop()
            self.btn_record.setText("● Record")
            if paths:
                self.log(f"saved {self.recorder.frames} frames to {paths[0]}", "action")

    def estop(self) -> None:
        if self.connected:
            self.dm.estop()
            self.runner.stop_requested.set()
        else:
            self.log("E-STOP pressed but not connected", "warn")

    def _selection_changed(self) -> None:
        rows = self.node_table.selectionModel().selectedRows()
        if rows:
            item = self.node_table.item(rows[0].row(), 0)
            if item:
                self._active_node = int(item.text())
                self.active_label.setText(f"  active node: {self._active_node}")

    # --- periodic refresh ---------------------------------------------------------------------
    def refresh(self) -> None:
        for line in self.runner.poll():
            self.log(line, "info")
        if self.link is not None and not self.link.connected:
            self.log(f"link lost: {self.link.last_error}", "fault")
            self.disconnect_link()
            return
        if not self.connected:
            self.status_label.setText("Not connected. Start the simulator, then press Connect.")
            return
        self._refresh_nodes()
        self._refresh_events()
        current = self.tabs.currentWidget()
        if hasattr(current, "refresh"):
            current.refresh()
        if current is not self.bus_tab:
            self.bus_tab._pending.clear()
        if current is not self.cal_tab:
            self.cal_tab.refresh()
        stats = self.dm.bus_stats()
        rec = f" · recording {self.recorder.frames} frames" if self.recorder and self.recorder.active else ""
        task = f" · running: {self.runner.current}" if self.runner.busy else ""
        self.status_label.setText(f"{self.url.text()} · {len(self.dm.node_ids())} node(s) · "
                                  f"{stats['frames_per_s']:.0f} frames/s · bus load {stats['bus_load_pct']:.1f}%"
                                  f"{rec}{task}")

    def _refresh_nodes(self) -> None:
        ids = self.dm.node_ids()
        now = time.time()
        if self.node_table.rowCount() != len(ids):
            self.node_table.setRowCount(len(ids))
        for r, n in enumerate(ids):
            with self.dm.lock:
                cells, severity = node_row(self.dm.nodes[n], now)
            tint = QBrush(QColor(ROW_TINT.get(severity, "#ffffff")))
            for c, text in enumerate(cells):
                it = self.node_table.item(r, c)
                if it is None:
                    it = QTableWidgetItem(text)
                    self.node_table.setItem(r, c, it)
                elif it.text() != text:
                    it.setText(text)
                it.setBackground(tint)
        if self._active_node is None and ids:
            self.node_table.selectRow(0)

    def _refresh_events(self) -> None:
        with self.dm.lock:
            events = list(self.dm.events)
        start = 0
        if self._last_event is not None:
            for i in range(len(events) - 1, -1, -1):
                if events[i] is self._last_event:
                    start = i + 1
                    break
        for e in events[start:]:
            text, color = event_line(e)
            self.events.appendHtml(f'<span style="color:{color}">{_escape(text)}</span>')
        if events:
            self._last_event = events[-1]

    def closeEvent(self, event) -> None:  # noqa: N802 (Qt name)
        self.timer.stop()
        self.runner.shutdown()
        if self.link is not None:
            self.disconnect_link()
        event.accept()


def _escape(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="bringup-gui", description="Actuator bring-up and diagnostics console")
    ap.add_argument("--url", default="tcp://localhost:29536", help="SLCAN link: tcp://host:port or a serial port")
    ap.add_argument("--connect", action="store_true", help="connect on start-up")
    args = ap.parse_args(argv)
    pg.setConfigOptions(antialias=True, background="w", foreground="#333333")
    app = QApplication(sys.argv[:1])
    app.setApplicationName("Actuator Bring-Up Console")
    win = MainWindow(args.url, autoconnect=args.connect)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
