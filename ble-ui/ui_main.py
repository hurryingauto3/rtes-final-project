from __future__ import annotations

from datetime import datetime
from typing import Iterable, Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QAction, QPainter
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QToolBar,
    QVBoxLayout,
    QWidget,
)
from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis

from config import DEVICE_NAME_FILTER, STALE_SECONDS
from utils import LogBuffer, age_seconds, format_timestamp


class DataPanel(QWidget):
    """Panel showing latest value, timestamp, and LIVE/STALE status."""

    def __init__(self, title: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._title = title
        self._last_update: Optional[datetime] = None
        self._max_points: int = 120
        self._history: list[float] = []

        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("font-size: 18px; font-weight: bold;")

        self.value_label = QLabel("-")
        self.value_label.setStyleSheet("font-size: 24px;")
        self.value_label.setWordWrap(True)

        self.timestamp_label = QLabel("Last update: -")

        self.status_label = QLabel("STALE")
        self.status_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._set_status_style(live=False)

        top_row = QHBoxLayout()
        top_row.addWidget(self.title_label)
        top_row.addStretch(1)
        top_row.addWidget(self.status_label)

        layout = QVBoxLayout(self)
        layout.addLayout(top_row)
        layout.addWidget(self.value_label)
        layout.addWidget(self.timestamp_label)

        # Time-series chart for this metric
        self.series = QLineSeries()
        self.chart = QChart()
        self.chart.addSeries(self.series)
        self.chart.setTitle(f"{title} History")
        self.chart.legend().hide()

        self.axis_x = QValueAxis()
        self.axis_x.setTitleText("Sample")
        self.axis_y = QValueAxis()
        self.axis_y.setTitleText("Value")
        self.axis_y.setRange(0.0, 1.0)

        self.chart.addAxis(self.axis_x, Qt.AlignBottom)
        self.chart.addAxis(self.axis_y, Qt.AlignLeft)
        self.series.attachAxis(self.axis_x)
        self.series.attachAxis(self.axis_y)

        self.chart_view = QChartView(self.chart)
        self.chart_view.setRenderHint(QPainter.Antialiasing)
        layout.addWidget(self.chart_view)

        self.setLayout(layout)
        self.setStyleSheet(
            "QWidget { border: 1px solid #ccc; border-radius: 4px; padding: 8px; }"
        )

    def _set_status_style(self, live: bool) -> None:
        if live:
            self.status_label.setText("LIVE")
            self.status_label.setStyleSheet(
                "QLabel { background-color: #4caf50; color: white; padding: 2px 6px; border-radius: 3px; }"
            )
        else:
            self.status_label.setText("STALE")
            self.status_label.setStyleSheet(
                "QLabel { background-color: #9e9e9e; color: white; padding: 2px 6px; border-radius: 3px; }"
            )

    def update_value(self, text: str, timestamp: datetime) -> None:
        self._last_update = timestamp
        self.value_label.setText(text)
        self.timestamp_label.setText(f"Last update: {format_timestamp(timestamp)}")
        # Update chart history if we can parse a numeric value
        value = self._extract_numeric_value(text)
        if value is not None:
            self._append_sample(value)
            self._update_chart()
            if value > 0.7:
                self._flash_threshold()
        self._update_status()

    def _extract_numeric_value(self, text: str) -> Optional[float]:
        """Best-effort extraction of the trailing numeric value from 'Label: value'."""
        try:
            parts = text.split(":")
            if len(parts) >= 2:
                value_str = parts[-1].strip()
                return float(value_str)
        except Exception:
            return None
        return None

    def _append_sample(self, value: float) -> None:
        self._history.append(value)
        if len(self._history) > self._max_points:
            self._history = self._history[-self._max_points :]

    def _update_chart(self) -> None:
        self.series.clear()
        if not self._history:
            return
        for idx, v in enumerate(self._history):
            self.series.append(float(idx), float(v))

        # Update axis ranges
        self.axis_x.setRange(0, max(1, len(self._history) - 1))
        min_v = min(self._history)
        max_v = max(self._history)
        if min_v == max_v:
            # Avoid zero-height range
            padding = 0.1 if max_v == 0 else abs(max_v) * 0.1
            min_v -= padding
            max_v += padding
        self.axis_y.setRange(min_v, max_v)

    def _flash_threshold(self) -> None:
        """Briefly flash the value label red when threshold is exceeded."""
        original_style = self.value_label.styleSheet()
        self.value_label.setStyleSheet(
            original_style + " QLabel { background-color: #ffcccc; }"
        )

        def _reset() -> None:
            self.value_label.setStyleSheet(original_style)

        QTimer.singleShot(200, _reset)

    def _update_status(self) -> None:
        age = age_seconds(self._last_update)
        live = age is not None and age < STALE_SECONDS
        self._set_status_style(live=live)

    def tick(self) -> None:
        """Called periodically to refresh LIVE/STALE state."""
        self._update_status()


class MainWindow(QMainWindow):
    # Signals to be used by the controller (app.py)
    scan_requested = Signal()
    connect_requested = Signal()
    disconnect_requested = Signal()
    test_mode_toggled = Signal(bool)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ParkinsonMonitor BLE UI")
        self.resize(1100, 700)

        self._log_buffer = LogBuffer()
        self._status = "DISCONNECTED"

        self._build_toolbar()
        self._build_central()

        # Timer to refresh LIVE/STALE status
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._on_tick)
        self._status_timer.start()

        # Test mode timer for simulated updates
        self._test_timer = QTimer(self)
        self._test_timer.setInterval(500)
        self._test_timer.timeout.connect(self._emit_test_updates)
        self._test_mode_enabled = False

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_toolbar(self) -> None:
        toolbar = QToolBar("Main", self)
        self.addToolBar(toolbar)

        self.scan_action = QAction("Scan", self)
        self.scan_action.triggered.connect(self.scan_requested)
        toolbar.addAction(self.scan_action)

        self.connect_action = QAction("Connect", self)
        self.connect_action.triggered.connect(self.connect_requested)
        toolbar.addAction(self.connect_action)

        self.disconnect_action = QAction("Disconnect", self)
        self.disconnect_action.triggered.connect(self.disconnect_requested)
        toolbar.addAction(self.disconnect_action)

        toolbar.addSeparator()

        self.test_mode_action = QAction("Test Mode", self)
        self.test_mode_action.setCheckable(True)
        self.test_mode_action.toggled.connect(self._on_test_mode_toggled)
        toolbar.addAction(self.test_mode_action)

        toolbar.addSeparator()

        self.status_label = QLabel("DISCONNECTED")
        toolbar.addWidget(self.status_label)

    def _build_central(self) -> None:
        central = QWidget(self)
        main_layout = QVBoxLayout(central)

        splitter = QSplitter(Qt.Horizontal, central)

        # Left: device list
        left_widget = QWidget(splitter)
        left_layout = QVBoxLayout(left_widget)
        left_label = QLabel("Devices")
        self.device_list = QListWidget()
        left_layout.addWidget(left_label)
        left_layout.addWidget(self.device_list)
        left_widget.setLayout(left_layout)

        # Right: data panels + log
        right_widget = QWidget(splitter)
        right_layout = QVBoxLayout(right_widget)

        panels_container = QWidget(right_widget)
        panels_layout = QHBoxLayout(panels_container)
        self.tremor_panel = DataPanel("Tremor", panels_container)
        self.dyskinesia_panel = DataPanel("Dyskinesia", panels_container)
        self.fog_panel = DataPanel("Freezing of Gait", panels_container)
        panels_layout.addWidget(self.tremor_panel)
        panels_layout.addWidget(self.dyskinesia_panel)
        panels_layout.addWidget(self.fog_panel)
        panels_container.setLayout(panels_layout)

        # Log panel
        log_container = QWidget(right_widget)
        log_layout = QVBoxLayout(log_container)
        log_header = QHBoxLayout()
        log_label = QLabel("Log")
        self.clear_log_button = QPushButton("Clear")
        self.copy_log_button = QPushButton("Copy")
        self.clear_log_button.clicked.connect(self._on_clear_logs)
        self.copy_log_button.clicked.connect(self._on_copy_logs)
        log_header.addWidget(log_label)
        log_header.addStretch(1)
        log_header.addWidget(self.clear_log_button)
        log_header.addWidget(self.copy_log_button)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)

        log_layout.addLayout(log_header)
        log_layout.addWidget(self.log_view)
        log_container.setLayout(log_layout)

        right_layout.addWidget(panels_container)
        right_layout.addWidget(log_container)
        right_widget.setLayout(right_layout)

        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)

        main_layout.addWidget(splitter)
        central.setLayout(main_layout)

        self.setCentralWidget(central)

    # ------------------------------------------------------------------
    # Public methods used by controller
    # ------------------------------------------------------------------
    def set_connection_state(self, state: str) -> None:
        self._status = state
        self.status_label.setText(state)

    def populate_devices(self, devices: Iterable[tuple[str, str]]) -> None:
        """Update device list with (name, address)."""
        self.device_list.clear()
        for name, address in devices:
            label = f"{name or '<unknown>'} [{address}]"
            item = QListWidgetItem(label)
            if DEVICE_NAME_FILTER and DEVICE_NAME_FILTER in (name or ""):
                item.setBackground(Qt.yellow)
            # store address for later retrieval
            item.setData(Qt.UserRole, address)
            self.device_list.addItem(item)

    def get_selected_device_address(self) -> Optional[str]:
        items = self.device_list.selectedItems()
        if not items:
            return None
        return items[0].data(Qt.UserRole)

    def append_log(self, line: str) -> None:
        self._log_buffer.append(line)
        self.log_view.setPlainText(self._log_buffer.get_text())
        self.log_view.verticalScrollBar().setValue(
            self.log_view.verticalScrollBar().maximum()
        )

    def clear_logs(self) -> None:
        self._log_buffer.clear()
        self.log_view.clear()

    # Metric updates
    def update_tremor(self, text: str, timestamp: datetime) -> None:
        self.tremor_panel.update_value(text, timestamp)

    def update_dyskinesia(self, text: str, timestamp: datetime) -> None:
        self.dyskinesia_panel.update_value(text, timestamp)

    def update_fog(self, text: str, timestamp: datetime) -> None:
        self.fog_panel.update_value(text, timestamp)

    # ------------------------------------------------------------------
    # Internal handlers
    # ------------------------------------------------------------------
    def _on_tick(self) -> None:
        self.tremor_panel.tick()
        self.dyskinesia_panel.tick()
        self.fog_panel.tick()

    def _on_clear_logs(self) -> None:
        self.clear_logs()

    def _on_copy_logs(self) -> None:
        clipboard = QApplication.clipboard()
        clipboard.setText(self._log_buffer.get_text())

    def _on_test_mode_toggled(self, enabled: bool) -> None:
        self._test_mode_enabled = enabled
        if enabled:
            self.append_log("Test Mode enabled: BLE actions disabled.")
            self._test_timer.start()
        else:
            self.append_log("Test Mode disabled.")
            self._test_timer.stop()
        self.test_mode_toggled.emit(enabled)

    def _emit_test_updates(self) -> None:
        if not self._test_mode_enabled:
            return
        now = datetime.now().astimezone()
        self.update_tremor("Tremor: 0.12", now)
        self.update_dyskinesia("Dyskinesia: 0.48", now)
        self.update_fog("Freezing of Gait: 0.00", now)
        self.append_log("Test Mode: simulated metric update.")


