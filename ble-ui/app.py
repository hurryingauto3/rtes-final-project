from __future__ import annotations

import asyncio
import sys
from typing import Dict, Optional

from PySide6.QtWidgets import QApplication
from bleak.backends.device import BLEDevice
from qasync import QEventLoop

from ble_client import BleMonitorClient
from config import DEVICE_NAME_FILTER
from ui_main import MainWindow
from utils import current_time


class AppController:
    """Glue between the Qt UI and the async BLE client."""

    def __init__(self, window: MainWindow, client: BleMonitorClient) -> None:
        self.window = window
        self.client = client
        self.client.log_callback = self.window.append_log
        self._devices_by_address: Dict[str, BLEDevice] = {}
        self._test_mode: bool = False

        # Wire UI signals
        self.window.scan_requested.connect(self.on_scan_requested)
        self.window.connect_requested.connect(self.on_connect_requested)
        self.window.disconnect_requested.connect(self.on_disconnect_requested)
        self.window.test_mode_toggled.connect(self.on_test_mode_toggled)

    # ------------------------------------------------------------------
    # UI signal handlers
    # ------------------------------------------------------------------
    def on_test_mode_toggled(self, enabled: bool) -> None:
        self._test_mode = enabled

    def on_scan_requested(self) -> None:
        if self._test_mode:
            self.window.append_log("Scan ignored: Test Mode is enabled.")
            return
        asyncio.create_task(self._scan())

    def on_connect_requested(self) -> None:
        if self._test_mode:
            self.window.append_log("Connect ignored: Test Mode is enabled.")
            return
        asyncio.create_task(self._connect_and_subscribe())

    def on_disconnect_requested(self) -> None:
        if self._test_mode:
            self.window.append_log("Disconnect ignored: Test Mode is enabled.")
            return
        asyncio.create_task(self._disconnect())

    # ------------------------------------------------------------------
    # Async workers
    # ------------------------------------------------------------------
    async def _scan(self) -> None:
        self.window.set_connection_state("SCANNING")
        try:
            devices = await self.client.scan(timeout=5.0)
        except Exception as exc:
            self.window.append_log(f"[ERROR] Scan failed: {exc}")
            self.window.set_connection_state("DISCONNECTED")
            return

        self._devices_by_address.clear()
        entries = []
        for d in devices:
            self._devices_by_address[d.address] = d
            entries.append((d.name or "", d.address))
        self.window.populate_devices(entries)
        self.window.set_connection_state("DISCONNECTED")

    async def _connect_and_subscribe(self) -> None:
        address = self.window.get_selected_device_address()
        if not address:
            self.window.append_log("No device selected for connection.")
            return

        device: Optional[BLEDevice] = self._devices_by_address.get(address)
        if device is None:
            self.window.append_log(f"Selected device {address} not in cache.")
            return

        self.window.set_connection_state("CONNECTING")
        try:
            await self.client.connect(device)
        except Exception as exc:
            self.window.append_log(f"[ERROR] Connect failed: {exc}")
            self.window.set_connection_state("DISCONNECTED")
            return

        # Start notifications
        try:
            await self.client.start_notifications(
                self._on_tremor_notification,
                self._on_dyskinesia_notification,
                self._on_fog_notification,
            )
        except Exception as exc:
            self.window.append_log(f"[ERROR] Failed to start notifications: {exc}")
            await self.client.disconnect()
            self.window.set_connection_state("DISCONNECTED")
            return

        self.window.set_connection_state("CONNECTED")

    async def _disconnect(self) -> None:
        self.window.set_connection_state("DISCONNECTED")
        try:
            await self.client.disconnect()
        except Exception as exc:
            self.window.append_log(f"[ERROR] Disconnect failed: {exc}")

    # ------------------------------------------------------------------
    # BLE notification handlers
    # ------------------------------------------------------------------
    def _on_tremor_notification(self, text: str, _raw: bytes) -> None:
        now = current_time()
        self.window.update_tremor(text, now)

    def _on_dyskinesia_notification(self, text: str, _raw: bytes) -> None:
        now = current_time()
        self.window.update_dyskinesia(text, now)

    def _on_fog_notification(self, text: str, _raw: bytes) -> None:
        now = current_time()
        # Special one-shot marker: the firmware sends "Freezing of Gait: -0.10"
        # right after FOG calibration completes. Treat any negative FOG value
        # as a calibration notification and log it clearly for the user.
        try:
            # Expect format "Freezing of Gait: <value>"
            parts = text.split(":")
            if len(parts) >= 2:
                value_str = parts[-1].strip()
                value = float(value_str)
                if value < 0.0:
                    self.window.append_log("FOG calibration complete (negative marker received).")
        except Exception:
            # If parsing fails, just ignore and show the raw text
            pass

        self.window.update_fog(text, now)


def main() -> None:
    app = QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    window = MainWindow()
    client = BleMonitorClient()
    controller = AppController(window, client)

    # Initial status
    window.append_log(
        f"Ready. Looking for devices with name containing '{DEVICE_NAME_FILTER}'."
    )

    # Ensure clean shutdown
    def on_about_to_quit() -> None:
        if client is not None:
            # Schedule disconnect but do not block UI
            asyncio.create_task(client.disconnect())

    app.aboutToQuit.connect(on_about_to_quit)

    window.show()
    with loop:
        loop.run_forever()


if __name__ == "__main__":
    main()


