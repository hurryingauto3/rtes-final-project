from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Awaitable, Callable, Dict, Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

from config import (
    DEVICE_NAME_FILTER,
    DYSKINESIA_CHAR_UUID,
    FOG_CHAR_UUID,
    PARKINSON_SERVICE_UUID,
    TREMOR_CHAR_UUID,
)
from utils import bytes_to_hex, format_log_line


LogCallback = Callable[[str], None]
DataCallback = Callable[[str, bytes], None]


@dataclass
class BleMonitorClient:
    """Async BLE client for the ParkinsonMonitor device."""

    log_callback: Optional[LogCallback] = None

    def __post_init__(self) -> None:
        self._client: Optional[BleakClient] = None
        self._connected_device: Optional[BLEDevice] = None
        self._notifications_started: bool = False
        self._notify_callbacks: Dict[str, DataCallback] = {}
        self._lock = asyncio.Lock()
        self._fog_poll_task: Optional[asyncio.Task] = None

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------
    def _log(self, message: str) -> None:
        if self.log_callback:
            self.log_callback(format_log_line(message))

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    async def scan(self, timeout: float = 5.0) -> list[BLEDevice]:
        """Scan for BLE devices."""
        self._log(f"Scanning for BLE devices (timeout={timeout}s)...")
        devices = await BleakScanner.discover(timeout=timeout)
        count = len(devices)
        self._log(f"Scan complete: found {count} device(s).")
        highlight = [d for d in devices if DEVICE_NAME_FILTER in (d.name or "")]
        for d in highlight:
            self._log(f"  Candidate: {d.name} [{d.address}]")
        return devices

    async def connect(self, device: BLEDevice) -> None:
        """Connect to the specified BLE device."""
        async with self._lock:
            if self._client and self._client.is_connected:
                self._log("Already connected; disconnecting first.")
                await self._client.disconnect()

            self._log(f"Connecting to {device.name} [{device.address}] ...")
            client = BleakClient(device)
            try:
                await client.connect()
            except Exception as exc:  # pragma: no cover - platform dependent
                self._log(f"Connection failed: {exc}")
                await client.disconnect()
                raise

            self._client = client
            self._connected_device = device
            # On modern bleak versions, services are discovered automatically on
            # first use; explicit service discovery via get_services() is no
            # longer required and the old method may not exist.
            self._log("Connected.")

    async def disconnect(self) -> None:
        """Disconnect from the current device."""
        async with self._lock:
            if not self._client:
                return
            if self._notifications_started:
                await self.stop_notifications()
            if self._client.is_connected:
                self._log("Disconnecting from device...")
                try:
                    await self._client.disconnect()
                except Exception as exc:  # pragma: no cover - platform dependent
                    self._log(f"Error during disconnect: {exc}")
            self._client = None
            self._connected_device = None
            self._log("Disconnected.")

    async def start_notifications(
        self,
        on_tremor: DataCallback,
        on_dyskinesia: DataCallback,
        on_fog: DataCallback,
    ) -> None:
        """Subscribe to notifications on the three Parkinson service characteristics."""
        if not self._client or not self._client.is_connected:
            raise RuntimeError("Client not connected")

        self._notify_callbacks = {
            TREMOR_CHAR_UUID: on_tremor,
            DYSKINESIA_CHAR_UUID: on_dyskinesia,
            FOG_CHAR_UUID: on_fog,
        }

        def _notification_handler(char_uuid: str) -> Callable[[int, bytearray], None]:
            def inner(_handle: int, data: bytearray) -> None:
                text = data.decode("utf-8", errors="replace")
                hex_data = bytes_to_hex(data)
                self._log(
                    f"Notification from {char_uuid}: text='{text}' raw=[{hex_data}]"
                )
                cb = self._notify_callbacks.get(char_uuid)
                if cb:
                    # Pass along decoded text and raw bytes
                    cb(text, bytes(data))

            return inner

        self._log("Starting notifications for ParkinsonMonitor characteristics...")
        try:
            await self._client.start_notify(
                TREMOR_CHAR_UUID, _notification_handler(TREMOR_CHAR_UUID)
            )
            self._log(f"✓ Tremor notifications subscribed (UUID: {TREMOR_CHAR_UUID})")
            await self._client.start_notify(
                DYSKINESIA_CHAR_UUID,
                _notification_handler(DYSKINESIA_CHAR_UUID),
            )
            self._log(f"✓ Dyskinesia notifications subscribed (UUID: {DYSKINESIA_CHAR_UUID})")
            await self._client.start_notify(
                FOG_CHAR_UUID, _notification_handler(FOG_CHAR_UUID)
            )
            self._log(f"✓ FOG notifications subscribed (UUID: {FOG_CHAR_UUID})")
        except Exception as exc:  # pragma: no cover - platform dependent
            self._log(f"Error starting notifications: {exc}")
            raise
        else:
            self._notifications_started = True
            self._log("Notifications started.")

            # Start a background poller for FOG as a fallback if notifications
            # are flaky on this characteristic. This reads the FOG value once
            # per second and forwards it through the same callback.
            async def _poll_fog() -> None:
                assert self._client is not None
                while self._notifications_started and self._client and self._client.is_connected:
                    try:
                        data = await self._client.read_gatt_char(FOG_CHAR_UUID)
                        text = data.decode("utf-8", errors="replace")
                        hex_data = bytes_to_hex(data)
                        cb = self._notify_callbacks.get(FOG_CHAR_UUID)
                        self._log(f"[FOG poll] Read from {FOG_CHAR_UUID}: text='{text}' raw=[{hex_data}]")
                        if cb:
                            cb(text, bytes(data))
                    except Exception as exc:  # pragma: no cover - platform dependent
                        self._log(f"[FOG poll] Error reading FOG characteristic: {exc}")
                        # If read fails repeatedly, break out to avoid spamming logs
                        break
                    await asyncio.sleep(0.5)

            # Cancel any existing poller and start a new one
            if self._fog_poll_task is not None and not self._fog_poll_task.done():
                self._fog_poll_task.cancel()
            self._fog_poll_task = asyncio.create_task(_poll_fog())

    async def stop_notifications(self) -> None:
        """Stop notifications on all subscribed characteristics."""
        if not self._client or not self._client.is_connected:
            return

        self._log("Stopping notifications...")
        try:
            await self._client.stop_notify(TREMOR_CHAR_UUID)
            await self._client.stop_notify(DYSKINESIA_CHAR_UUID)
            await self._client.stop_notify(FOG_CHAR_UUID)
        except Exception as exc:  # pragma: no cover - platform dependent
            self._log(f"Error stopping notifications: {exc}")
        self._notifications_started = False
        # Stop FOG poller
        if self._fog_poll_task is not None and not self._fog_poll_task.done():
            self._fog_poll_task.cancel()
            self._fog_poll_task = None
        self._log("Notifications stopped.")
