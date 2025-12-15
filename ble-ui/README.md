# ParkinsonMonitor BLE UI

Desktop Python application that connects via Bluetooth Low Energy (BLE) to an STM32 board advertising as `ParkinsonMonitor`, subscribes to three UTF-8 BLE characteristics, and displays them live in a responsive UI.

## Features

- Scan and list BLE devices, highlighting those whose name contains `ParkinsonMonitor`.
- Connect / disconnect to a single device at a time.
- Subscribe to three UTF-8 notification characteristics:
  - Tremor
  - Dyskinesia
  - Freezing of Gait
- Live data display in three large panels showing:
  - Latest received text
  - Timestamp of last update
  - Status badge (`LIVE` if updated within 5s, otherwise `STALE`)
- Logging panel for:
  - Scan events
  - Connect / disconnect
  - Notifications (decoded UTF-8 + raw hex)
  - Errors / exceptions
- Test Mode:
  - When enabled, disables BLE operations.
  - Simulates updates every 500 ms:
    - `Tremor: 0.12`
    - `Dyskinesia: 0.48`
    - `Freezing of Gait: 0.00`

## Project Layout

- `config.py` – BLE UUIDs, device name filter, staleness configuration.
- `utils.py` – Timestamp helpers, log buffer, formatting utilities.
- `ble_client.py` – Async BLE client using `bleak`.
- `ui_main.py` – PySide6 UI definition.
- `app.py` – Application entry point, Qt/asyncio integration, wiring UI ⇄ BLE.

## Setup

From the `ble-ui` directory:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python app.py
```

On Windows, activate the virtual environment with:

```bash
.venv\\Scripts\\activate
```

## macOS Bluetooth Permissions

On macOS, ensure that the process running the app (e.g. Terminal, iTerm, VS Code) is allowed to use Bluetooth:

- Open System Settings → Privacy & Security → Bluetooth.
- Make sure your terminal or Python host application is listed and enabled.

Also note:

- Only one BLE central can usually connect to your board at a time.
- If you use tools like LightBlue or nRF Connect, disconnect them before running this app.

## Test Mode

- Toggle **Test Mode** from the toolbar.
- While Test Mode is enabled:
  - BLE scan / connect / disconnect are ignored.
  - Simulated data is emitted every 500 ms to all three panels.
  - Use this to demo the UI without hardware connected.


