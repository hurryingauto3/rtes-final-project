# Board Configuration Guide

## Which Board Do You Have?

### Option 1: STM32L475VG Discovery Kit (B-L475E-IOT01A) ✅ RECOMMENDED
**Features**: Built-in BLE, WiFi, sensors
**Environment**: `disco_l475vg_iot01a` (default)
**Build command**: `platformio run -e disco_l475vg_iot01a`

### Option 2: STM32L475 without BLE (Serial Only)
**Use case**: Testing without BLE or if BLE isn't working
**Environment**: `disco_l475vg_iot01a_serial`
**Build command**: `platformio run -e disco_l475vg_iot01a_serial`

### Option 3: STM32F429ZI Discovery Kit (32F429IDISCOVERY)
**Features**: NO BLE support, display, different processor
**Environment**: `stm32f429_discovery`
**Build command**: `platformio run -e stm32f429_discovery`

## How to Switch Between Configurations

### Method 1: Command Line
```bash
# Build with BLE (default)
platformio run -e disco_l475vg_iot01a

# Build serial-only version
platformio run -e disco_l475vg_iot01a_serial

# Build for STM32F429 board
platformio run -e stm32f429_discovery
```

### Method 2: VS Code Tasks
The PlatformIO extension automatically creates tasks for each environment.

## What Changed?

1. **Created `output_handler.hpp`**: Abstraction layer that works with or without BLE
2. **Updated `main.cpp`**: Uses `OutputHandler` instead of direct BLE calls
3. **Added 3 build environments** in `platformio.ini`:
   - BLE-enabled (L475)
   - Serial-only (L475)
   - STM32F429 support

## Data Output

### With BLE (`USE_BLE_OUTPUT=1`):
- Data sent via Bluetooth Low Energy
- Also printed to serial for debugging

### Without BLE (`USE_BLE_OUTPUT=0`):
- Data only printed to serial port
- View with: `platformio device monitor`

## Quick Troubleshooting

**If BLE linking fails:**
→ Use `disco_l475vg_iot01a_serial` environment

**If you have STM32F429:**
→ Use `stm32f429_discovery` environment (no BLE support)

**To verify your board:**
```bash
platformio device list
```
Look for the hardware ID in the output.
