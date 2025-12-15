#!/usr/bin/env python3
"""Build BLE stack sources from Mbed framework."""
# pylint: disable=undefined-variable
Import("env")

import os
import glob

FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-mbed")
ble_root = os.path.join(FRAMEWORK_DIR, "connectivity", "FEATURE_BLE")

def should_include(path):
    """Filter source files."""
    low = path.lower()
    # Exclude tests/examples
    if any(x in low for x in ["/test/", "\\test\\", "/tests/", "\\tests\\", "/example", "\\example"]):
        return False
    # Exclude TARGET_CORDIO_LL (for Nordic chips, not BlueNRG)
    if "target_cordio_ll" in low:
        return False
    return True

# Collect BLE stack sources
patterns = [
    os.path.join(ble_root, "source", "**", "*.c"),
    os.path.join(ble_root, "source", "**", "*.cpp"),
    os.path.join(ble_root, "libraries", "**", "*.c"),
    os.path.join(ble_root, "libraries", "**", "*.cpp"),
]

sources = []
for pattern in patterns:
    sources.extend(glob.glob(pattern, recursive=True))

sources = [s for s in sources if should_include(s)]

# Add include paths
include_paths = [
    os.path.join(ble_root, "include"),
    os.path.join(ble_root, "source"),
    os.path.join(ble_root, "source", "cordio"),
]

# Add Cordio library include directories
cordio_libs = os.path.join(ble_root, "libraries")
if os.path.isdir(cordio_libs):
    for root, dirs, files in os.walk(cordio_libs):
        if "target_cordio_ll" not in root.lower() and any(f.endswith(".h") for f in files):
            include_paths.append(root)

env.Append(CPPPATH=include_paths)

# Build BLE stack as static library
ble_lib = env.Library("mbed_ble_stack", sources)
env.Append(LIBS=[ble_lib])

print(f"[BLE] Added {len(sources)} BLE stack sources")
