#pragma once

#include "mbed.h"

// Configuration: Choose your output method
// Set to 1 for BLE, 0 for Serial only
#ifndef USE_BLE_OUTPUT
#define USE_BLE_OUTPUT 1
#endif

#if USE_BLE_OUTPUT
#include "ble_handler.hpp"
#endif

class OutputHandler {
public:
    OutputHandler(
#if USE_BLE_OUTPUT
        events::EventQueue &event_queue
#endif
    ) 
#if USE_BLE_OUTPUT
    : _ble_handler(event_queue)
#endif
    {
    }

    void init() {
#if USE_BLE_OUTPUT
        _ble_handler.init();
        printf("BLE output enabled\n");
#else
        printf("Serial-only output enabled\n");
#endif
    }

    void sendTremor(float value) {
#if USE_BLE_OUTPUT
        _ble_handler.updateTremor(value);
#endif
        // Always send via serial for debugging
        printf("Tremor: %.2f\n", value);
    }

    void sendDyskinesia(float value) {
#if USE_BLE_OUTPUT
        _ble_handler.updateDyskinesia(value);
#endif
        printf("Dyskinesia: %.2f\n", value);
    }

    void sendFreezingGait(float value) {
#if USE_BLE_OUTPUT
        _ble_handler.updateFreezingGait(value);
#endif
        printf("FOG: %.2f\n", value);
    }

private:
#if USE_BLE_OUTPUT
    ParkinsonBLE _ble_handler;
#endif
};
