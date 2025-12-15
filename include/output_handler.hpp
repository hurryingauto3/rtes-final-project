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
    }

    void sendDyskinesia(float value) {
#if USE_BLE_OUTPUT
        _ble_handler.updateDyskinesia(value);
#endif
    }

    void sendFreezingGait(float value) {
#if USE_BLE_OUTPUT
        _ble_handler.updateFreezingGait(value);
#endif
    }

private:
#if USE_BLE_OUTPUT
    ParkinsonBLE _ble_handler;
#endif
};
