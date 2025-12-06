#pragma once

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "events/mbed_events.h"
#include "globals.hpp"

// UUIDs for the Service and Characteristics
// You can generate your own UUIDs, these are placeholders
inline constexpr const char* PARKINSON_SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
inline constexpr const char* TREMOR_CHAR_UUID       = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
inline constexpr const char* DYSKINESIA_CHAR_UUID   = "825eef3b-e10c-4a60-9b9c-f929c1e997b9";
inline constexpr const char* FOG_CHAR_UUID          = "c7333083-b830-4542-97c3-07027f51f404";

class ParkinsonBLE : private mbed::NonCopyable<ParkinsonBLE>, public ble::Gap::EventHandler {
public:
    ParkinsonBLE(events::EventQueue &event_queue) :
        _event_queue(event_queue),
        _ble(ble::BLE::Instance()),
        _tremor_char(
            UUID(TREMOR_CHAR_UUID),
            (uint8_t *)&_tremor_value,
            sizeof(float),
            sizeof(float),
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
        ),
        _dyskinesia_char(
            UUID(DYSKINESIA_CHAR_UUID),
            (uint8_t *)&_dyskinesia_value,
            sizeof(float),
            sizeof(float),
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
        ),
        _fog_char(
            UUID(FOG_CHAR_UUID),
            (uint8_t *)&_fog_value,
            sizeof(float),
            sizeof(float),
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
        ),
        _adv_data_builder(_adv_buffer, sizeof(_adv_buffer))
    {
    }

    ~ParkinsonBLE() {}

    void init();

    void updateTremor(float value);

    void updateDyskinesia(float value);

    void updateFreezingGait(float value);

private:

    void on_init_complete(ble::BLE::InitializationCompleteCallbackContext *params);

    void start_advertising();

    void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context);

    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) override;
    
    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) override;

private:
    events::EventQueue &_event_queue;
    ble::BLE &_ble;

    float _tremor_value = 0.0f;
    float _dyskinesia_value = 0.0f;
    float _fog_value = 0.0f;

    GattCharacteristic _tremor_char;
    GattCharacteristic _dyskinesia_char;
    GattCharacteristic _fog_char;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;

    GattAttribute::Handle_t _tremor_handle = 0;
    GattAttribute::Handle_t _dyskinesia_handle = 0;
    GattAttribute::Handle_t _fog_handle = 0;
};
