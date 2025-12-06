#include "ble_handler.hpp"
#include "mbed.h"
#include "globals.hpp"

void ParkinsonBLE::init() {
    _ble.onEventsToProcess(makeFunctionPointer(this, &ParkinsonBLE::schedule_ble_events));

    _ble.init(this, &ParkinsonBLE::on_init_complete);
}

void ParkinsonBLE::on_init_complete(ble::BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        #ifdef DEBUG
        printf("BLE initialization failed.\n");
        #endif
        return;
    }

    #ifdef DEBUG
    printf("BLE initialized.\n");
    #endif

    ble::BLE &ble = params->ble;
    ble.gap().setEventHandler(this);

    GattCharacteristic *charTable[] = {&_tremor_char, &_dyskinesia_char, &_fog_char};
    GattService parkinsonService(
        UUID(PARKINSON_SERVICE_UUID),
        charTable,
        sizeof(charTable) / sizeof(GattCharacteristic *)
    );

    ble.gattServer().addService(parkinsonService);

    _tremor_handle = _tremor_char.getValueHandle();
    _dyskinesia_handle = _dyskinesia_char.getValueHandle();
    _fog_handle = _fog_char.getValueHandle();

    start_advertising();
}

void ParkinsonBLE::start_advertising() {
    ble::AdvertisingParameters adv_parameters(
        ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
        ble::adv_interval_t(ble::millisecond_t(100))
    );

    _adv_data_builder.setFlags();
    _adv_data_builder.setName("ParkinsonMonitor");
    
    // Add service UUID to advertising data
    UUID service_uuid(PARKINSON_SERVICE_UUID);
    _adv_data_builder.setLocalServiceList(mbed::make_Span(&service_uuid, 1));

    ble_error_t error = _ble.gap().setAdvertisingParameters(
        ble::LEGACY_ADVERTISING_HANDLE,
        adv_parameters
    );

    if (error) {
        #ifdef DEBUG
        printf("BLE: _ble.gap().setAdvertisingParameters() failed\n");
        #endif
        return;
    }

    error = _ble.gap().setAdvertisingPayload(
        ble::LEGACY_ADVERTISING_HANDLE,
        _adv_data_builder.getAdvertisingData()
    );

    if (error) {
        #ifdef DEBUG
        printf("BLE: _ble.gap().setAdvertisingPayload() failed\n");
        #endif
        return;
    }

    error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

    if (error) {
        #ifdef DEBUG
        printf("BLE: _ble.gap().startAdvertising() failed\n");
        #endif
        return;
    }

    #ifdef DEBUG
    printf("BLE: Advertising started.\n");
    #endif
}

void ParkinsonBLE::schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    _event_queue.call(mbed::callback(&context->ble, &BLE::processEvents));
}

void ParkinsonBLE::onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
    #ifdef DEBUG
    if (event.getStatus() == BLE_ERROR_NONE) {
        printf("BLE: Connected to %02x:%02x:%02x:%02x:%02x:%02x\n",
               event.getPeerAddress()[5], event.getPeerAddress()[4], event.getPeerAddress()[3],
               event.getPeerAddress()[2], event.getPeerAddress()[1], event.getPeerAddress()[0]);
    }
    #endif
}

void ParkinsonBLE::onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) {
    #ifdef DEBUG
    printf("BLE: Disconnected. Reason: %u\n", event.getReason().value());
    #endif
    start_advertising();
}

void ParkinsonBLE::updateTremor(float value) {
    if (_tremor_value != value) {
        _tremor_value = value;
        _ble.gattServer().write(
            _tremor_handle,
            (uint8_t *)&_tremor_value,
            sizeof(_tremor_value)
        );
    }
}

void ParkinsonBLE::updateDyskinesia(float value) {
    if (_dyskinesia_value != value) {
        _dyskinesia_value = value;
        _ble.gattServer().write(
            _dyskinesia_handle,
            (uint8_t *)&_dyskinesia_value,
            sizeof(_dyskinesia_value)
        );
    }
}

void ParkinsonBLE::updateFreezingGait(float value) {
    if (_fog_value != value) {
        _fog_value = value;
        _ble.gattServer().write(
            _fog_handle,
            (uint8_t *)&_fog_value,
            sizeof(_fog_value)
        );
    }
}
