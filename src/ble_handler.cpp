#include "ble_handler.hpp"
#include "mbed.h"
#include "globals.hpp"
#include "ble/GattAttribute.h"


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

    // --- Descriptors to tell BLE clients these characteristics are UTF-8 text ---
    // 0x2904 = Characteristic Presentation Format Descriptor
    // Format 0x19 = UTF-8 string (Bluetooth SIG Assigned Numbers)
    static uint8_t cpf_utf8[7] = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // 0x2901 = Characteristic User Description Descriptor
    static uint8_t tremor_desc[]     = "Tremor";
    static uint8_t dyskinesia_desc[] = "Dyskinesia";
    static uint8_t fog_desc[]        = "Freezing of Gait";

    static GattAttribute tremor_cpf(UUID((uint16_t)0x2904), cpf_utf8, sizeof(cpf_utf8), sizeof(cpf_utf8), false);
    static GattAttribute dysk_cpf(UUID((uint16_t)0x2904), cpf_utf8, sizeof(cpf_utf8), sizeof(cpf_utf8), false);
    static GattAttribute fog_cpf(UUID((uint16_t)0x2904), cpf_utf8, sizeof(cpf_utf8), sizeof(cpf_utf8), false);

    static GattAttribute tremor_ud(UUID((uint16_t)0x2901), tremor_desc, sizeof(tremor_desc) - 1, sizeof(tremor_desc) - 1, false);
    static GattAttribute dysk_ud(UUID((uint16_t)0x2901), dyskinesia_desc, sizeof(dyskinesia_desc) - 1, sizeof(dyskinesia_desc) - 1, false);
    static GattAttribute fog_ud(UUID((uint16_t)0x2901), fog_desc, sizeof(fog_desc) - 1, sizeof(fog_desc) - 1, false);

    // Build descriptor tables for each characteristic
    GattAttribute *tremor_attrs[] = {&tremor_ud, &tremor_cpf};
    GattAttribute *dysk_attrs[] = {&dysk_ud, &dysk_cpf};
    GattAttribute *fog_attrs[] = {&fog_ud, &fog_cpf};

    // Create new characteristics with descriptors
    GattCharacteristic tremor_char(
        UUID(TREMOR_CHAR_UUID),
        _tremor_buffer,
        0,
        sizeof(_tremor_buffer),
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ,
        tremor_attrs,
        sizeof(tremor_attrs) / sizeof(GattAttribute *)
    );

    GattCharacteristic dysk_char(
        UUID(DYSKINESIA_CHAR_UUID),
        _dyskinesia_buffer,
        0,
        sizeof(_dyskinesia_buffer),
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ,
        dysk_attrs,
        sizeof(dysk_attrs) / sizeof(GattAttribute *)
    );

    GattCharacteristic fog_char(
        UUID(FOG_CHAR_UUID),
        _fog_buffer,
        0,
        sizeof(_fog_buffer),
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ,
        fog_attrs,
        sizeof(fog_attrs) / sizeof(GattAttribute *)
    );

    GattCharacteristic *charTable[] = {&tremor_char, &dysk_char, &fog_char};
    GattService parkinsonService(
        UUID(PARKINSON_SERVICE_UUID),
        charTable,
        sizeof(charTable) / sizeof(GattCharacteristic *)
    );

    ble.gattServer().addService(parkinsonService);

    _tremor_handle = tremor_char.getValueHandle();
    _dyskinesia_handle = dysk_char.getValueHandle();
    _fog_handle = fog_char.getValueHandle();

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
    int len = snprintf((char *)_tremor_buffer, sizeof(_tremor_buffer), "Tremor: %.2f", value);
    if (len > 0) {
        _ble.gattServer().write(
            _tremor_handle,
            _tremor_buffer,
            (uint16_t)len
        );
    }
}

void ParkinsonBLE::updateDyskinesia(float value) {
    int len = snprintf((char *)_dyskinesia_buffer, sizeof(_dyskinesia_buffer), "Dyskinesia: %.2f", value);
    if (len > 0) {
        _ble.gattServer().write(
            _dyskinesia_handle,
            _dyskinesia_buffer,
            (uint16_t)len
        );
    }
}

void ParkinsonBLE::updateFreezingGait(float value) {
    int len = snprintf((char *)_fog_buffer, sizeof(_fog_buffer), "Freezing of Gait: %.2f", value);
    if (len > 0) {
        _ble.gattServer().write(
            _fog_handle,
            _fog_buffer,
            (uint16_t)len
        );
    }
}
