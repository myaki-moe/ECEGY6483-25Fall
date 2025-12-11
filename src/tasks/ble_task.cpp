#include "tasks/ble_task.hpp"
#include "mbed.h"
#include "ble/BLE.h"
#include "ble/gatt/GattService.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "events/EventQueue.h"
#include <chrono>
#include <string.h>
#include "logger.hpp"
#include "main.hpp"
#include "tasks/analysis_task.hpp"


using namespace ble;
using namespace events;
using namespace std::chrono;

BLE &ble_interface = BLE::Instance();
EventQueue event_queue;

const UUID TREMOR_SERVICE_UUID("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
const UUID TREMOR_TYPE_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");

const char* TREMOR_STRING = "TREMOR";
const char* DYSKINESIA_STRING = "DYSKINESIA";
const char* FOG_STRING = "FOG";
const char* NONE_STRING = "NONE";

#define MAX_TREMOR_STRING_LEN 11
uint8_t TREMORValue[MAX_TREMOR_STRING_LEN];

ReadOnlyArrayGattCharacteristic<uint8_t, MAX_TREMOR_STRING_LEN>
TREMORTypeCharacteristic(
    TREMOR_TYPE_CHAR_UUID,
    TREMORValue,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

GattCharacteristic *charTable[] = { &TREMORTypeCharacteristic };
GattService TREMOR_Service(TREMOR_SERVICE_UUID, charTable, 1);

Ticker notification_ticker;
bool device_connected = false;

void send_TREMOR_notification() {
    if (!device_connected) {
        LOG_DEBUG("No device connected, skipping notification");
        return;
    }
    
    if (get_fog_status()) {
        strcpy((char*)TREMORValue, FOG_STRING);
    } else if (get_dyskinesia_status()) {
        strcpy((char*)TREMORValue, DYSKINESIA_STRING);
    } else if (get_tremor_status()) {
        strcpy((char*)TREMORValue, TREMOR_STRING);
    } else {
        strcpy((char*)TREMORValue, NONE_STRING);
    }
    
    ble_interface.gattServer().write(
        TREMORTypeCharacteristic.getValueHandle(),
        TREMORValue,
        strlen((char*)TREMORValue) + 1
    );
    
    LOG_DEBUG("Sent notification: %s", (char*)TREMORValue);
}

class ConnectionEventHandler : public ble::Gap::EventHandler {
public:
    virtual void onConnectionComplete(const
        ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            LOG_INFO("BLE device connected");
            device_connected = true;
            strcpy((char*)TREMORValue, TREMOR_STRING);
            
            notification_ticker.attach([]() {
                event_queue.call(send_TREMOR_notification);
            }, 1s);
        }
    }
    
    virtual void onDisconnectionComplete(const
        ble::DisconnectionCompleteEvent &event) {
        device_connected = false;
        notification_ticker.detach();
        ble_interface.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        LOG_INFO("BLE device disconnected, restarting advertising");
    }
};

ConnectionEventHandler connection_handler;

void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        LOG_FATAL("BLE initialization failed.");
        trigger_fatal_error();
        return;
    }
    
    strcpy((char*)TREMORValue, TREMOR_STRING);
    ble_interface.gattServer().addService(TREMOR_Service);
    
    uint8_t adv_buffer[LEGACY_ADVERTISING_MAX_SIZE];
    AdvertisingDataBuilder adv_data(adv_buffer);
    adv_data.setFlags();
    adv_data.setName(BLE_DEVICE_NAME);
    
    ble_interface.gap().setAdvertisingParameters(
        LEGACY_ADVERTISING_HANDLE,
        AdvertisingParameters(advertising_type_t::CONNECTABLE_UNDIRECTED,
                            adv_interval_t(160))
    );
    
    ble_interface.gap().setAdvertisingPayload(
        LEGACY_ADVERTISING_HANDLE,
        adv_data.getAdvertisingData()
    );
    
    ble_interface.gap().setEventHandler(&connection_handler);
    ble_interface.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);
    
    LOG_INFO("BLE advertising started as %s", BLE_DEVICE_NAME);
}

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(callback(&ble_interface, &BLE::processEvents));
}

void ble_task() {
    LOG_INFO("BLE Task Started");
    ble_interface.onEventsToProcess(schedule_ble_events);
    ble_interface.init(on_ble_init_complete);
    event_queue.dispatch_forever();
}

bool ble_is_connected() {
    return device_connected;
}
