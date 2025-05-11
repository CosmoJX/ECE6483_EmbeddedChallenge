#include "mbed.h"
#include "ble/BLE.h"
#include "ble/gatt/GattService.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "events/EventQueue.h"
#include <string.h>
#include <vector>
#include "arm_math.h"
#include "i2c.h"
#include "fft.h"
#include "bluetooth.h"


// extern globals
extern enum symptom {NO_SYMPTOM, TREMOR, DYSKINESIA} flag;
extern float32_t intensity;

// initialize UART on USB
BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) { return &serial_port; }

using namespace ble;
using namespace events;
using namespace std::chrono;

ble::BLE &ble_interface = ble::BLE::Instance();  
EventQueue event_queue;
DigitalOut led(LED1);
Ticker ble_check_ticker; // trigger to update data

// UUIDs for the service and characteristics
const UUID SERVICE_UUID("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
const UUID STATE_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6"); // displays state - TREMOR, STABLE, DYSKINESIA
const UUID INTENSITY_CHAR_UUID("B1C2D3E4-F5A6-B7C8-D9E0-F1A2B3C4D5E6"); // displays intensity (32-bit value)

// State Strings
const char* STATE_STRINGS[] = { "STABLE", "TREMOR", "DYSKINESIA" };
#define MAX_STATE_STRING_LEN 11

// Intensity Values
uint8_t StateValue[MAX_STATE_STRING_LEN];
uint8_t IntensityValue = 0;

// initialize state characteristic
ReadOnlyArrayGattCharacteristic<uint8_t, MAX_STATE_STRING_LEN> StateCharacteristic(
    STATE_CHAR_UUID, StateValue, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

// initialize intensity characteristic
GattCharacteristic IntensityCharacteristic(
    INTENSITY_CHAR_UUID,
    (uint8_t*)&IntensityValue,
    sizeof(IntensityValue),
    sizeof(IntensityValue),
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

// initialize charactersitic disiplay 
GattCharacteristic *charTable[] = { &StateCharacteristic, &IntensityCharacteristic };
GattService DisorderService(SERVICE_UUID, charTable, sizeof(charTable) / sizeof(charTable[0]));
bool device_connected = false;

// Display Function  

void display(int flag, float32_t intensity) {
    if (!device_connected) {
        printf("No device connected. Skipping BLE update.\n");
        return;
    }

    // Clamp flag to valid range [0, 2]
    if (flag < 0) flag = 0;
    if (flag > 2) flag = 2;

    // Update State Characteristic
    strcpy((char*)StateValue, STATE_STRINGS[flag]);
    ble_interface.gattServer().write(
        StateCharacteristic.getValueHandle(), 
        StateValue, 
        strlen((char*)StateValue) + 1
    );

    // Update Intensity Characteristic
    IntensityValue = min((uint8_t)(intensity/11), (uint8_t)255);
     
    ble_interface.gattServer().write(
        IntensityCharacteristic.getValueHandle(), 
        (uint8_t*)&IntensityValue, 
        sizeof(IntensityValue)
    );

    printf("BLE Update -> State: %s, Intensity: %f\n", (char*)StateValue, intensity);
    led = !led;  // Blink LED to show transmission
}

// function to be called after connection 
void periodic_check() {
    if (device_connected) {
        // fill the data_array by reading from the IMU
        read_acceleration();
        // perform FFT operation on data_array
        run_fft();
        // check for tremor/dyskines symptoms over multiple sampling periods
        // modify flag and intensity
        detect_tremor_and_dyskinesia();
        // display using bluetooth
        display(flag, intensity);
    } else {
        printf("Waiting for BLE connection...\n");
    }
}

class ConnectionEventHandler : public ble::Gap::EventHandler {
public:
    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            printf("Device connected!\n");
            device_connected = true;
            ble_check_ticker.attach([]() { 
            event_queue.call(periodic_check);  // Schedule display safely on EventQueue 
            }, 6s);
        }
    }

    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) {
        printf("Device disconnected!\n");
        device_connected = false;
        ble_interface.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        printf("Restarted advertising.\n");
    }
};

ConnectionEventHandler connection_handler;

void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        printf("BLE initialization failed.\n");
        return;
    }

    // Initialize Characteristics
    strcpy((char*)StateValue, "STABLE");
    IntensityValue = 0;

    ble_interface.gattServer().addService(DisorderService);

    uint8_t adv_buffer[LEGACY_ADVERTISING_MAX_SIZE];
    AdvertisingDataBuilder adv_data(adv_buffer);

    adv_data.setFlags();
    adv_data.setName("Parkinson's-Monitor");

    ble_interface.gap().setAdvertisingParameters(
        LEGACY_ADVERTISING_HANDLE,
        AdvertisingParameters(advertising_type_t::CONNECTABLE_UNDIRECTED, adv_interval_t(160))
    );

    ble_interface.gap().setAdvertisingPayload(
        LEGACY_ADVERTISING_HANDLE,
        adv_data.getAdvertisingData()
    );

    ble_interface.gap().setEventHandler(&connection_handler);
    ble_interface.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

    printf("BLE advertising started. Waiting for connection...\n");
}

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(callback(&ble_interface, &BLE::processEvents));
}