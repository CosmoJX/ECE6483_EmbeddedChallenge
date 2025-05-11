#include <mbed.h>
#include "ble/BLE.h"
#include "ble/gatt/GattService.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "events/EventQueue.h"
#include <string.h>
#include <vector>
#include "arm_math.h"

FileHandle *mbed::mbed_override_console(int);
void schedule_ble_events(ble::BLE::OnEventsToProcessCallbackContext *context);
void on_ble_init_complete(ble::BLE::InitializationCompleteCallbackContext *params);

