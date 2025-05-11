#include <mbed.h>
#include "i2c.h"
#include "fft.h"
#include "bluetooth.h"

// setup I2C for accelerometer
I2C i2c(PB_11, PB_10);  // I2C2: SDA = PB11, SCL = PB10

// global data for FFT operations
float32_t data_array[FFT_SIZE];
float32_t fft_out[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];
float32_t SAMPLE_RATE = 104.0f; // sampling rate of accelerometer
float32_t intensity;            // tremor/dyskinesia intensity

// global data for tremor/dyskinesia detection
enum symptom {NO_SYMPTOM, TREMOR, DYSKINESIA} flag;
arm_rfft_fast_instance_f32 FFT_Instance;

// global data for Bluetooth
using namespace ble;
using namespace events;
using namespace std::chrono;
extern EventQueue event_queue;
extern BLE &ble_interface ; 

int main(void) {

    // initialize i2c connection and IMU config
    i2c_init();
    // initialize FFT instance
    arm_status status = arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);
    if (status != ARM_MATH_SUCCESS) {
        printf("FFT initialization failed\r\n");
        while(1);
    }
    ble_interface.onEventsToProcess(schedule_ble_events);
    ble_interface.init(on_ble_init_complete);
    event_queue.dispatch_forever();

    return 0;
}