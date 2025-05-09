#include <mbed.h>
#include "i2c.h"
#include "fft.h"

// setup I2C for accelerometer
I2C i2c(PB_11, PB_10);  // I2C2: SDA = PB11, SCL = PB10

// global data for FFT operations
float32_t data_array[FFT_SIZE];
float32_t fft_out[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];
float32_t SAMPLE_RATE = 104.0f; // sampling rate of accelerometer
float32_t intensity;            // tremor/dyskinesia intensity

enum symptom {NO_SYMPTOM, TREMOR, DYSKINESIA} flag;

arm_rfft_fast_instance_f32 FFT_Instance;

int main(void) {

    // initialize i2c connection and IMU config
    i2c_init();
    // initialize FFT instance
    arm_status status = arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);
    if (status != ARM_MATH_SUCCESS) {
        while(1) printf("FFT initialization failed\r\n");
    }
    // Main loop
    while (1) {
        // fill the data_array by reading from the IMU
        read_acceleration();
        // perform FFT operation on data_array
        run_fft();
        show_results();
        detect_tremor_and_dyskinesia();

        if (flag == TREMOR) 
            printf("tremor detected, intensity = %f\n", intensity);
        else if (flag == DYSKINESIA) 
            printf("dyskinesia detected, intensity = %f\n", intensity);
    }

    return 0;
}
