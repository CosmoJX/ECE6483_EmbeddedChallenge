#include "i2c.h"
#include "arm_math.h"
#include "fft.h"

#define FILTER_ALPHA 0.2f

extern I2C i2c;
extern float32_t data_array[FFT_SIZE];

// Write a value to a register
void write_register(uint8_t reg, uint8_t value) {
    char data[2] = {(char)reg, (char)value};
    i2c.write(LSM6DSL_ADDR, data, 2);
}

// Read a value from a register
uint8_t read_register(uint8_t reg) {
    char data = reg;
    i2c.write(LSM6DSL_ADDR, &data, 1, true); // No stop
    i2c.read(LSM6DSL_ADDR, &data, 1);
    return (uint8_t)data;
}

// Read a 16-bit value (combines low and high byte registers)
int16_t read_16bit_value(uint8_t low_reg, uint8_t high_reg) {
    // Read low byte
    char low_byte = read_register(low_reg);
    // Read high byte
    char high_byte = read_register(high_reg);
    // Combine the bytes (little-endian: low byte first)
    return (high_byte << 8) | low_byte;
}


// setup i2c connection and IMU config
void i2c_init(void) {
    // setup I2C frequency
    i2c.frequency(400000);  // 400kHz

    // Check if sensor is connected
    uint8_t id = read_register(WHO_AM_I);
    if (id != 0x6A) {
        printf("i2c not connected\n");
        while(1);   // pause
    }

    // Configure the accelerometer (104 Hz, ±2g range)
    write_register(CTRL1_XL, 0x40);
    // printf("Accelerometer configured: 104 Hz, ±2g range\r\n");
}


// read one set of accelerometer data from IMU
void read_acceleration_sample(float32_t *acc_data) {
    // spin until new data is ready
    while (!(read_register(STATUS_REG) & 0x1)); 

    // read x, y, z acceleration
    int16_t acc_x_raw = read_16bit_value(OUTX_L_XL, OUTX_H_XL);
    int16_t acc_y_raw = read_16bit_value(OUTY_L_XL, OUTY_H_XL);
    int16_t acc_z_raw = read_16bit_value(OUTZ_L_XL, OUTZ_H_XL);

    acc_data[0] = (float32_t)acc_x_raw * ACC_SENSITIVITY / 1000.0f;
    acc_data[1] = (float32_t)acc_y_raw * ACC_SENSITIVITY / 1000.0f;
    acc_data[2] = (float32_t)acc_z_raw * ACC_SENSITIVITY / 1000.0f;
}


/*
    read FFT_SIZE number of acceleration samples from IMU
    fill the data_array for FFT operation
*/
void read_acceleration(void) {
    float32_t sum = 0, mean = 0;
    float32_t data_sample[3];   // x, y, z
    // read one sample first
    read_acceleration_sample(data_sample);
    // initialize low-pass filter
    LowPassFilter filter_x(data_sample[0], FILTER_ALPHA);
    LowPassFilter filter_y(data_sample[1], FILTER_ALPHA);
    LowPassFilter filter_z(data_sample[2], FILTER_ALPHA);
    // fill data_array
    // calculate the size of acceleration vector
    data_array[0] = sqrt(pow(data_sample[0], 2) + pow(data_sample[1], 2) + pow(data_sample[2], 2));
    sum += data_array[0];

    // continue reading data
    for (int i=1; i<FFT_SIZE; ++i) {
        read_acceleration_sample(data_sample);
        data_array[i] = sqrt(
                                pow(filter_x.filter(data_sample[0]), 2) + 
                                pow(filter_y.filter(data_sample[1]), 2) + 
                                pow(filter_z.filter(data_sample[2]), 2)
                            );
        sum += data_array[i];
    }
    
    // subtract mean to remove DC offset
    mean = sum/FFT_SIZE;
    for (int i=0; i<FFT_SIZE; ++i) data_array[i] -= mean;
}
