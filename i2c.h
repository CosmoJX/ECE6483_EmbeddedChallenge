/*
    header file for I2C communication and IMU access functions
*/

#include <mbed.h>
#include "arm_math.h"

// LSM6DSL address (0x6A in datasheet, shifted left for 8-bit format)
#define LSM6DSL_ADDR (0x6A << 1)  // Equals 0xD4

// Please Refer to 48 and 49th pages in LSM6DSL datasheet 
#define WHO_AM_I    0x0F  // ID register - should return 0x6A
#define CTRL1_XL    0x10  // Accelerometer control register to configure range
#define CTRL2_G     0x11  // Gyroscope control register to configure range
#define OUTX_L_XL   0x28  // XL X-axis (low byte)
#define OUTX_H_XL   0x29  // XL X-axis (high byte)
#define OUTY_L_XL   0x2A  // XL Y-axis (low byte)
#define OUTY_H_XL   0x2B  // XL Y-axis (high byte)
#define OUTZ_L_XL   0x2C  // XL Z-axis (low byte)
#define OUTZ_H_XL   0x2D  // XL Z-axis (high byte)
#define OUTX_L_G    0x22  // Gyro X-axis (low byte)
#define OUTX_H_G    0x23  // Gyro X-axis (high byte)
#define OUTY_L_G    0x24  // Gyro Y-axis (low byte)
#define OUTY_H_G    0x25  // Gyro Y-axis (high byte)
#define OUTZ_L_G    0x26  // Gyro Z-axis (low byte)
#define OUTZ_H_G    0x27  // Gyro Z-axis (high byte)
#define STATUS_REG  0x1E  // New Data Ready Status

#define ACC_SENSITIVITY 0.061

/*
    Low-Pass filter for DSP of accelerometer data
*/
class LowPassFilter {
protected:
    float32_t y;
    const float32_t alpha;
public:
    LowPassFilter(float32_t y, const float32_t a) : y(y), alpha(a) {}

    float32_t filter(const float32_t x) {
        y = x + alpha * (y - x);
        return y;
    }

    void reset() {y = 0;}
};

// write 1 byte value into reg
void write_register(uint8_t reg, uint8_t value);

// read 1 byte value from reg
uint8_t read_register(uint8_t reg);

// read 2 bytes of value from reg
int16_t read_16bit_value(uint8_t low_reg, uint8_t high_reg);

// setup i2c connection and IMU config
void i2c_init(void);

// read one set of accelerometer data from IMU
void read_acceleration_sample(float32_t *acc_data);

/*
    read FFT_SIZE number of acceleration samples from IMU
    fill the data_array for FFT operation
*/
void read_acceleration(void);