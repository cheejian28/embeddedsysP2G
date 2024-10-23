#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

// LSM303DLHC I2C address for accelerometer
#define ACCEL_ADDRESS 0x19

// Accelerometer registers
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23

// I2C instance and pins
#define I2C_PORT i2c0
#define SCL_PIN 5
#define SDA_PIN 4

// Function declarations
void i2c_init_gy511();
void read_raw_accel_data(int16_t *x, int16_t *y, int16_t *z);
void calibrate_accel(int16_t *x_offset, int16_t *y_offset, int16_t *z_offset);
void apply_calibration(int16_t *x, int16_t *y, int16_t *z, int16_t x_offset, int16_t y_offset, int16_t z_offset);

// Initialize the I2C for communication
void i2c_init_gy511() {
    i2c_init(I2C_PORT, 400 * 1000); // Initialize I2C at 400kHz
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(SDA_PIN);

    // Initialize accelerometer (CTRL_REG1_A) - Enable, 10Hz data rate
    uint8_t data[2] = {CTRL_REG1_A, 0x27}; // 0x27 -> Enable accelerometer, 10Hz data rate
    i2c_write_blocking(I2C_PORT, ACCEL_ADDRESS, data, 2, false);

    // Set accelerometer range to ±2g (CTRL_REG4_A)
    uint8_t range_data[2] = {CTRL_REG4_A, 0x00};  // ±2g
    i2c_write_blocking(I2C_PORT, ACCEL_ADDRESS, range_data, 2, false);
}

// Function to read raw accelerometer data from the GY-511
void read_raw_accel_data(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];

    // Set the register address with auto-increment for multi-byte read
    uint8_t reg = OUT_X_L_A | 0x80;

    // Read 6 bytes from the accelerometer's X, Y, Z registers
    i2c_write_blocking(I2C_PORT, ACCEL_ADDRESS, &reg, 1, true);   // Send the register address
    i2c_read_blocking(I2C_PORT, ACCEL_ADDRESS, buffer, 6, false); // Read 6 bytes of data

    // Combine high and low bytes into 16-bit signed values
    *x = (int16_t)(buffer[1] << 8 | buffer[0]);
    *y = (int16_t)(buffer[3] << 8 | buffer[2]);
    *z = (int16_t)(buffer[5] << 8 | buffer[4]);
}

// Calibration function to calculate offsets
void calibrate_accel(int16_t *x_offset, int16_t *y_offset, int16_t *z_offset) {
    printf("Calibrating... Keep the sensor flat and still.\n");
    int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    int16_t x, y, z;
    int num_samples = 100;

    for (int i = 0; i < num_samples; i++) {
        read_raw_accel_data(&x, &y, &z);
        x_sum += x;
        y_sum += y;
        z_sum += z;
        sleep_ms(50);  // Short delay between samples
    }

    // Calculate the average offset for each axis
    *x_offset = x_sum / num_samples;
    *y_offset = y_sum / num_samples;
    *z_offset = (z_sum / num_samples) - 16384;  // Subtract gravity (~1g) on Z axis

    printf("Calibration complete.\n");
    printf("Offsets - X: %d, Y: %d, Z: %d\n", *x_offset, *y_offset, *z_offset);
}

// Apply the calibration offsets to the raw data
void apply_calibration(int16_t *x, int16_t *y, int16_t *z, int16_t x_offset, int16_t y_offset, int16_t z_offset) {
    *x -= x_offset;
    *y -= y_offset;
    *z -= z_offset;
}

int main() {
    // Initialize stdio for USB serial communication
    stdio_init_all();

    // Initialize the I2C and GY-511 sensor
    i2c_init_gy511();

    // Variables to store accelerometer data and calibration offsets
    int16_t x, y, z;
    int16_t x_offset, y_offset, z_offset;

    // Perform calibration
    calibrate_accel(&x_offset, &y_offset, &z_offset);

    // Main loop
    while (true) {
        // Read raw accelerometer data
        read_raw_accel_data(&x, &y, &z);

        // Apply calibration offsets
        apply_calibration(&x, &y, &z, x_offset, y_offset, z_offset);

        // Print the calibrated accelerometer data
        printf("Calibrated Accelerometer Data - X: %d, Y: %d, Z: %d\n", x, y, z);

        // Delay before the next reading
        sleep_ms(1000);
    }

    return 0;
}
