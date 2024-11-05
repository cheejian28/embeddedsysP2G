#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>

#define ACCEL_ADDRESS 0x19
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define I2C_PORT i2c0
#define SCL_PIN 5
#define SDA_PIN 4

// Calibration and movement constants
#define CALIBRATION_SAMPLES 200
#define NOISE_THRESHOLD_PERCENTAGE 0.1  // 10% of baseline noise
#define MAX_SPEED 100.0  // Define the maximum speed to calculate percentage

void i2c_init_gy511();
void read_raw_accel_data(int16_t *x, int16_t *y, int16_t *z);
void enhanced_calibrate_accel(int16_t *x_offset, int16_t *y_offset, int16_t *z_offset, int16_t *noise_x, int16_t *noise_y);
void apply_calibration(int16_t *x, int16_t *y, int16_t *z, int16_t x_offset, int16_t y_offset, int16_t z_offset);
float calculate_speed(int16_t accel, int16_t *prev_accel, float *speed, float time_interval);
void generate_command(int16_t x, int16_t y, float speed_x, float speed_y, int16_t noise_x, int16_t noise_y);

void i2c_init_gy511() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(SDA_PIN);

    uint8_t data[2] = {CTRL_REG1_A, 0x27};
    i2c_write_blocking(I2C_PORT, ACCEL_ADDRESS, data, 2, false);

    uint8_t range_data[2] = {CTRL_REG4_A, 0x00};  // ±2g range
    i2c_write_blocking(I2C_PORT, ACCEL_ADDRESS, range_data, 2, false);
}

void read_raw_accel_data(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];
    uint8_t reg = OUT_X_L_A | 0x80;
    i2c_write_blocking(I2C_PORT, ACCEL_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ACCEL_ADDRESS, buffer, 6, false);

    *x = (int16_t)(buffer[1] << 8 | buffer[0]);
    *y = (int16_t)(buffer[3] << 8 | buffer[2]);
    *z = (int16_t)(buffer[5] << 8 | buffer[4]);
}

// Enhanced calibration to account for noise levels and adaptive thresholds
void enhanced_calibrate_accel(int16_t *x_offset, int16_t *y_offset, int16_t *z_offset, int16_t *noise_x, int16_t *noise_y) {
    printf("Enhanced Calibration in Progress...\n");

    int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    int16_t x, y, z;
    int32_t x_variance = 0, y_variance = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        read_raw_accel_data(&x, &y, &z);
        x_sum += x;
        y_sum += y;
        z_sum += z;

        x_variance += x * x;
        y_variance += y * y;

        sleep_ms(10);
    }

    *x_offset = x_sum / CALIBRATION_SAMPLES;
    *y_offset = y_sum / CALIBRATION_SAMPLES;
    *z_offset = (z_sum / CALIBRATION_SAMPLES) - 16384;

    *noise_x = sqrt(x_variance / CALIBRATION_SAMPLES - (*x_offset) * (*x_offset));
    *noise_y = sqrt(y_variance / CALIBRATION_SAMPLES - (*y_offset) * (*y_offset));

    printf("Enhanced Calibration Complete.\n");
    printf("Offsets - X: %d, Y: %d, Z: %d\n", *x_offset, *y_offset, *z_offset);
    printf("Noise Levels - X: %d, Y: %d\n", *noise_x, *noise_y);
}

void apply_calibration(int16_t *x, int16_t *y, int16_t *z, int16_t x_offset, int16_t y_offset, int16_t z_offset) {
    *x -= x_offset;
    *y -= y_offset;
    *z -= z_offset;
}

float calculate_speed(int16_t accel, int16_t *prev_accel, float *speed, float time_interval) {
    float avg_accel = (*prev_accel + accel) / 2.0;
    *speed += avg_accel * time_interval / 1000.0;
    *prev_accel = accel;
    return *speed;
}

// Generates and prints command string based on direction and speed
void generate_command(int16_t x, int16_t y, float speed_x, float speed_y, int16_t noise_x, int16_t noise_y) {
    int16_t left_threshold = -noise_x * (1 + NOISE_THRESHOLD_PERCENTAGE);
    int16_t right_threshold = noise_x * (1 + NOISE_THRESHOLD_PERCENTAGE);
    int16_t forward_threshold = noise_y * (1 + NOISE_THRESHOLD_PERCENTAGE);
    int16_t backward_threshold = -noise_y * (1 + NOISE_THRESHOLD_PERCENTAGE);

    char command[50];
    float speed_percentage;

    if (x > right_threshold) {
        speed_percentage = (speed_x / MAX_SPEED) * 100;
        snprintf(command, sizeof(command), "turn right at %.1f%% speed", speed_percentage);
    } else if (x < left_threshold) {
        speed_percentage = (speed_x / MAX_SPEED) * 100;
        snprintf(command, sizeof(command), "turn left at %.1f%% speed", speed_percentage);
    } else if (y > forward_threshold) {
        speed_percentage = (speed_y / MAX_SPEED) * 100;
        snprintf(command, sizeof(command), "forward at %.1f%% speed", speed_percentage);
    } else if (y < backward_threshold) {
        speed_percentage = (speed_y / MAX_SPEED) * 100;
        snprintf(command, sizeof(command), "backward at %.1f%% speed", speed_percentage);
    } else {
        snprintf(command, sizeof(command), "stationary");
    }

    printf("%s\n", command);
}

int main() {
    stdio_init_all();
    i2c_init_gy511();

    int16_t x, y, z;
    int16_t x_offset, y_offset, z_offset;
    int16_t prev_x = 0, prev_y = 0;
    float speed_x = 0.0, speed_y = 0.0;
    int16_t noise_x, noise_y;
    const float time_interval = 1.0;  // milliseconds

    enhanced_calibrate_accel(&x_offset, &y_offset, &z_offset, &noise_x, &noise_y);

    while (true) {
        read_raw_accel_data(&x, &y, &z);
        apply_calibration(&x, &y, &z, x_offset, y_offset, z_offset);

        float speed_x_val = calculate_speed(x, &prev_x, &speed_x, time_interval);
        float speed_y_val = calculate_speed(y, &prev_y, &speed_y, time_interval);

        generate_command(x, y, speed_x_val, speed_y_val, noise_x, noise_y);

        sleep_ms(1000);
    }

    return 0;
}