#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <wifi_communication.h>
#include <udp_client_communication.h>


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

#define CALIBRATION_SAMPLES 200
#define NOISE_THRESHOLD_PERCENTAGE 0.1
#define MAX_SPEED 100.0
#define SPEED_SCALING_FACTOR 500.0
#define HIGH_PASS_THRESHOLD 10
#define STATIONARY_THRESHOLD 0.05

void i2c_init_gy511();
void read_raw_accel_data(int16_t *x, int16_t *y, int16_t *z);
void enhanced_calibrate_accel(int16_t *x_offset, int16_t *y_offset, int16_t *z_offset, int16_t *noise_x, int16_t *noise_y);
void apply_calibration(int16_t *x, int16_t *y, int16_t *z, int16_t x_offset, int16_t y_offset, int16_t z_offset);
void generate_command(int16_t x, int16_t y, float *speed_x, float *speed_y, int16_t noise_x, int16_t noise_y, char *last_command);
float calculate_speed(int16_t accel, int16_t *prev_accel, float *speed, float time_interval);

void i2c_init_gy511() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(SDA_PIN);
    uint8_t data[2] = {CTRL_REG1_A, 0x27};
    i2c_write_blocking(I2C_PORT, ACCEL_ADDRESS, data, 2, false);
    uint8_t range_data[2] = {CTRL_REG4_A, 0x00};
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

void enhanced_calibrate_accel(int16_t *x_offset, int16_t *y_offset, int16_t *z_offset, int16_t *noise_x, int16_t *noise_y) {
    int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    int16_t x, y, z;
    int16_t x_max = INT16_MIN, y_max = INT16_MIN;
    int16_t x_min = INT16_MAX, y_min = INT16_MAX;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        read_raw_accel_data(&x, &y, &z);
        x_sum += x;
        y_sum += y;
        z_sum += z;
        if (x > x_max) x_max = x;
        if (y > y_max) y_max = y;
        if (x < x_min) x_min = x;
        if (y < y_min) y_min = y;
        sleep_ms(10);
    }

    *x_offset = x_sum / CALIBRATION_SAMPLES;
    *y_offset = y_sum / CALIBRATION_SAMPLES;
    *z_offset = z_sum / CALIBRATION_SAMPLES;

    *noise_x = (x_max - x_min) * NOISE_THRESHOLD_PERCENTAGE;
    *noise_y = (y_max - y_min) * NOISE_THRESHOLD_PERCENTAGE;
}

void apply_calibration(int16_t *x, int16_t *y, int16_t *z, int16_t x_offset, int16_t y_offset, int16_t z_offset) {
    *x -= x_offset;
    *y -= y_offset;
    *z -= z_offset;
}

void generate_command(int16_t x, int16_t y, float *speed_x, float *speed_y, int16_t noise_x, int16_t noise_y, char *last_command) {
    float speed_x_abs = fabs(*speed_x);
    float speed_y_abs = fabs(*speed_y);
    char command[50];

    if (speed_x_abs < STATIONARY_THRESHOLD && speed_y_abs < STATIONARY_THRESHOLD) {
        snprintf(command, sizeof(command), "s");
        *speed_x = 0.0;
        *speed_y = 0.0;
    } else {
        if (speed_x_abs > speed_y_abs) {
            if (*speed_x > 0) {
                // snprintf(command, sizeof(command), "forward at %.1f%% speed", (*speed_x / MAX_SPEED) * 100);
                snprintf(command, sizeof(command), "f %.1f%%", (*speed_x / MAX_SPEED) * 100);
            } else {
                // snprintf(command, sizeof(command), "backward at %.1f%% speed", (-*speed_x / MAX_SPEED) * 100);
                snprintf(command, sizeof(command), "b %.1f%%", (-*speed_x / MAX_SPEED) * 100);
            }
        } else {
            if (*speed_y > 0) {
                // snprintf(command, sizeof(command), "turn right at %.1f%% speed", (*speed_y / MAX_SPEED) * 100);
                snprintf(command, sizeof(command), "l %.1f%%", (*speed_y / MAX_SPEED) * 100);
            } else {
                // snprintf(command, sizeof(command), "turn left at %.1f%% speed", (-*speed_y / MAX_SPEED) * 100);
                snprintf(command, sizeof(command), "r %.1f%%", (-*speed_y / MAX_SPEED) * 100);
            }
        }
    }

    // Only print command if it differs from the last printed command
    if (strcmp(command, last_command) != 0) {
        printf("Command: %s\n", command);
        // tcp_send_data(command);
        strncpy(last_command, command, sizeof(command)); 
        send_udp_data(command);
    }
}

float calculate_speed(int16_t accel, int16_t *prev_accel, float *speed, float time_interval) {
    float accel_diff = accel - *prev_accel;
    if (fabs(accel_diff) > HIGH_PASS_THRESHOLD) { 
        *speed += (accel_diff / SPEED_SCALING_FACTOR) * time_interval;
    }
    *prev_accel = accel;
    return *speed;
}

int main() {
    stdio_init_all();
    i2c_init_gy511(); 

    set_ssid_password("SimPhone", "a1234567");
    // set_ssid_password("Galaxy S10edc70", "xmhq2715");
    checkWifiConnection();

    // set_ip_address("172.20.10.2");
    // set_ip_address("192.168.252.184");
    // checkClientConnection();
    init_udp_client("172.20.10.7", 42069);

    // init_wifi_with_ssid_password("SimPhone","a1234567");
    // init_tcp_client_with_ip("172.20.10.2");

    int16_t x, y, z;
    int16_t x_offset = 0, y_offset = 0, z_offset = 0;
    int16_t noise_x = 0, noise_y = 0;
    float speed_x = 0.0, speed_y = 0.0;
    int16_t prev_x = 0, prev_y = 0;
    char last_command[50] = "";  // Stores the last command for comparison

    enhanced_calibrate_accel(&x_offset, &y_offset, &z_offset, &noise_x, &noise_y);

    while (true) {
        read_raw_accel_data(&x, &y, &z);
        apply_calibration(&x, &y, &z, x_offset, y_offset, z_offset);

        float time_interval = 0.1;
        calculate_speed(x, &prev_x, &speed_x, time_interval);
        calculate_speed(y, &prev_y, &speed_y, time_interval);

        // printf("Current X: %d, Y: %d, Z: %d, Speed X: %.2f, Speed Y: %.2f\n", x, y, z, speed_x, speed_y);
        generate_command(x, y, &speed_x, &speed_y, noise_x, noise_y, last_command);

        // checkWifiConnection();
        // checkClientConnection();

        sleep_ms(25);
    }

    return 0;
}