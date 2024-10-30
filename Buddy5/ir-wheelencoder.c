/**
 * @file wheel_encoder_dual.c
 * @brief Dual wheel encoder driver for HCMODU0240 IR encoders using Raspberry Pi Pico.
 *
 * This file contains functions to capture the pulse width data from two HCMODU0240 IR wheel encoders
 * connected to a Raspberry Pi Pico. The pulse width data is measured using interrupts to detect
 * the rising and falling edges of the signals for both encoders.
 *
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <math.h>

#define ENCODER_PIN_LEFT 1         ///< GPIO pin number for the left encoder output pin
#define ENCODER_PIN_RIGHT 2        ///< GPIO pin number for the right encoder output pin
#define DEBOUNCE_US 100            ///< Debounce time in microseconds to avoid noise
#define MAX_PULSE_WIDTH_US 1000000 ///< Maximum pulse width to detect (1 second in microseconds)
#define WHEEL_DIAMETER_CM 10.0     ///< Wheel diameter in centimeters
#define PPR 8                      ///< Pulses per revolution of the encoder

// Struct to store speed information
typedef struct
{
    double left_wheel_speed;
    double right_wheel_speed;
} WheelSpeeds;

// Variables for the left encoder
volatile int64_t pulse_width_us_left = 0;  ///< Width of the pulse in microseconds for left encoder.
volatile int pulse_count_left = 0;         ///< Counter for pulses on the left encoder
volatile bool new_pulse_data_left = false; ///< Flag to indicate new pulse width data for left encoder
absolute_time_t last_rise_time_left;
absolute_time_t last_fall_time_left;

// Variables for the right encoder
volatile int64_t pulse_width_us_right = 0;  ///< Width of the pulse in microseconds for right encoder.
volatile int pulse_count_right = 0;         ///< Counter for pulses on the right encoder
volatile bool new_pulse_data_right = false; ///< Flag to indicate new pulse width data for right encoder
absolute_time_t last_rise_time_right;
absolute_time_t last_fall_time_right;

double wheel_circumference; ///< Circumference of the wheel
double distance_per_pulse;  ///< Distance traveled per pulse

// Function prototypes
void encoder_callback(uint gpio, uint32_t events);
void init_wheel_encoder();
void init_wheel_parameters();
double calculate_speed_left();
double calculate_speed_right();
WheelSpeeds get_speeds();

void encoder_callback(uint gpio, uint32_t events)
{
    absolute_time_t now = get_absolute_time();

    if (gpio == ENCODER_PIN_LEFT)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            if (absolute_time_diff_us(last_rise_time_left, now) > DEBOUNCE_US)
            {
                last_rise_time_left = now;
            }
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            if (absolute_time_diff_us(last_fall_time_left, now) > DEBOUNCE_US)
            {
                last_fall_time_left = now;
                pulse_width_us_left = absolute_time_diff_us(last_rise_time_left, last_fall_time_left);
                new_pulse_data_left = true;
                pulse_count_left++;
            }
        }
    }
    else if (gpio == ENCODER_PIN_RIGHT)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            if (absolute_time_diff_us(last_rise_time_right, now) > DEBOUNCE_US)
            {
                last_rise_time_right = now;
            }
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            if (absolute_time_diff_us(last_fall_time_right, now) > DEBOUNCE_US)
            {
                last_fall_time_right = now;
                pulse_width_us_right = absolute_time_diff_us(last_rise_time_right, last_fall_time_right);
                new_pulse_data_right = true;
                pulse_count_right++;
            }
        }
    }
}

void init_wheel_encoder()
{
    gpio_init(ENCODER_PIN_LEFT);
    gpio_set_dir(ENCODER_PIN_LEFT, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_LEFT);

    gpio_set_irq_enabled_with_callback(ENCODER_PIN_LEFT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);

    gpio_init(ENCODER_PIN_RIGHT);
    gpio_set_dir(ENCODER_PIN_RIGHT, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_RIGHT);

    gpio_set_irq_enabled(ENCODER_PIN_RIGHT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void init_wheel_parameters()
{
    wheel_circumference = M_PI * WHEEL_DIAMETER_CM;
    distance_per_pulse = wheel_circumference / PPR;
}

double calculate_speed_left()
{
    if (pulse_width_us_left > 0 && pulse_width_us_left < MAX_PULSE_WIDTH_US)
    {
        double time_interval_s = pulse_width_us_left / 1000000.0;
        double speed = distance_per_pulse / time_interval_s;
        return speed;
    }
    return 0.0;
}

double calculate_speed_right()
{
    if (pulse_width_us_right > 0 && pulse_width_us_right < MAX_PULSE_WIDTH_US)
    {
        double time_interval_s = pulse_width_us_right / 1000000.0;
        double speed = distance_per_pulse / time_interval_s;
        return speed;
    }
    return 0.0;
}

// Function to get the speeds of both wheels
WheelSpeeds get_speeds()
{
    WheelSpeeds speeds;
    speeds.left_wheel_speed = calculate_speed_left();
    speeds.right_wheel_speed = calculate_speed_right();
    return speeds;
}

int main()
{
    stdio_init_all();
    init_wheel_parameters();
    init_wheel_encoder();

    while (true)
    {
        WheelSpeeds speeds = get_speeds();

        if (new_pulse_data_left || new_pulse_data_right)
        {
            printf("Left Wheel Speed: %.2f cm/s, Right Wheel Speed: %.2f cm/s\n", speeds.left_wheel_speed, speeds.right_wheel_speed);
            new_pulse_data_left = false;
            new_pulse_data_right = false;
        }
        else
        {
            printf("No valid pulse detected\n");
        }

        sleep_ms(500);
    }

    return 0;
}
