/**
 * @file wheel_encoder_dual.c
 * @brief Dual wheel encoder driver for HCMODU0240 IR encoders using Raspberry Pi Pico.
 *
 * This file contains functions to capture the pulse width data from two HCMODU0240 IR wheel encoders
 * connected to a Raspberry Pi Pico. The pulse width data is measured using interrupts to detect
 * the rising and falling edges of the signals for both encoders.
 *
 * @author Hew Zhong Xuan
 * @date 15/10/2024
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
double get_distance_traveled_left();
double calculate_speed_left();
double get_distance_traveled_right();
double calculate_speed_right();

void encoder_callback(uint gpio, uint32_t events)
{
    absolute_time_t now = get_absolute_time();

    if (gpio == ENCODER_PIN_LEFT)
    {
        // Handle left encoder events
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            // Debounce and capture the start time when the signal rises
            if (absolute_time_diff_us(last_rise_time_left, now) > DEBOUNCE_US)
            {
                last_rise_time_left = now; // Record the time of the rising edge
            }
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            // Debounce and capture the pulse width when the signal falls
            if (absolute_time_diff_us(last_fall_time_left, now) > DEBOUNCE_US)
            {
                last_fall_time_left = now;                                                             // Record the time of the falling edge
                pulse_width_us_left = absolute_time_diff_us(last_rise_time_left, last_fall_time_left); // Calculate the pulse width
                new_pulse_data_left = true;                                                            // Set the flag when new data is available
                pulse_count_left++;                                                                    // Increment pulse count
            }
        }
    }
    else if (gpio == ENCODER_PIN_RIGHT)
    {
        // Handle right encoder events
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            // Debounce and capture the start time when the signal rises
            if (absolute_time_diff_us(last_rise_time_right, now) > DEBOUNCE_US)
            {
                last_rise_time_right = now; // Record the time of the rising edge
            }
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            // Debounce and capture the pulse width when the signal falls
            if (absolute_time_diff_us(last_fall_time_right, now) > DEBOUNCE_US)
            {
                last_fall_time_right = now;                                                               // Record the time of the falling edge
                pulse_width_us_right = absolute_time_diff_us(last_rise_time_right, last_fall_time_right); // Calculate the pulse width
                new_pulse_data_right = true;                                                              // Set the flag when new data is available
                pulse_count_right++;                                                                      // Increment pulse count
            }
        }
    }
}

// Initialise wheel encoders
void init_wheel_encoder()
{
    // Left encoder initialization
    gpio_init(ENCODER_PIN_LEFT);
    gpio_set_dir(ENCODER_PIN_LEFT, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_LEFT);

    // Enable interrupts on both rising and falling edges for left encoder
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_LEFT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);

    // Right encoder initialization
    gpio_init(ENCODER_PIN_RIGHT);
    gpio_set_dir(ENCODER_PIN_RIGHT, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_RIGHT);

    // Enable interrupts on both rising and falling edges for right encoder
    gpio_set_irq_enabled(ENCODER_PIN_RIGHT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Function to initialize wheel parameters
void init_wheel_parameters()
{
    wheel_circumference = M_PI * WHEEL_DIAMETER_CM;
    distance_per_pulse = wheel_circumference / PPR;
}

// Functions for the left encoder
double get_distance_traveled_left()
{
    return pulse_count_left * distance_per_pulse;
}

double calculate_speed_left()
{
    if (pulse_width_us_left > 0 && pulse_width_us_left < MAX_PULSE_WIDTH_US)
    {
        double time_interval_s = pulse_width_us_left / 1000000.0; // Convert pulse width from microseconds to seconds
        double speed = distance_per_pulse / time_interval_s;      // Calculate speed (distance per pulse / time interval)
        return speed;
    }
    return 0.0;
}

// Functions for the right encoder
double get_distance_traveled_right()
{
    return pulse_count_right * distance_per_pulse;
}

double calculate_speed_right()
{
    if (pulse_width_us_right > 0 && pulse_width_us_right < MAX_PULSE_WIDTH_US)
    {
        double time_interval_s = pulse_width_us_right / 1000000.0; // Convert pulse width from microseconds to seconds
        double speed = distance_per_pulse / time_interval_s;       // Calculate speed (distance per pulse / time interval)
        return speed;
    }
    return 0.0;
}

int main()
{
    // Initialize standard I/O and the wheel encoders
    stdio_init_all();
    init_wheel_parameters();
    init_wheel_encoder();

    while (true)
    {
        double distance_left = get_distance_traveled_left();
        double speed_left = calculate_speed_left();

        double distance_right = get_distance_traveled_right();
        double speed_right = calculate_speed_right();

        if (new_pulse_data_left)
        {
            printf("Left Motor - Distance Traveled: %.2f cm, Speed: %.2f cm/s\n", distance_left, speed_left);
            new_pulse_data_left = false; // Reset the flag
        }

        if (new_pulse_data_right)
        {
            printf("Right Motor - Distance Traveled: %.2f cm, Speed: %.2f cm/s\n", distance_right, speed_right);
            new_pulse_data_right = false; // Reset the flag
        }

        if (!new_pulse_data_left && !new_pulse_data_right)
        {
            printf("No valid pulse detected\n");
        }

        sleep_ms(500);
    }

    return 0;
}
