/**
 * @file wheel_encoder.c
 * @brief Wheel encoder driver for HCMODU0240 IR encoder using Raspberry Pi Pico.
 *
 * This file contains functions to capture the pulse width data from an HCMODU0240 IR wheel encoder
 * connected to a Raspberry Pi Pico. The pulse width data is measured using interrupts to detect
 * the rising and falling edges of the signal.
 *
 * @author Hew Zhong Xuan
 * @date 15/10/2024
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <math.h>

#define ENCODER_PIN 1              ///< GPIO pin number for the encoder output pin
#define DEBOUNCE_US 100            ///< Debounce time in microseconds to avoid noise
#define MAX_PULSE_WIDTH_US 1000000 ///< Maximum pulse width to detect (1 second in microseconds)
#define WHEEL_DIAMETER_CM 10.0     ///< Wheel diameter in centimeters
#define PPR 8                      ///< Pulses per revolution of the encoder

volatile int64_t pulse_width_us = 0;  ///< Width of the pulse in microseconds.
volatile int pulse_count = 0;         ///< Counter for pulses
volatile bool new_pulse_data = false; // Flag to indicate new pulse width data

absolute_time_t last_rise_time;
absolute_time_t last_fall_time;

double wheel_circumference; ///< Circumference of the wheel
double distance_per_pulse;  ///< Distance traveled per pulse

// Function prototypes
void encoder_callback(uint gpio, uint32_t events);
void init_wheel_encoder();
double get_distance_traveled();
double calculate_speed();

void encoder_callback(uint gpio, uint32_t events)
{
    absolute_time_t now = get_absolute_time();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        // Debounce and capture the start time when the signal rises
        if (absolute_time_diff_us(last_rise_time, now) > DEBOUNCE_US)
        {
            last_rise_time = now; // Record the time of the rising edge
        }
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        // Debounce and capture the pulse width when the signal falls
        if (absolute_time_diff_us(last_fall_time, now) > DEBOUNCE_US)
        {
            last_fall_time = now;                                                   // Record the time of the falling edge
            pulse_width_us = absolute_time_diff_us(last_rise_time, last_fall_time); // Calculate the pulse width
            new_pulse_data = true;                                                  // Set the flag when new data is available
            pulse_count++;                                                          // Increment pulse count
        }
    }
}

// Initialise wheel encoder
void init_wheel_encoder()
{
    gpio_init(ENCODER_PIN);
    gpio_set_dir(ENCODER_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_PIN);

    // Enable interrupts on both rising and falling edges
    gpio_set_irq_enabled_with_callback(ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
}

// New function to initialize wheel parameters
void init_wheel_parameters()
{
    wheel_circumference = M_PI * WHEEL_DIAMETER_CM;
    distance_per_pulse = wheel_circumference / PPR;
}

// Function to calculate total distance traveled
double get_distance_traveled()
{
    return pulse_count * distance_per_pulse;
}

// Function to calculate speed based on pulse width
double calculate_speed()
{
    if (pulse_width_us > 0 && pulse_width_us < MAX_PULSE_WIDTH_US)
    {
        double time_interval_s = pulse_width_us / 1000000.0; // Convert pulse width from microseconds to seconds
        double speed = distance_per_pulse / time_interval_s; // Calculate speed (distance per pulse / time interval)
        return speed;
    }
    return 0.0;
}

int main()
{
    // Initialize standard I/O and the wheel encoder
    stdio_init_all();
    init_wheel_parameters();
    init_wheel_encoder();

    while (true)
    {
        double distance = get_distance_traveled();
        double speed = calculate_speed();

        if (new_pulse_data)
        {
            printf("Distance Traveled: %.2f cm, Speed: %.2f cm/s\n", distance, speed);
            new_pulse_data = false; // Reset the flag
        }
        else
        {
            printf("No valid pulse detected\n");
        }

        sleep_ms(500);
    }

    return 0;
}
