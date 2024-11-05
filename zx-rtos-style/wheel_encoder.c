#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "ultrasonic.h"

#define ENCODER_LEFT 7
#define ENCODER_RIGHT 8
#define PPR 20
#define WHEEL_DIAMETER 6.5f
#define WHEEL_CIRCUMFERENCE 21.8f

volatile int total_num_edge_l = 0;
volatile int total_num_edge_r = 0;
static volatile float pulse_width_l = 0.0f;
static volatile float pulse_width_r = 0.0f;
volatile absolute_time_t last_rise_time_l;
volatile absolute_time_t last_fall_time_l;
volatile absolute_time_t last_rise_time_r;
volatile absolute_time_t last_fall_time_r;

volatile float speedLeftEncoder = 0.0f;
volatile float speedRightEncoder = 0.0f;

volatile float cumulative_distance_left = 0.0f;
volatile float cumulative_distance_right = 0.0f;
volatile absolute_time_t last_distance_update_left;
volatile absolute_time_t last_distance_update_right;

// Optional: Define constants for distance calculations
#define DISTANCE_PER_PULSE 0.314f // in cm

// Encoder callback function
void gpio_callback(uint gpio, uint32_t events)
{

    absolute_time_t current_time = get_absolute_time();

    if (gpio == ENCODER_LEFT)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            last_rise_time_l = current_time;
            total_num_edge_l++;

            if (!is_nil_time(last_fall_time_l))
            {
                float time_interval = (float)absolute_time_diff_us(last_fall_time_l, last_rise_time_l) / 1000000.0;
                if (time_interval > 0.01f)
                {
                    speedLeftEncoder = (WHEEL_CIRCUMFERENCE / PPR) / time_interval;
                    // t_distance_travelled += DISTANCE_PER_PULSE;
                    // printf("Speed: %.2f cm/s\n", speed);
                    // printf("Distance Travelled: %.2f cm\n", t_distance_travelled);
                }
            }
            last_fall_time_l = last_rise_time_l;

            float time_since_last_update = (float)absolute_time_diff_us(last_distance_update_left, current_time) / 1000000.0;
            cumulative_distance_left += speedLeftEncoder * time_since_last_update;
            last_distance_update_left = current_time;
        }
        // Print on every left pulse
        // printf("Left Encoder Pulse Detected. Total Left Pulses: %d\n", total_num_edge_l);
    }
    else if (gpio == ENCODER_RIGHT)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            last_rise_time_r = current_time; // Update the last rise time for the right encoder                    // Increment edge count for the right encoder
            total_num_edge_r++;              // Increment total edge count for the right encoder

            // Calculate instantaneous speed for the right encoder
            if (!is_nil_time(last_fall_time_r))
            {
                float time_interval = (float)absolute_time_diff_us(last_fall_time_r, last_rise_time_r) / 1000000.0;
                if (time_interval > 0.0f)
                { // Safeguard against zero time interval
                    speedRightEncoder = (WHEEL_CIRCUMFERENCE / PPR) / time_interval;
                }
                else
                {
                    speedRightEncoder = 0.0f; // Handle zero or invalid time interval
                }
            }

            last_fall_time_r = last_rise_time_r; // Update the last fall time for the right encoder

            // Update cumulative distance for the right wheel
            float time_since_last_update = (float)absolute_time_diff_us(last_distance_update_right, current_time) / 1000000.0;
            cumulative_distance_right += speedRightEncoder * time_since_last_update;
            last_distance_update_right = current_time; // Update the last distance update time for the right encoder
        }

        // Optional: Print on every right pulse for debugging
        // printf("Right Encoder Pulse Detected. Total Right Pulses: %d\n", total_num_edge_r);
    }
    else if (gpio == ECHO_PIN)
    {
        computePulse(events);
    }
}

void setup_encoder()
{
    gpio_init(ENCODER_LEFT);
    gpio_set_dir(ENCODER_LEFT, GPIO_IN);
    gpio_pull_up(ENCODER_LEFT);

    gpio_init(ENCODER_RIGHT);
    gpio_set_dir(ENCODER_RIGHT, GPIO_IN);
    gpio_pull_up(ENCODER_RIGHT);

    gpio_set_irq_enabled_with_callback(ENCODER_LEFT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(ENCODER_RIGHT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}
