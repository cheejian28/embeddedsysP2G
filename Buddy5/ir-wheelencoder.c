#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define ENCODER_LEFT 6
#define ENCODER_RIGHT 7

#define ENCODER_CIRCUMFERENCE 6.28f                                            // cm
#define WHEEL_CIRCUMFERENCE 21.0f                                              // cm
#define EDGES_PER_ROTATION 20                                                  // Number of edges (slits) in the encoder wheel
#define DISTANCE_PER_EDGE_ENCODER (ENCODER_CIRCUMFERENCE / EDGES_PER_ROTATION) // cm per edge
#define WHEEL_TO_ENCODER_RATIO (WHEEL_CIRCUMFERENCE / ENCODER_CIRCUMFERENCE)   // Ratio

static int num_edge_l;
static int num_edge_r;
static float pulse_width_l;
static float pulse_width_r;
static float t_distance_travelled;
struct repeating_timer timer;

void gpio_callback(uint gpio, uint32_t events)
{
    static uint32_t edge_fall_time_l;
    static uint32_t edge_fall_time_r;

    uint32_t current_time = time_us_32();

    if (gpio == ENCODER_LEFT)
    {                                                                          // Left wheel
        pulse_width_l = (float)(current_time - edge_fall_time_l) / 1000000.0f; // Pulse width in seconds
        num_edge_l++;
        edge_fall_time_l = current_time; // Reset timer
    }
    else if (gpio == ENCODER_RIGHT)
    {                                                                          // Right wheel
        pulse_width_r = (float)(current_time - edge_fall_time_r) / 1000000.0f; // Pulse width in seconds
        num_edge_r++;
        edge_fall_time_r = current_time;
    }
}

bool print_out(struct repeating_timer *t)
{
    float speed_per_sec_l = 0;  // Measured in cm/s
    float speed_per_sec_r = 0;  // Measured in cm/s
    float distance_per_sec = 0; // Measured in cm

    // Calculate the average number of edges detected
    float avg_num_edges = ((num_edge_l + num_edge_r) / 2.0f);

    // Calculate encoder wheel distance
    float encoder_distance = avg_num_edges * DISTANCE_PER_EDGE_ENCODER;

    // Adjust to actual wheel distance
    distance_per_sec = encoder_distance * WHEEL_TO_ENCODER_RATIO;

    // Update total distance traveled
    t_distance_travelled += distance_per_sec;

    // Calculate speed for left and right wheels
    speed_per_sec_l = (pulse_width_l > 0) ? (DISTANCE_PER_EDGE_ENCODER / pulse_width_l) * WHEEL_TO_ENCODER_RATIO : 0;
    speed_per_sec_r = (pulse_width_r > 0) ? (DISTANCE_PER_EDGE_ENCODER / pulse_width_r) * WHEEL_TO_ENCODER_RATIO : 0;

    // Print results
    printf("Total distance: %.2f cm\n", t_distance_travelled);
    printf("Distance per second: %.2f cm/s\n", distance_per_sec);
    printf("Speed using left pulse width: %.2f cm/s\n", speed_per_sec_l);
    printf("Speed using right pulse width: %.2f cm/s\n\n", speed_per_sec_r);

    // Reset edge counts
    num_edge_l = 0;
    num_edge_r = 0;
    return true;
}

int main()
{
    stdio_init_all();

    // Setup pins
    printf("Wheel Encoder Measurement\n");
    gpio_set_function(ENCODER_LEFT, GPIO_IN);
    gpio_set_function(ENCODER_RIGHT, GPIO_IN);

    // Configure GPIO pins with interrupts
    gpio_set_irq_enabled_with_callback(ENCODER_LEFT, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(ENCODER_RIGHT, GPIO_IRQ_EDGE_FALL, true);

    // Timer for periodic output (every 1000ms)
    add_repeating_timer_ms(-1000, print_out, NULL, &timer);

    // Main loop
    while (1)
    {
        tight_loop_contents();
    }
}
