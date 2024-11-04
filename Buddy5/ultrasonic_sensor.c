#include <stdio.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

#define TRIGGER_PIN 4
#define ECHO_PIN 5
#define TIMEOUT_US 26100
#define EMA_ALPHA 0.2 // Smoothing factor for EMA (0 < alpha <= 1)

absolute_time_t startTime;
absolute_time_t endTime;

float ultrasonic_ema = 0; // Variable to hold the EMA of the distance

// Function to send a 10us initiation pulse
void sendInitiationPulse(uint trigPin)
{
    gpio_put(trigPin, 1);
    busy_wait_us(10);
    gpio_put(trigPin, 0);
}

// Function to calculate and display the EMA-smoothed distance
void getUltrasonicDetection(uint64_t pulseLength)
{
    // Use precise speed of sound in cm/us
    float speed_of_sound_cm_per_us = 0.0343f;

    // Convert pulse length to centimeters
    float ultrasonic_in_cm = (pulseLength * speed_of_sound_cm_per_us) / 2.0f;

    // Apply EMA formula
    ultrasonic_ema = (EMA_ALPHA * ultrasonic_in_cm) + ((1 - EMA_ALPHA) * ultrasonic_ema);

    // Display the smoothed distance with higher precision
    printf("Moving Distance (EMA): %.3f cm\n\n", ultrasonic_ema);
}

// Interrupt handler for echo pin
void handle_echo(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        // Rising edge detected, start timing
        startTime = get_absolute_time();
    }

    if (events & GPIO_IRQ_EDGE_FALL)
    {
        // Falling edge detected, end timing
        endTime = get_absolute_time();

        // Calculate pulse duration in microseconds
        uint64_t pulseDuration = absolute_time_diff_us(startTime, endTime);

        // Check for timeout
        if (pulseDuration > TIMEOUT_US)
        {
            printf("Error: Timeout\n");
            return;
        }

        // Process the pulse duration
        getUltrasonicDetection(pulseDuration);
    }
}

// Timer callback to trigger ultrasonic measurement
bool repeating_timer_callback(struct repeating_timer *t)
{
    sendInitiationPulse(TRIGGER_PIN);
    return true;
}

// Setup function for ultrasonic sensor pins
void setupUltrasonicPins(uint trigPin, uint echoPin)
{
    // Initialize trigger pin
    gpio_init(trigPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_put(trigPin, 0); // Ensure trigger pin is low

    // Initialize echo pin
    gpio_init(echoPin);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_pull_down(echoPin);

    // Set up interrupt handler for echo pin
    gpio_set_irq_enabled_with_callback(echoPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &handle_echo);
}

int main()
{
    // Initialize standard IO
    stdio_init_all();

    // Setup ultrasonic sensor pins
    setupUltrasonicPins(TRIGGER_PIN, ECHO_PIN);

    // Initialize the EMA value to a reasonable starting point (optional)
    ultrasonic_ema = TIMEOUT_US * 0.0343f / 2.0f;

    // Add a repeating timer to trigger measurements every 100ms
    struct repeating_timer timer;
    if (!add_repeating_timer_ms(100, repeating_timer_callback, NULL, &timer))
    {
        printf("Failed to add repeating timer\n");
        return 1;
    }

    // Main loop remains responsive and can perform other tasks
    while (1)
    {
        tight_loop_contents();
    }
}