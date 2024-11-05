#ifndef ULTRASONIC_H
#define ULTRASONIC_H

// Include FreeRTOS headers for BaseType_t and UBaseType_t
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include "hardware/gpio.h"
#include "pico/time.h" // Include the header that defines struct repeating_timer

// Pin definitions and settings
#define TRIGGER_PIN 4
#define ECHO_PIN 5
#define TIMEOUT_US 26100
#define EMA_ALPHA 0.2f // Smoothing factor for EMA (0 < alpha <= 1)

// Extern variable to hold the EMA of the distance
extern volatile float distance;

// Function prototypes
void setupUltrasonicPins(); // Removed parameters for consistency
// void sendInitiationPulse(); // Removed parameters for consistency
void getUltrasonicDetection(uint64_t pulseLength);
void handle_echo(uint32_t events);

// FreeRTOS task for ultrasonic sensor
void ultrasonic_task();

#endif // ULTRASONIC_H
