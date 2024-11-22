#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>
#include "hardware/gpio.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define ENCODER_LEFT 6
#define ENCODER_RIGHT 8
#define PPR 20
#define WHEEL_DIAMETER 6.5f
#define WHEEL_CIRCUMFERENCE 21.8f

extern volatile int total_num_edge_l;
extern volatile int total_num_edge_r;

extern volatile float speedLeftEncoder;
extern volatile float speedRightEncoder;

extern volatile float cumulative_distance_left;
extern volatile float cumulative_distance_right;

// Function prototypes
void gpio_callback(uint gpio, uint32_t events);
void setup_encoder();
#endif // WHEEL_ENCODER_H
