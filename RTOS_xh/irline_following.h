#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <stdio.h>

#define IR_PIN 28

extern volatile uint32_t line_input;

void setUpLineFollowing();
void getIrLineValue();

#endif