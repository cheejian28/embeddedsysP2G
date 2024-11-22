#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define TIMEOUT 50000

const int debounce_time = 100;
volatile absolute_time_t rise_time;
volatile absolute_time_t fall_time;
extern volatile float distance=100.0f;


void setUpUltrasonicPins()
{
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_put(TRIGGER_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);

    // Configure GPIO interrupt for both rising and falling edges
    }

void triggerUltraSonicPins()
{
    gpio_put(TRIGGER_PIN, 1);
    sleep_us(10);
    gpio_put(TRIGGER_PIN, 0);
    // printf("Triggered Ultrasonic Pins\n");  // Debug print
}

void handle_echo(uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        rise_time = get_absolute_time();
    }
    if (events & GPIO_IRQ_EDGE_FALL)
    {
        fall_time = get_absolute_time();
    }

    if(!is_nil_time(fall_time) && !is_nil_time(rise_time))
    {
        uint64_t time_diff = absolute_time_diff_us(rise_time, fall_time);
        // printf("Rise time: %lld\n", to_us_since_boot(rise_time));
        // printf("Fall time: %lld\n", to_us_since_boot(fall_time));
        // printf("abs time diff: %llu\n", time_diff);
           
        if(time_diff < debounce_time)
        {
            printf("Invalid pulse width\n");
            rise_time = nil_time;
            fall_time = nil_time;
            return;
        }
        if(time_diff > TIMEOUT)
        {
            printf("Timeout\n");
            rise_time = nil_time;
            fall_time = nil_time;
            return;
        }

        uint64_t pulse_width = time_diff;
        distance = (pulse_width * 0.0343f) / 2.0f;
        // printf("Distance: %.2f cm\n", distance); // Debug print
        rise_time = nil_time;
        fall_time = nil_time;
    }
}