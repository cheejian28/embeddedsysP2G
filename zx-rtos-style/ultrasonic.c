#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define TIMEOUT 50000
#define DEBOUNCE 100

volatile absolute_time_t rise_time;
volatile absolute_time_t fall_time;
volatile float distance = 0.0f;

void setUpUltrasonicPins()
{
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_put(TRIGGER_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);
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
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        fall_time = get_absolute_time();
        uint64_t pulse_duration = absolute_time_diff_us(rise_time, fall_time);
        if (pulse_duration > TIMEOUT)
        {
            printf("Error: Timeout\n");
            return;
        }
        distance = (pulse_duration * 0.0343f) / 2.0f;
        // printf("Distance: %.2f cm\n", distance);
    }
}

void ultrasonic_task(void *pvParameters)
{
    while (1)
    {
        // Trigger ultrasonic pulse and calculate distance
        triggerUltraSonicPins();
        vTaskDelay(pdMS_TO_TICKS(300)); // Wait for echo
    }
}