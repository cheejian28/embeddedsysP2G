#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define IR_SENSOR_CHANNEL 0    // ADC channel 0 (GPIO 26) for the IR sensor
#define N_SAMPLES 1000         // no of samples for capturing
#define THRESHOLD 1000         // set threshold for identify black/white surface
#define SURFACE_THRESHOLD 3000 // set threshold for surface/no surface detection

uint16_t sample_buf[N_SAMPLES];
uint32_t start_time;
int last_surface = -1;

void __not_in_flash_func(adc_capture)(uint16_t *buf, size_t count)
{
    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);
    for (size_t i = 0; i < count; i++)
        buf[i] = adc_fifo_get_blocking();
    adc_run(false);
    adc_fifo_drain();
}

int main(void)
{
    stdio_init_all();
    adc_init();
    adc_select_input(IR_SENSOR_CHANNEL);

    start_time = to_ms_since_boot(get_absolute_time()); // start timer

    while (1)
    {
        // read IR sensor ADC value
        uint32_t result = adc_read();
        const float conversion_factor = 3.3f / (1 << 12);
        float voltage = result * conversion_factor;

        // display the voltage reading
        printf("\nVoltage: %f V\n", voltage);

        int current_surface = -1; // set intial surface to no surface(default)

        if (result < SURFACE_THRESHOLD)
        {
            if (result > THRESHOLD)
            {
                printf("Black surface detected\n");
                printf("ADC value: %d\n", result);
                current_surface = 1;
            }
            else
            {
                printf("White surface detected\n");
                printf("ADC value: %d\n", result);
                current_surface = 0;
            }
        }
        else
        {
            printf("There is no surface.\n");
            current_surface = -1;
        }

        // print elapsed time on surface change
        if (current_surface != last_surface)
        {
            // calculate and print time spent on the last surface
            if (last_surface != -1)
            {
                uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
                if (last_surface == 1)
                    printf("Time spent on black surface: %d ms\n", elapsed_time);
                else if (last_surface == 0)
                    printf("Time spent on white surface: %d ms\n", elapsed_time);
                else if (last_surface == -1)
                    printf("Time with no surface: %d ms\n", elapsed_time);
            }

            // reset the timer and update the last surface
            start_time = to_ms_since_boot(get_absolute_time());
            last_surface = current_surface;
        }

        sleep_ms(2000); // add delay
    }
}
