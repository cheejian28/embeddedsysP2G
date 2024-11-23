#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <stdio.h>

#define IR_PIN 27 // GPIO pin where the IR module is connected

extern volatile uint32_t line_input = 0;

void setUpLineFollowing()
{
    adc_init();
    adc_gpio_init(IR_PIN);
    adc_select_input(1);
}

void getIrLineValue()
{
    line_input = adc_read();
}

// // int main()
// {
//     // Initialize the GPIO pin for the IR sensor
//     stdio_init_all();
//     adc_init();
//     adc_gpio_init(IR_PIN);
//     adc_select_input(2);
//     // gpio_set_dir(IR_PIN, GPIO_IN);

//     while (true)
//     {
//         // Read the value from the IR sensor
//         uint32_t lineDetected = adc_read();

//         // printf("%d \n", lineDetected);

//         // if (lineDetected > 1000)
//         // {
//         //     printf("it's black \n");
//         // }
//         // else
//         // {
//         //     printf("it's white \n");
//         // }

//         // Add a small delay for stability
//         sleep_ms(100);
//     }
//     return 0;
// }
