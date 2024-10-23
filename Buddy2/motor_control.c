#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define DIR_PIN1 0          // GPIO pin to drive 1 direction
#define DIR_PIN2 1          // for other direction
#define PWM_PIN 2           // PWM GPIO pin
#define CON_PIN1 20         // conditional 1
#define CON_PIN2 21         // conditional 2
#define CON_PIN3 22         // conditional 3
#define DEBOUNCE_TIME_MS 20 
#define NUM_OF_ELEMS_DUTY_CYCLE 2

float duty_cycles[NUM_OF_ELEMS_DUTY_CYCLE] = {0.5f, 1.0f};  //update definition of NUM_OF_ELEMS_DUTY_CYCLE
int duty_cycles_index = 0;

bool button_state = false;           // Current debounced button state
uint32_t last_debounce_time_ms = 0;  // Timestamp of the last state change

// Debounce function to filter out button state changes caused by noise
bool debounce(bool new_state)
{
    // Get current time in milliseconds
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());

    // Check if enough time has passed since the last state change
    if (current_time_ms - last_debounce_time_ms >= DEBOUNCE_TIME_MS) 
    {
        // Update the last state change timestamp
        last_debounce_time_ms = current_time_ms;

        // Update the debounced state
        button_state = new_state;
    }

    printf("Debounced state: %d\n", button_state);
    return button_state;
}

// Function to set up the PWM
void setup_pwm(uint gpio, float freq, float duty_cycle) {
    // Set the GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Calculate the PWM frequency and set the PWM wrap value
    float clock_freq = 125000000.0f;  // Default Pico clock frequency in Hz
    uint32_t divider = clock_freq / (freq * 65536);  // Compute divider for given frequency
    pwm_set_clkdiv(slice_num, divider);

    // Set the PWM wrap value (maximum count value)
    pwm_set_wrap(slice_num, 65535);  // 16-bit counter (0 - 65535)

    // Set the duty cycle
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535)); 

    // Enable the PWM
    pwm_set_enabled(slice_num, true);
    
    printf("PWM set on pin %d: Frequency = %.2f Hz, Duty cycle = %.2f%%\n", gpio, freq, duty_cycle * 100);
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    printf("PWM Setting Button pressed - GPIO %d interrupt handling\n", gpio);
    duty_cycles_index = (duty_cycles_index + 1) % NUM_OF_ELEMS_DUTY_CYCLE; 
    setup_pwm(PWM_PIN, 100.0f, duty_cycles[duty_cycles_index]);  
}

int main() {
    stdio_init_all();

    // Initialize GPIO pins for direction control
    gpio_init(DIR_PIN1);
    gpio_init(DIR_PIN2);
    gpio_set_dir(DIR_PIN1, GPIO_OUT);
    gpio_set_dir(DIR_PIN2, GPIO_OUT);

    setup_pwm(PWM_PIN, 100.0f, duty_cycles[duty_cycles_index]);  // 100 Hz frequency, x% duty cycle

    while (true) {
        bool button_pressed_1 = !gpio_get(CON_PIN1); 
        bool button_pressed_2 = !gpio_get(CON_PIN2); 
        bool button_pressed_pwm = !gpio_get(CON_PIN3);

        // Change PWM setting on button press
        if(button_pressed_pwm)  
        {
            printf("\nButton pressed\n");  
            button_state = debounce(button_pressed_pwm); 
            
            // If state is high, change PWM setting
            if (button_state)
            {
                // Enable interrupts on GPIO pin for falling & rising edges (button pressed)
                // gpio_set_irq_enabled_with_callback(CON_PIN3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
                duty_cycles_index = (duty_cycles_index + 1) % NUM_OF_ELEMS_DUTY_CYCLE; // Circular index
                setup_pwm(PWM_PIN, 100.0f, duty_cycles[duty_cycles_index]);  // 100 Hz frequency, x% duty cycle
            }
            else 
            {
                printf("Failed to change settings\n");
            }
            sleep_ms(200);
        }

        // Change direction based on button press
        if (!button_pressed_1 && !button_pressed_2)
        {
            gpio_put(DIR_PIN1, 0);
            gpio_put(DIR_PIN2, 0);
        }
        else if (button_pressed_1 && !button_pressed_2)
        {
            printf("Button 1 pressed\n");  
            button_state = debounce(button_pressed_1);
            
            if (button_state)
            {
                // If state is high, drive gpio
                gpio_put(DIR_PIN1, 1);
                gpio_put(DIR_PIN2, 0);
            }
        }
        else if (button_pressed_2 && !button_pressed_1)  
        {
            printf("Button 2 pressed\n");  
            button_state = debounce(button_pressed_2); 
            
            if (button_state)
            {
                // If state is high, drive gpio
                gpio_put(DIR_PIN2, 1);
                gpio_put(DIR_PIN1, 0);
            }
        }
        else
        {
            printf("Invalid state\n");
            gpio_put(DIR_PIN2, 0);
            gpio_put(DIR_PIN1, 0);
        }
    }
    return 0;
}
