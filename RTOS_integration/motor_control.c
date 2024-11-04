#include <stdio.h>
#include <math.h>
#include <inttypes.h> // Include the inttypes.h header for PRId64 macro
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define ENCODER_PIN_LEFT 7         ///< GPIO pin number for the left encoder output pin
#define ENCODER_PIN_RIGHT 8        ///< GPIO pin number for the right encoder output pin
#define DEBOUNCE_US 100            ///< Debounce time in microseconds to avoid noise
#define MAX_PULSE_WIDTH_US 2000000 ///< Maximum pulse width to detect (1 second in microseconds)
#define WHEEL_DIAMETER_CM 10.0     ///< Wheel diameter in centimeters
#define PPR 8

#define ENCODER_CIRCUMFERENCE_CM 6.28f // Encoder wheel circumference in cm
#define WHEEL_CIRCUMFERENCE_CM 21.0f   // Actual wheel circumference in cm
#define PULSES_PER_ROTATION 20         // Number of pulses per full rotation of the encoder wheel

#define DISTANCE_PER_PULSE_ENCODER (ENCODER_CIRCUMFERENCE_CM / PULSES_PER_ROTATION) // cm per pulse
#define WHEEL_TO_ENCODER_RATIO (WHEEL_CIRCUMFERENCE_CM / ENCODER_CIRCUMFERENCE_CM)

#define MOTOR_A_IN1 0
#define MOTOR_A_IN2 1
#define MOTOR_A_PWM 2 // left motor
#define MOTOR_B_IN1 3
#define MOTOR_B_IN2 4
#define MOTOR_B_PWM 5 // right motor

#define CON_PIN1 20 // conditional 1
#define CON_PIN2 21 // conditional 2
#define CON_PIN3 22 // conditional 3
#define DEBOUNCE_TIME_MS 20
#define MOVE_DURATION_MS 10000
#define PWM_FREQ 100.0f

// Initialize PID controller parameters / weights
float Kp_left = 0.007f;
float Ki_left = 0.0001f;
float Kd_left = 0.0001f;

float Kp_right = 0.007f;
float Ki_right = 0.0001f;
float Kd_right = 0.0001f;

float duty_cycle = 0.5f; // duty cycle %

// Initialize variables
float setpoint = 180.0;       // Desired speed (cm/s)
float integral_motor_A = 0.0; // Integral term; component that accumulates the error over time
                              // addresses the cumulative effect of past errors, helping to eliminate steady-state errors (offsets).
                              // If there’s a persistent error (even if small), the integral term will grow, leading to a
                              // stronger corrective action. This helps the system reach the setpoint more accurately over time
float integral_motor_B = 0.0;
float prev_error_motor_A = 0.0; // Previous error term; deviation of current value from setpoint
float prev_error_motor_B = 0.0;

// debouncing variables
bool button_state = false;          // Current debounced button state
uint32_t last_debounce_time_ms = 0; // Timestamp of the last state change

bool is_moving = false;          // Flag to indicate if the car is moving
absolute_time_t move_start_time; // Start time for the move duration

// Variables for the left encoder
volatile int64_t pulse_width_us_left = 0;  ///< Width of the pulse in microseconds for left encoder.
volatile int pulse_count_left = 0;         ///< Counter for pulses on the left encoder
volatile bool new_pulse_data_left = false; ///< Flag to indicate new pulse width data for left encoder
absolute_time_t last_rise_time_left;
absolute_time_t last_fall_time_left;
double left_wheel_speed = 0.0f;

// Variables for the right encoder
volatile int64_t pulse_width_us_right = 0;  ///< Width of the pulse in microseconds for right encoder.
volatile int pulse_count_right = 0;         ///< Counter for pulses on the right encoder
volatile bool new_pulse_data_right = false; ///< Flag to indicate new pulse width data for right encoder
absolute_time_t last_rise_time_right;
absolute_time_t last_fall_time_right;
double right_wheel_speed = 0.0f;

double wheel_circumference; ///< Circumference of the wheel
double distance_per_pulse;  ///< Distance traveled per pulse

// Function prototypes
void encoder_callback(uint gpio, uint32_t events);
void init_wheel_encoder();
void init_wheel_parameters();
double calculate_speed_left();
double calculate_speed_right();

void stop_motors()
{
    gpio_put(MOTOR_A_IN1, 0);
    gpio_put(MOTOR_A_IN2, 0);
    gpio_put(MOTOR_B_IN1, 0);
    gpio_put(MOTOR_B_IN2, 0);
    printf("Motors stopped\n");
}

void move_forward()
{
    gpio_put(MOTOR_A_IN1, 1);
    gpio_put(MOTOR_A_IN2, 0);
    gpio_put(MOTOR_B_IN1, 0);
    gpio_put(MOTOR_B_IN2, 1);
    printf("Motors moving forward\n");
}

void move_backward()
{
    gpio_put(MOTOR_A_IN1, 0);
    gpio_put(MOTOR_A_IN2, 1);
    gpio_put(MOTOR_B_IN1, 1);
    gpio_put(MOTOR_B_IN2, 0);
    printf("Motors moving backward\n");
}

void turn_left()
{
    gpio_put(MOTOR_A_IN1, 0);
    gpio_put(MOTOR_A_IN2, 1);
    gpio_put(MOTOR_B_IN1, 0);
    gpio_put(MOTOR_B_IN2, 1);
    printf("Turning left\n");
}

void turn_right()
{
    gpio_put(MOTOR_A_IN1, 1);
    gpio_put(MOTOR_A_IN2, 0);
    gpio_put(MOTOR_B_IN1, 1);
    gpio_put(MOTOR_B_IN2, 0);
    printf("Turning right\n");
}

// Debounce button state changes caused by noise
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
    return button_state;
}

// Compute control signal
float compute_pid(float setpoint, float current_motor_speed, float *integral, float *prev_error, float Kp, float Ki, float Kd)
{
    float error, derivative;

    // Update error, integral & derivative
    if (current_motor_speed <= 0)
    {
        error = 0.0f;
        *integral = 0.0f;
        derivative = 0.0f;
    }
    else
    {
        error = setpoint - current_motor_speed; // Compute error; -ve if current > desired, +ve correction
        *integral += error;
        derivative = error - *prev_error; // Derivative helps reduce overshoot & oscillations in the system response
    }

    // Compute control signal; adjust the weights of each term to tune PID controller
    float control_signal = (Kp * error) + (Ki * (*integral)) + (Kd * derivative);

    // Update previous error for next iteration
    *prev_error = error;

    printf("Setpoint: %f\n", setpoint);
    printf("Current Speed: %f\n", current_motor_speed);
    printf("Error: %f\n", error);

    printf("Integral: %f\n", *integral);
    printf("Derivative: %f\n", derivative);
    printf("Control Signal = %f\n", control_signal);
    return control_signal;
}

void setup_pwm(uint gpio, float freq, float duty_cycle)
{
    if (duty_cycle > 1.0f)
    {
        duty_cycle = 1.0f;
    }
    else if (duty_cycle < 0.0f)
    {
        duty_cycle = 0.0f;
    }

    // Set the GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Calculate the PWM frequency and set the PWM wrap value
    float clock_freq = 125000000.0f;                // Default Pico clock frequency in Hz
    uint32_t divider = clock_freq / (freq * 65536); // Compute divider for given frequency
    pwm_set_clkdiv(slice_num, divider);

    // Set the PWM wrap value (maximum count value)
    pwm_set_wrap(slice_num, 65535); // 16-bit counter (0 - 65535)

    // Set the duty cycle
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535));

    // Enable the PWM
    pwm_set_enabled(slice_num, true);

    printf("PWM set on pin %d: Frequency = %.2f Hz, Duty cycle = %.2f%%\n", gpio, freq, duty_cycle * 100);
}

void encoder_callback(uint gpio, uint32_t events)
{
    absolute_time_t now = get_absolute_time();

    if (gpio == ENCODER_PIN_LEFT)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            last_rise_time_left = now;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            pulse_width_us_left = absolute_time_diff_us(last_rise_time_left, now);
            new_pulse_data_left = true;
            pulse_count_left++; // Increment pulse count
        }
    }
    else if (gpio == ENCODER_PIN_RIGHT)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            last_rise_time_right = now;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            pulse_width_us_right = absolute_time_diff_us(last_rise_time_right, now);
            new_pulse_data_right = true;
            pulse_count_right++; // Increment pulse count
        }
    }
}

double calculate_wheel_speed(int64_t pulse_width_us)
{
    if (pulse_width_us > 0 && pulse_width_us < MAX_PULSE_WIDTH_US)
    {
        double time_interval_s = pulse_width_us / 1000000.0;
        // Speed based on encoder wheel
        double speed_encoder_cm_s = DISTANCE_PER_PULSE_ENCODER / time_interval_s;
        // Adjust to actual wheel speed
        double speed_actual_cm_s = speed_encoder_cm_s * WHEEL_TO_ENCODER_RATIO;
        return speed_actual_cm_s;
    }
    printf("Cannot calculate wheel speed: Invalid pulse width detected\n");
    return 0.0;
}

double calculate_distance_traveled(int pulse_count)
{
    // Distance traveled by encoder wheel
    double encoder_distance_cm = pulse_count * DISTANCE_PER_PULSE_ENCODER;
    // Adjust to actual wheel distance
    double actual_distance_cm = encoder_distance_cm * WHEEL_TO_ENCODER_RATIO;
    return actual_distance_cm;
}

void task_move(__unused void *params)
{
    while (true)
    {
        bool button_pressed_1 = !gpio_get(CON_PIN1);
        bool button_pressed_2 = !gpio_get(CON_PIN2);

        if (button_pressed_1 && !button_pressed_2)
        {
            if (!is_moving)
            {
                is_moving = true;
                move_start_time = get_absolute_time();
                move_forward();
            }
        }
        if (button_pressed_2 && !button_pressed_1)
        {
            if (!is_moving)
            {
                is_moving = true;
                move_start_time = get_absolute_time();
                move_backward();
            }
        }

        if (is_moving)
        {
            if (absolute_time_diff_us(move_start_time, get_absolute_time()) >= MOVE_DURATION_MS * 700)
            {
                stop_motors();
                is_moving = false;
            }
        }
        vTaskDelay(10);
    }
}

void task_speed(__unused void *params)
{
    static int total_pulse_count_left = 0;
    static int total_pulse_count_right = 0;
    static double total_distance_left = 0.0;
    static double total_distance_right = 0.0;

    while (true)
    {
        if (new_pulse_data_left || new_pulse_data_right)
        {
            // Accumulate pulse counts
            total_pulse_count_left += pulse_count_left;
            total_pulse_count_right += pulse_count_right;

            // Reset pulse counts
            pulse_count_left = 0;
            pulse_count_right = 0;

            // Calculate wheel speeds
            left_wheel_speed = calculate_wheel_speed(pulse_width_us_left);
            right_wheel_speed = calculate_wheel_speed(pulse_width_us_right);

            // Calculate total distance traveled
            total_distance_left = calculate_distance_traveled(total_pulse_count_left);
            total_distance_right = calculate_distance_traveled(total_pulse_count_right);

            // Clear new data flags
            new_pulse_data_left = false;
            new_pulse_data_right = false;

            // Print total distances
            printf("Total Distance Left: %.2f cm\n", total_distance_left);
            printf("Total Distance Right: %.2f cm\n", total_distance_right);
        }
        else
        {
            left_wheel_speed = 0.0f;
            right_wheel_speed = 0.0f;
        }

        // PID controllers
        float control_signal_a = compute_pid(setpoint, left_wheel_speed, &integral_motor_A, &prev_error_motor_A, Kp_left, Ki_left, Kd_left);
        float control_signal_b = compute_pid(setpoint, right_wheel_speed, &integral_motor_B, &prev_error_motor_B, Kp_right, Ki_right, Kd_right);

        // Adjust duty cycle based on control signals
        setup_pwm(MOTOR_A_PWM, PWM_FREQ, duty_cycle + control_signal_a);
        setup_pwm(MOTOR_B_PWM, PWM_FREQ, duty_cycle + control_signal_b);

        vTaskDelay(100);
    }
}

void vLaunch()
{
    TaskHandle_t moveTaskHandle;
    TaskHandle_t speedTaskHandle;
    xTaskCreate(task_move, "MoveTask", configMINIMAL_STACK_SIZE, NULL, 0, &moveTaskHandle);
    xTaskCreate(task_speed, "SpeedTask", configMINIMAL_STACK_SIZE, NULL, 2, &speedTaskHandle);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main()
{
    stdio_init_all();

    // Initialize GPIO pins for direction control
    gpio_init(MOTOR_A_IN1);
    gpio_init(MOTOR_A_IN2);
    gpio_init(MOTOR_B_IN1);
    gpio_init(MOTOR_B_IN2);
    gpio_set_dir(MOTOR_A_IN1, GPIO_OUT);
    gpio_set_dir(MOTOR_A_IN2, GPIO_OUT);
    gpio_set_dir(MOTOR_B_IN1, GPIO_OUT);
    gpio_set_dir(MOTOR_B_IN2, GPIO_OUT);

    // init wheel encoder
    gpio_init(ENCODER_PIN_LEFT);
    gpio_set_dir(ENCODER_PIN_LEFT, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_LEFT);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_LEFT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);

    gpio_init(ENCODER_PIN_RIGHT);
    gpio_set_dir(ENCODER_PIN_RIGHT, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_RIGHT);
    gpio_set_irq_enabled(ENCODER_PIN_RIGHT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Initialize wheel parameters
    wheel_circumference = WHEEL_CIRCUMFERENCE_CM;
    distance_per_pulse = wheel_circumference / PPR; // Use actual wheel circumference

    /* Configure the hardware ready to run the demo. */
    const char *rtos_name = "FreeRTOS";
#if (portSUPPORT_SMP == 1)
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

#if (portSUPPORT_SMP == 1) && (configNUM_CORES == 2)
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();
#elif (RUN_FREERTOS_ON_CORE == 1)
    printf("Starting %s on core 1:\n", rtos_name);
    multicore_launch_core1(vLaunch);
    while (true)
        ;
#else
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();
#endif
    return 0;
}
