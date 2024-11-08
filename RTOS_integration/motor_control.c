// main.c

#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

// Include the ultrasonic sensor and wheel encoder headers
#include "ultrasonic.h"
#include "wheel_encoder.h"

// Definitions and configurations
#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define DEBOUNCE_US 100            ///< Debounce time in microseconds to avoid noise
#define MAX_PULSE_WIDTH_US 2000000 ///< Maximum pulse width to detect (1 second in microseconds)

#define MOTOR_A_IN1 0
#define MOTOR_A_IN2 1
#define MOTOR_A_PWM 2 // left motor
#define MOTOR_B_IN1 3
#define MOTOR_B_IN2 4
#define MOTOR_B_PWM 5 // right motor

// #define CON_PIN1 20 // conditional 1
// #define CON_PIN2 21 // conditional 2
// #define CON_PIN3 22 // conditional 3
#define DEBOUNCE_TIME_MS 20
#define MOVE_DURATION_MS 10000
#define PWM_FREQ 100.0f
#define MAX_INTEGRAL 100.0f

// Define macros for fmaxf and fminf
#define fmaxf(a, b) ((a) > (b) ? (a) : (b))
#define fminf(a, b) ((a) < (b) ? (a) : (b))

TaskHandle_t moveTaskHandle;
TaskHandle_t speedTaskHandle;
TaskHandle_t ultrasonicTaskHandle;
// TaskHandle_t printTaskHandle;

// PID controller parameters
float Kp_left = 0.1f;
float Ki_left = 0.01f;
float Kd_left = 0.0000f;

float Kp_right = 0.08f;
float Ki_right = 0.01f;
float Kd_right = 0.0000f;

float duty_cycle = 0.5f; // duty cycle %

float setpoint = 25.0;          // Desired speed (cm/s)
float integral_motor_A = 0.0;   // Integral term for left motor
float integral_motor_B = 0.0;   // Integral term for right motor
float prev_error_motor_A = 0.0; // Previous error for left motor
float prev_error_motor_B = 0.0; // Previous error for right motor

bool button_state = false;          // Debounced button state
uint32_t last_debounce_time_ms = 0; // Timestamp of the last state change

bool is_moving = false;          // Flag to indicate if the car is moving
absolute_time_t move_start_time; // Start time for the move duration

bool turned_right = false;

float cumulative_distance = 0.0;

// Function prototypes
bool debounce(bool new_state);
float compute_pid(float setpoint, float current_motor_speed, float *integral, float *prev_error, float Kp, float Ki, float Kd);
void setup_pwm(uint gpio, float freq, float duty_cycle);
// double calculate_wheel_speed(int64_t pulse_width_us);
// double calculate_distance_traveled(int pulse_count);

// Function to stop motors
void stop_motors()
{
    gpio_put(MOTOR_A_IN1, 0);
    gpio_put(MOTOR_A_IN2, 0);
    gpio_put(MOTOR_B_IN1, 0);
    gpio_put(MOTOR_B_IN2, 0);
}

// Function to move forward
void move_forward()
{
    gpio_put(MOTOR_A_IN1, 1);
    gpio_put(MOTOR_A_IN2, 0);
    gpio_put(MOTOR_B_IN1, 0);
    gpio_put(MOTOR_B_IN2, 1);
}

// Function to move backward
void move_backward()
{
    gpio_put(MOTOR_A_IN1, 0);
    gpio_put(MOTOR_A_IN2, 1);
    gpio_put(MOTOR_B_IN1, 1);
    gpio_put(MOTOR_B_IN2, 0);
}

// Function to turn left
void turn_left()
{
    gpio_put(MOTOR_A_IN1, 0);
    gpio_put(MOTOR_A_IN2, 1);
    gpio_put(MOTOR_B_IN1, 0);
    gpio_put(MOTOR_B_IN2, 1);
}

// Function to turn right
void turn_right()
{
    gpio_put(MOTOR_A_IN1, 1);
    gpio_put(MOTOR_A_IN2, 0);
    gpio_put(MOTOR_B_IN1, 1);
    gpio_put(MOTOR_B_IN2, 0);
}

// Debounce function
bool debounce(bool new_state)
{
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());

    if (current_time_ms - last_debounce_time_ms >= DEBOUNCE_TIME_MS)
    {
        last_debounce_time_ms = current_time_ms;
        button_state = new_state;
    }
    return button_state;
}

// PID computation function
float compute_pid(float setpoint, float current_motor_speed, float *integral, float *prev_error, float Kp, float Ki, float Kd)
{
    float error, derivative;

    if (current_motor_speed <= 0)
    {
        error = 0.0f;
        *integral = 0.0f;
        derivative = 0.0f;
    }
    else
    {
        error = setpoint - current_motor_speed;
        *integral += error;
        derivative = error - *prev_error;
    }

    if (*integral > MAX_INTEGRAL)
    {
        *integral = MAX_INTEGRAL;
    }
    else if (*integral < -MAX_INTEGRAL)
    {
        *integral = -MAX_INTEGRAL;
    }

    float control_signal = (Kp * error) + (Ki * (*integral)) + (Kd * derivative);
    *prev_error = error;
    // printf("Current Speed: %f\n", current_motor_speed);
    // printf("Error: %f\n", error);

    // printf("Integral: %f\n", *integral);
    // printf("Derivative: %f\n", derivative);
    // printf("Control Signal = %f\n", control_signal);

    return control_signal;
}

// PWM setup function
void setup_pwm(uint gpio, float freq, float duty_cycle)
{

    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    float clock_freq = 125000000.0f;
    uint32_t divider = clock_freq / (freq * 65536);
    pwm_set_clkdiv(slice_num, divider);

    pwm_set_wrap(slice_num, 65535);
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535));
    pwm_set_enabled(slice_num, true);
    // printf("PWM set on pin %d: Frequency = %.2f Hz, Duty cycle = %.2f%%\n", gpio, freq, duty_cycle * 100);
}

void reset_encoders()
{
    total_num_edge_l = 0;
    total_num_edge_r = 0;
    cumulative_distance_left = 0.0;
    cumulative_distance_right = 0.0;
    cumulative_distance = 0.0;
}

// Task to move the car
void task_motor_speed(__unused void *params)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        // Compute distance-based speed reduction factor
        float speed_factor = 1.0f;
        if (distance < 50.0f) // Adjust threshold as needed
        {
            // Reduce speed as distance decreases
            speed_factor = distance / 50.0f; // Scale between 0 (0 cm) and 1 (50 cm)
            speed_factor = fmaxf(speed_factor, 0.2f); // Minimum speed factor to avoid stopping
        }

        // PID controllers
        float control_signal_a = compute_pid(setpoint, speedLeftEncoder, &integral_motor_A, &prev_error_motor_A, Kp_left, Ki_left, Kd_left);
        float control_signal_b = compute_pid(setpoint, speedRightEncoder, &integral_motor_B, &prev_error_motor_B, Kp_right, Ki_right, Kd_right);

        // Adjust duty cycles based on control signals and speed factor
        float new_duty_cycle_a = (duty_cycle + control_signal_a) * speed_factor;
        float new_duty_cycle_b = (duty_cycle + control_signal_b) * speed_factor;

        // Constrain duty cycles to valid range
        new_duty_cycle_a = fminf(fmaxf(new_duty_cycle_a, 0.25f), 0.725f);
        new_duty_cycle_b = fminf(fmaxf(new_duty_cycle_b, 0.25f), 0.775f);

        // Apply the duty cycles
        setup_pwm(MOTOR_A_PWM, PWM_FREQ, new_duty_cycle_a);
        setup_pwm(MOTOR_B_PWM, PWM_FREQ, new_duty_cycle_b);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
    }
}

// Task to control speed using PID
void task_move(__unused void *params)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    setup_pwm(MOTOR_A_PWM, PWM_FREQ, duty_cycle);
    setup_pwm(MOTOR_B_PWM, PWM_FREQ, duty_cycle);
    move_forward();
    while (true)
    {
        cumulative_distance = (cumulative_distance_left + cumulative_distance_right) / 2;

        if (distance < 20.0f && !turned_right)
        {
            stop_motors();
            vTaskSuspend(speedTaskHandle);
            setup_pwm(MOTOR_A_PWM, PWM_FREQ, 0.73f); //0.73
            setup_pwm(MOTOR_B_PWM, PWM_FREQ, 0.7f);  //0.7
            
            vTaskDelay(pdMS_TO_TICKS(1000));
            turn_right();
            reset_encoders(); // Reset encoders after turning
            
            while (total_num_edge_l < 10)
            {
                // ...existing code...
            }
            reset_encoders(); // Reset encoders after turning
            stop_motors();
            setup_pwm(MOTOR_A_PWM, PWM_FREQ, duty_cycle);
            setup_pwm(MOTOR_B_PWM, PWM_FREQ, duty_cycle);
            vTaskDelay(pdMS_TO_TICKS(1000));
            vTaskResume(speedTaskHandle);  // Resume the speed task
            move_forward();
            turned_right = true;
        }
        else if (turned_right && cumulative_distance >= 90.0f)
        {
            stop_motors();
            vTaskDelete(NULL);
        }
        // ...existing code...
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(300));
    }
}

// Task for handling prints
void task_print(__unused void *params)
{
    while (1)
    {
        // Print ultrasonic distance
        // printf("Ultrasonic Distance (EMA): %.3f cm\n", ultrasonic_ema);

        // Print wheel speeds
        // printf("Left Wheel Speed: %.2f cm/s\n", speedLeftEncoder);
        // printf("Right Wheel Speed: %.2f cm/s\n", speedRightEncoder);
        // printf("Total Left Pulses: %d\n", total_num_edge_l);
        // printf("Total Right Pulses: %d\n", total_num_edge_r);
        // printf("Cumulative Distance: %.2f cm\n", cumulative_distance);

        // Print total distance traveled
        // printf("Total Distance Left: %.2f cm\n", cumulative_distance_left);
        // printf("Total Distance Right: %.2f cm\n", cumulative_distance_right);
        printf("UltraSonic Distance: %.2f cm\n", distance);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust the delay as needed
    }
}

// Launch function to start tasks
void vLaunch()
{
    xTaskCreate(task_move, "MoveTask", 2048, NULL, 3, &moveTaskHandle);
    xTaskCreate(task_motor_speed, "SpeedTask", 2048, NULL, 2, &speedTaskHandle);
    xTaskCreate(ultrasonic_task, "UltrasonicTask", 2048, NULL, 2, &ultrasonicTaskHandle);
    // xTaskCreate(task_print, "PrintTask", 2048, NULL, 1, &printTaskHandle);

    // Start the scheduler
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

    // Initialize wheel encoders
    setup_encoder();
    setUpUltrasonicPins();

    // Initialize cumulative distance

    // Start FreeRTOS
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
