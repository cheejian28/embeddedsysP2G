#include <stdio.h>
#include <math.h>
#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

#include "wifi_communication.h"
#include "udp_server_communication.h"
#include "udp_client_communication.h"
#include "ultrasonic.h"
#include "wheel_encoder.h"

// Definitions and configurations
#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define DEBOUNCE_US 100            ///< Debounce time in microseconds to avoid noise
#define MAX_PULSE_WIDTH_US 2000000 ///< Maximum pulse width to detect (1 second in microseconds)

// Define macros for fmaxf and fminf
#define fmaxf(a, b) ((a) > (b) ? (a) : (b))
#define fminf(a, b) ((a) < (b) ? (a) : (b))

#define LED_PIN1 10
#define LED_PIN2 11
#define LED_PIN3 12
#define LED_PIN4 13
#define LED_PIN5 14
#define LED_PIN6 15
#define BUTTON_PIN 22

#define MOTOR_A_IN1 0
#define MOTOR_A_IN2 1
#define MOTOR_A_PWM 4 // left motor

#define MOTOR_B_IN1 2
#define MOTOR_B_IN2 3
#define MOTOR_B_PWM 5 // right motor

#define DEBOUNCE_BUTTON_TIME_MS 20
#define PWM_FREQ 100.0f
#define MAX_INTEGRAL 100.0f
#define MIN_DUTY_CYCLE 0.6f
#define MAX_DUTY_CYCLE 0.95f

// PID controller parameters
float Kp_left = 0.1f;   
float Ki_left = 0.01f;  
float Kd_left = 0.0001f;

float Kp_right = 0.065f;
float Ki_right = 0.01f; 
float Kd_right = 0.0001f;

float duty_cycle = 0.5f; // duty cycle %
bool is_speed_control_active = true;

float setpoint = 20.0;          // Desired speed (cm/s)
float integral_motor_A = 0.0;   // Integral term for left motor
float integral_motor_B = 0.0;   // Integral term for right motor
float prev_error_motor_A = 0.0; // Previous error for left motor
float prev_error_motor_B = 0.0; // Previous error for right motor

// bool button_state = false;          // Debounced button state
// uint32_t last_debounce_time_ms = 0; // Timestamp of the last state change

float cumulative_distance = 0.0;

char action_received[128];

char direction[5];
float speed = 0;

TaskHandle_t moveTaskHandle;
TaskHandle_t speedTaskHandle;
TaskHandle_t ultrasonicTaskHandle;
TaskHandle_t serverTaskHandle;
TaskHandle_t clientTaskHandle;
// TaskHandle_t printTaskHandle;

// Function prototypes
void vWifiTask(void *pvParameters);
void vServerTask(void *pvParameters);
void message_handler(const char *message);

float compute_pid(float setpoint, float current_motor_speed, float *integral, float *prev_error, float Kp, float Ki, float Kd);
void setup_pwm(uint gpio, float freq, float duty_cycle);

// bool debounce(bool new_state);

// double calculate_wheel_speed(int64_t pulse_width_us);
// double calculate_distance_traveled(int pulse_count);

typedef enum {
  STATE_REMOTE,
  STATE_AUTONOMOUS,
  STATE_OBSTACLE_DETECTED,
  STATE_STOP
} RobotState;

RobotState robot_state = STATE_REMOTE;

// Function to set the LEDs according to the state
// void set_behaviour(RobotState state) {
//   switch (state) {
//     case STATE_TURNING_RIGHT:
//     //   gpio_put(LED1_PIN, 0);
//       break;
//     case STATE_REMOTE:
//     // vTaskSuspend(speedTaskHandle); //change handle to accelerometer task
//     //   gpio_put(LED1_PIN, 0);
//     //   gpio_put(LED2_PIN, 1);
//       break;
//     case STATE_AUTONOMOUS:
//     //   gpio_put(LED1_PIN, 1);
//     //   gpio_put(LED2_PIN, 0);
//       break;
//     case STATE_STOP:
//       break;
//   }
// }

void vWifiTask(void *pvParameters){
    while(1){
        // printf("Checking Wifi Connection\n");
        checkWifiConnection();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vServerTask(void *pvParameters){
    bool isConnected = false;
    while(1){
        // printf("Checking TCP Connection to Server\n");
        isConnected = init_udp_server(42069, message_handler);
        if(isConnected){
            printf("[UDP Server Task] Connected to Server! Suspending Task\n");
            vTaskSuspend(serverTaskHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vClientTask(void *pvParameters){
    bool isConnected = false;
    while(1){
        // printf("Checking TCP Connection to Server\n");
        isConnected = init_udp_client("172.20.10.9", 42069);
        if(isConnected){
            printf("[UDP Server Task] Failed Client Task\n");
            vTaskSuspend(clientTaskHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void message_handler(const char *message){
    // printf("\033[2J\033[H");
    // printf("Main Program Received: %s\n", message);
    strncpy(action_received, message, sizeof(action_received) - 1);
    action_received[sizeof(action_received) - 1] = '\0'; // Ensure null-termination
}

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
    gpio_put(MOTOR_A_IN1, 0);
    gpio_put(MOTOR_A_IN2, 1);
    gpio_put(MOTOR_B_IN1, 0);
    gpio_put(MOTOR_B_IN2, 1);
}

// Function to move backward
void move_backward()
{
    gpio_put(MOTOR_A_IN1, 1);
    gpio_put(MOTOR_A_IN2, 0);
    gpio_put(MOTOR_B_IN1, 1);
    gpio_put(MOTOR_B_IN2, 0);
}

void turn_left()
{
    gpio_put(MOTOR_A_IN1, 1); //1 to turn on spot
    gpio_put(MOTOR_A_IN2, 0);
    gpio_put(MOTOR_B_IN1, 0);
    gpio_put(MOTOR_B_IN2, 1);
}

void turn_right()
{
    gpio_put(MOTOR_A_IN1, 0);
    gpio_put(MOTOR_A_IN2, 1);
    gpio_put(MOTOR_B_IN1, 1); //1 to turn on spot
    gpio_put(MOTOR_B_IN2, 0);
}

// bool debounce(bool new_state)
// {
//     uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());

//     if (current_time_ms - last_debounce_time_ms >= DEBOUNCE_BUTTON_TIME_MS)
//     {
//         last_debounce_time_ms = current_time_ms;
//         button_state = new_state;
//     }
//     return button_state;
// }

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
    uint32_t divider = clock_freq / (freq * 65535);
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
        if (is_speed_control_active)
        {
            // Compute distance-based speed reduction factor
            float speed_factor = 1.0f;
            if (distance < 70.0f) // Adjust threshold as needed
            {
                // Reduce speed as distance decreases
                speed_factor = distance / 70.0f; // Scale between 0 (0 cm) and 1 (50 cm)
                speed_factor = fmaxf(speed_factor, MIN_DUTY_CYCLE); // Minimum speed factor to avoid stopping
            }

            // PID controllers
            float control_signal_a = compute_pid(setpoint, speedLeftEncoder, &integral_motor_A, &prev_error_motor_A, Kp_left, Ki_left, Kd_left);
            float control_signal_b = compute_pid(setpoint, speedRightEncoder, &integral_motor_B, &prev_error_motor_B, Kp_right, Ki_right, Kd_right);

            // Adjust duty cycles based on control signals and speed factor
            float new_duty_cycle_a = (duty_cycle + control_signal_a) * speed_factor;
            float new_duty_cycle_b = (duty_cycle + control_signal_b) * speed_factor;

            // Constrain duty cycles to valid range
            new_duty_cycle_a = fminf(fmaxf(new_duty_cycle_a, MIN_DUTY_CYCLE), MAX_DUTY_CYCLE);
            new_duty_cycle_b = fminf(fmaxf(new_duty_cycle_b, MIN_DUTY_CYCLE), MAX_DUTY_CYCLE);

            // Apply the duty cycles
            setup_pwm(MOTOR_A_PWM, PWM_FREQ, new_duty_cycle_a);
            setup_pwm(MOTOR_B_PWM, PWM_FREQ, new_duty_cycle_b);
        }
        else
        {
            setup_pwm(MOTOR_A_PWM, PWM_FREQ, 0.0f);
            setup_pwm(MOTOR_B_PWM, PWM_FREQ, 0.0f);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
    }
}

void task_move(__unused void *params)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    bool button_pressed = !gpio_get(BUTTON_PIN); 

    while (true)
    {
        // if (button_pressed == !gpio_get(BUTTON_PIN))
        // {
        //     move_forward();
        // }
        cumulative_distance = (cumulative_distance_left + cumulative_distance_right) / 2;

        switch (robot_state)
        {
            case STATE_REMOTE:
                gpio_put(LED_PIN1, 1);
                gpio_put(LED_PIN2, 0);
                is_speed_control_active = true;
                // printf("Action Received: %s\n", action_received);
                sscanf(action_received, "%s %f%%", direction, &speed);

                printf("Moving Direction: %s, Speed: %f\n", direction, speed);
                
                if(strcmp(direction, "f") == 0){
                    move_forward();
                }
                else if(strcmp(direction, "b") == 0){
                    move_backward();
                }
                else if(strcmp(direction, "l") == 0){
                    turn_left();
                }
                else if(strcmp(direction, "r") == 0){
                    turn_right();
                }
                else if(strcmp(direction, "s") == 0){
                    stop_motors();
                }
                break;

            // case STATE_OBSTACLE_DETECTED:
            //     stop_motors();
            //     is_speed_control_active = false;  // Flag for speed control task instead of suspending task
            //     reset_encoders(); // Reset encoders before turning                
            //     turn_right();
            //     robot_state = STATE_TURNING_RIGHT;  
            //     break;

            // case STATE_TURNING_RIGHT:
            //     // Wait until left encoder counts 9 pulses
            //     if (total_num_edge_l >= 9)
            //     {
            //         stop_motors();
            //         reset_encoders(); // Reset encoders after turning
            //         is_speed_control_active = true; // Reactivate speed control
            //         move_forward();
            //         robot_state = STATE_MOVING_FORWARD_AFTER_TURN;
            //     }
            //     break;

            // case STATE_MOVING_FORWARD_AFTER_TURN:
            //     if (cumulative_distance >= 90.0f)
            //     {
            //         stop_motors();
            //         // vTaskDelete(NULL);
            //         robot_state = STATE_REMOTE;
            //     }        
            //     break;

            case STATE_AUTONOMOUS:
                // based on IR
                // if ultrasonic distance threshold set robot_state to measure distance to object when stopped
                break;

            case STATE_STOP:
                gpio_put(LED_PIN1, 0);
                gpio_put(LED_PIN2, 1);
                stop_motors();
                break;

            default:
                break;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(25));
    }
}

void ultrasonic_task(void *pvParameters)
{
    while (1)
    {
        // Trigger ultrasonic pulse and calculate distance
        triggerUltraSonicPins();
        vTaskDelay(pdMS_TO_TICKS(300)); // Wait for echo
        
        if (distance < 25.0f)
        {
            robot_state = STATE_STOP;
            printf("Obstacle detected! Stopping the robot\n");
            vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 1 second
            robot_state = STATE_REMOTE;
            vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for 1 second
        }
        // else
        // {
        //     robot_state = STATE_REMOTE;
        //     // printf("Resumed remote state");
        // }
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
    xTaskCreate(task_move, "MoveTask", 2048, NULL, 4, &moveTaskHandle);
    xTaskCreate(task_motor_speed, "SpeedTask", 2048, NULL, 4, &speedTaskHandle);
    xTaskCreate(ultrasonic_task, "UltrasonicTask", 2048, NULL, 5, &ultrasonicTaskHandle);

    xTaskCreate(vWifiTask, "Wifi Task", 256, NULL, 1, NULL);
    xTaskCreate(vServerTask, "UDP Server Task", 256, NULL, 1, &serverTaskHandle);
    // xTaskCreate(vClientTask, "UDP Client Task", 256, NULL, 1, &clientTaskHandle);
    // xTaskCreate(task_print, "PrintTask", 2048, NULL, 1, &printTaskHandle);

    vTaskStartScheduler();
}

int main()
{
    stdio_init_all();
    set_ssid_password("SimPhone", "a1234567");
    // set_callback(message_handler);

    gpio_init(LED_PIN1);
    gpio_init(LED_PIN2);
    gpio_init(LED_PIN3);
    gpio_init(LED_PIN4);
    gpio_init(LED_PIN5);
    gpio_init(LED_PIN6);
    gpio_set_dir(LED_PIN1, GPIO_OUT);
    gpio_set_dir(LED_PIN2, GPIO_OUT);
    gpio_set_dir(LED_PIN3, GPIO_OUT);
    gpio_set_dir(LED_PIN4, GPIO_OUT);
    gpio_set_dir(LED_PIN5, GPIO_OUT);
    gpio_set_dir(LED_PIN6, GPIO_OUT);

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_OUT);
    
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
