#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#include "wifi_communication.h"
#include "udp_server_communication.h"

void vWifiTask(void *pvParameters);
void vServerTask(void *pvParameters);
void message_handler(const char *message);
bool updateDashboardTimerCallback(repeating_timer_t *rt);
void updateDashboard();

typedef struct {
    uint32_t lastUpdateTime;
    char direction[32];
    char speed[32];
    char distance_travelled[32];
    char ultrasonic_distance[32];
    char ir_state[32];
} DASHBOARD_DATA;
DASHBOARD_DATA dashboard;
TaskHandle_t serverTaskHandle;

int main(){
    stdio_init_all();

    sleep_ms(5000);

    set_ssid_password("yo", "pleasestophacking");
    xTaskCreate(vWifiTask, "Wifi Task", 256, NULL, 1, NULL);

    xTaskCreate(vServerTask, "UDP Server Task", 256, NULL, 1, &serverTaskHandle);

    struct repeating_timer timer;
    add_repeating_timer_ms(-1000, updateDashboardTimerCallback, NULL, &timer);

    updateDashboard();

    vTaskStartScheduler();

    while(1);
}

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
        printf("\n[UDP Server Task] Checking UDP Connection to Server\n");
        isConnected = init_udp_server(42069, message_handler);
        if (isConnected) {
            printf("SUSPENDING UDP SERVER TASK!!!!!!!\n\n");
            updateDashboard();
            vTaskSuspend(serverTaskHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void message_handler(const char *message){
    printf("\033[8;1H");
    printf("Main Program Received: %s", message);
    printf("\033[K");

    char type[10];
    char data[32];

    dashboard.lastUpdateTime = to_ms_since_boot(get_absolute_time());

    sscanf(message, "%s %s", type, data);
    // printf("Type: %s, Data: %s\n", type, data);

    if(strcmp(type, "dir") == 0){
        printf("\033[3;1H");  // Move cursor to row 3, column 1
        printf("Car Direction: %s %s", data);
        printf("\033[K"); 
    }else if(strcmp(type, "sp") == 0){
        printf("\033[4;1H");  // Move cursor to row 3, column 1
        printf("Setpoint: %s", data);
        printf("\033[K");
    }else if(strcmp(type, "dt") == 0){
        // memcpy(dashboard.distance_travelled, data, sizeof(data));
        printf("\033[5;1H");  // Move cursor to row 3, column 1
        printf("Distance Travelled: %s", data);
        printf("\033[K");
    }else if(strcmp(type, "us") == 0){
        // memcpy(dashboard.ultrasonic_distance, data, sizeof(data));
        printf("\033[6;1H");  // Move cursor to row 3, column 1
        printf("Ultrasonic Distance: %s", data);
        printf("\033[K");
    }else if(strcmp(type, "ir") == 0){
        // memcpy(dashboard.ir_state, data, sizeof(data));
        printf("\033[7;1H");  // Move cursor to row 3, column 1
        printf("Distance Travelled: %s", data);
        printf("\033[K");
    }

    // updateDashboard();
}

bool updateDashboardTimerCallback(repeating_timer_t *rt){
    // updateDashboard();

    uint32_t elapsed_time_ms = to_ms_since_boot(get_absolute_time()) - dashboard.lastUpdateTime;
    uint32_t seconds = elapsed_time_ms / 1000;

    printf("\033[1;1H");
    printf("DASHBOARD (Updated %d seconds ago)", seconds);
    printf("\033[K");

    printf("\033[2;1H");
    printf("==================================");
    printf("\033[K");

    return true;
}

void updateDashboard(){

    uint32_t elapsed_time_ms = to_ms_since_boot(get_absolute_time()) - dashboard.lastUpdateTime;
    uint32_t seconds = elapsed_time_ms / 1000;

    // printf("\033[2J\033[H");
    printf("DASHBOARD (Updated %d seconds ago)\n", seconds);
    printf("==================================\n");
    printf("Car Direction: %s\n", dashboard.direction);
    printf("Car Speed: %s\n", dashboard.speed);
    printf("Distance Travelled: %s\n", dashboard.distance_travelled);
    printf("Ultrasonic Distance: %s\n", dashboard.ultrasonic_distance);
    printf("IR Sensor State: %s\n", dashboard.ir_state);
}