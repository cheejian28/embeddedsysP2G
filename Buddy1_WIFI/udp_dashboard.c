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
    uint8_t speed;
    char direction[10];
    char ultrasonic_distance[1024];
} DASHBOARD_DATA;
DASHBOARD_DATA dashboard;
TaskHandle_t serverTaskHandle;

static char type[10];
static char data[1024];

int main(){
    stdio_init_all();

    sleep_ms(5000);

    set_ssid_password("SimPhone", "a1234567");
    xTaskCreate(vWifiTask, "Wifi Task", 256, NULL, 1, NULL);

    xTaskCreate(vServerTask, "TCP Server Task", 256, NULL, 1, &serverTaskHandle);

    struct repeating_timer timer;
    add_repeating_timer_ms(-1000, updateDashboardTimerCallback, NULL, &timer);

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
            vTaskSuspend(serverTaskHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void message_handler(const char *message){
    // printf("Main Program Received: %s\n", message);

    dashboard.lastUpdateTime = to_ms_since_boot(get_absolute_time());

    sscanf(message, "%s %s", type, data);
    // printf("Type: %s, Data: %s\n", type, data);

    if(strcmp(type, "us") == 0){
        memcpy(dashboard.ultrasonic_distance, data, sizeof(data));
    }

    // updateDashboard();
}

bool updateDashboardTimerCallback(repeating_timer_t *rt){
    updateDashboard();
    return true;
}

void updateDashboard(){

    uint32_t elapsed_time_ms = to_ms_since_boot(get_absolute_time()) - dashboard.lastUpdateTime;
    uint32_t seconds = elapsed_time_ms / 1000;

    printf("\033[2J\033[H");
    printf("DASHBOARD (Updated %d seconds ago)\n", seconds);
    printf("==================================\n");
    printf("Speed: %d\n", dashboard.speed);
    printf("Direction: %c\n", dashboard.direction);
    printf("Ultrasonic Distance: %s\n", dashboard.ultrasonic_distance);
}