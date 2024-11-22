#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "wifi_communication.h"
#include "server_communication.h"

void vWifiTask(void *pvParameters);
void vServerTask(void *pvParameters);
void message_handler(const char *message);
bool updateDashboardTimerCallback(repeating_timer_t *rt);
void updateDashboard();

typedef struct {
    uint32_t lastUpdateTime;
    uint8_t speed;
    char direction [10];
} DASHBOARD_DATA;
DASHBOARD_DATA dashboard;

int main(){
    stdio_init_all();

    sleep_ms(5000);

    set_ssid_password("SimPhone", "a1234567");
    xTaskCreate(vWifiTask, "Wifi Task", 256, NULL, 1, NULL);

    set_callback(message_handler);
    xTaskCreate(vServerTask, "TCP Server Task", 256, NULL, 1, NULL);

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
    while(1){
        // printf("Checking TCP Connection to Server\n");
        checkServerConnection();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


void message_handler(const char *message){
    printf("Main Program Received: %s\n", message);

    dashboard.lastUpdateTime = to_ms_since_boot(get_absolute_time());
    updateDashboard();
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
}