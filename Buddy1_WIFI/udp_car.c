#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "wifi_communication.h"
#include "udp_server_communication.h"

TaskHandle_t serverTaskHandle;

void message_handler(const char *message){
    printf("Received Message: %s\n", message);
}

void vWifiTask(void *pvParameters){
    while(1){
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

int main(){
    stdio_init_all();

    sleep_ms(5000);
    printf("Starting Car..\n");

    set_ssid_password("SimPhone", "a1234567");
    xTaskCreate(vWifiTask, "Wifi Task", 256, NULL, 1, NULL);

    xTaskCreate(vServerTask, "UDP Server Task", 256, NULL, 1, &serverTaskHandle);

    // Start the scheduler
    vTaskStartScheduler();
    while(1);
}