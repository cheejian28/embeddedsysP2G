#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "wifi_communication.h"
#include "client_communication.h"

// Function prototypes
void vWifiTask(void *pvParameters);
void vClientTask(void *pvParameters);
void vSendDataTask(void *pvParameters);

int main() {
    // Initialize the Pico standard library
    stdio_init_all();

    sleep_ms(5000);

    set_ssid_password("SimPhone", "a1234567");
    xTaskCreate(vWifiTask, "Wifi Task", 256, NULL, 1, NULL);

    set_ip_address("172.20.10.4");
    xTaskCreate(vClientTask, "TCP Client Task", 256, NULL, 1, NULL);

    xTaskCreate(vSendDataTask, "Send Data Task", 256, NULL, 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();

    // We should never reach here as the scheduler will now be running
    while (1);
}

void vWifiTask(void *pvParameters){
    while(1){
        checkWifiConnection();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vClientTask(void *pvParameters){
    while(1){
        checkClientConnection();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vSendDataTask(void *pvParameters){
    while(1){
        printf("Sending Data\n");
        tcp_send_data("Sending Testing Data");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}