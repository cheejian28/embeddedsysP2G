#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "wifi_communication.h"
#include "server_communication.h"

// Function prototypes
void vWifiTask(void *pvParameters);
void vClientTask(void *pvParameters);
void vSendDataTask(void *pvParameters);

void message_handler(const char *message);

int main() {
    // Initialize the Pico standard library
    stdio_init_all();

    sleep_ms(5000);

    set_ssid_password("SimPhone", "a1234567");
    xTaskCreate(vWifiTask, "Wifi Task", 256, NULL, 1, NULL);

    // set_ip_address("172.20.10.3");
    set_callback(message_handler);
    xTaskCreate(vClientTask, "TCP Client Task", 256, NULL, 1, NULL);

    // xTaskCreate(vSendDataTask, "Sending Task", 256, NULL, 1, NULL);
    // init_wifi_with_ssid_password("SimPhone", "a1234567");
    // init_tcp_client_with_ip("172.20.10.3");
    // tcp_send_data("Test data");

    // while(1);

    // Start the scheduler
    vTaskStartScheduler();

    // We should never reach here as the scheduler will now be running
    while (1);
}

void vWifiTask(void *pvParameters){
    while(1){
        printf("Checking Wifi Connection\n");
        checkWifiConnection();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vClientTask(void *pvParameters){
    while(1){
        printf("Checking TCP Connection to Server\n");
        checkServerConnection();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vSendDataTask(void *pvParameters){
    while(1){
        printf("Sending Data\n");
        tcp_send_data("Testing");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void message_handler(const char *message){
    printf("Main Program Received: %s\n", message);
}