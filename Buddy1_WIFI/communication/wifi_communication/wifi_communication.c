#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"


#include "wifi_communication.h"


void check_connections(void);
static void handle_wifi_reconnect(void);

static bool wifi_connected = false;


static bool wifi_connect(const char *ssid, const char *password){
    if(cyw43_arch_init()){
        printf("Failed to initialize\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    printf("Connecting to WIFI..\n");
    if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to WIFI\n");
        return false;
    } 

    printf("Connected to WIFI!\n");
    return true;
}

bool init_wifi(){
    if(wifi_connect(WIFI_SSID, WIFI_PASSWORD)){
        printf("Successfully Initialised WIFI!\n");
        return true;
    }else{
        printf("Failed to initialise WIFI!\n");
        return false;
    }
}


//=======================================================
//  CODE TO CHECK FOR DISCONNECTS AND HANDLE RECONNECT
//=======================================================

void run_connection_monitor() {
    while (true) {
        if (!cyw43_arch_wifi_is_connected()) {
            wifi_connected = false;
            handle_wifi_reconnect();
        }
        sleep_ms(1000); // Adjust as needed
    }
}

static void handle_wifi_reconnect() {
    while (!wifi_connected) {
        printf("Attempting to reconnect to Wi-Fi...\n");
        wifi_connected = wifi_connect(WIFI_SSID, WIFI_PASSWORD);
        if (wifi_connected) {
            printf("Reconnected to Wi-Fi!\n");
        } else {
            printf("Failed to connect to WIFI, retrying in 5 seconds");
            sleep_ms(5000);
        }
    }
}

//=======================================================
//  FREERTOS TASK FOR WIFI
//=======================================================

#ifndef configMINIMAL_STACK_SIZE
#define configMINIMAL_STACK_SIZE 128 // Or another appropriate size
#endif

void freertos_init_wifi_tasks() {
    xTaskCreate(freertos_wifi_task, "WiFi Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void freertos_wifi_task(void *pvParameters){
    while(1){
        if(!cyw43_arch_wifi_is_connected()){
            printf("[WIFI TASK] Attempting to re-connect to WIFI..");
            if(wifi_connect(WIFI_SSID, WIFI_PASSWORD)){
                printf("[WIFI TASK] Connected to WIFI!");
            }else{
                printf("[WIFI TASK] Failed to connect to WIFI, retrying in 5 seconds");
            }
        }
        vTaskDelay(pdMs_TO_TICKS(5000));
    }
}