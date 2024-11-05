#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "wifi_communication.h"

static bool wifi_connect(const char *ssid, const char *password){
    static bool initialized = false;

    if(!initialized && cyw43_arch_init()){
        printf("Failed to initialize\n");
        return false;
    }
    initialized = true;

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WIFI..\n");
    if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 5000)) {
        printf("Failed to connect to WIFI using %s, %s\n", ssid, password);
        return false;
    } 

    printf("Connected to WIFI!\n");
    return true;
}


bool init_wifi_with_ssid_password(const char *ssid, const char *password){
    if(wifi_connect(ssid, password)){
        printf("Successfully Initialised WIFI!\n");
        return true;
    }else{
        printf("Failed to initialise WIFI!\n");
        return false;
    }
}

bool init_wifi(){
    return init_wifi_with_ssid_password(WIFI_SSID, WIFI_PASSWORD);
}

//=======================================================
//  FREERTOS TASK FOR WIFI
//=======================================================

char *w_ssid = NULL;
char *w_password = NULL;

void set_ssid_password(char *s, char *p){
    w_ssid = s;
    w_password = p;
}


void checkWifiConnection(){
    printf("\nHere: %u\n", cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP);
    if(cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP){
        printf("[WIFI TASK] Attempting to re-connect to WIFI..\n");

        if(w_ssid && w_password && wifi_connect(w_ssid, w_password)){
            printf("[WIFI TASK] Connected to WIFI using %s, %s!\n", w_ssid, w_password);
        }else{
            printf("[WIFI TASK] Failed to connect to WIFI, retrying in 5 seconds\n");
        }
    }
}