#include <stdio.h>
#include <pico/stdlib.h>
#include "wifi_communication.h"
#include "client_communication.h"
#include "server_communication.h"

#define FORWARD_BUTTON_PIN 22
#define RIGHT_BUTTON_PIN 21
#define LEFT_BUTTON_PIN 20


int main(){
    stdio_init_all();

    sleep_ms(5000);

    // init_wifi();
    // init_tcp_client_with_ip("172.20.10.3");

    set_ssid_password("SimPhone", "a1234567");
    checkWifiConnection();

    set_ip_address("172.20.10.3");
    checkClientConnection();


    while(1){
        if(!gpio_get(FORWARD_BUTTON_PIN)){
            tcp_send_data("forward\n");
        }else if(!gpio_get(LEFT_BUTTON_PIN)){
            tcp_send_data("left\n");
        }else if(!gpio_get(RIGHT_BUTTON_PIN)){
            tcp_send_data("right\n");
        }

        checkWifiConnection();
        checkClientConnection();

        sleep_ms(100);
    }


    // while(1){
    //     sleep_ms(5000);
    //     tcp_send_data("Sending Data");
    // }
}