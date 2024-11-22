#include <stdio.h>
#include "pico/stdlib.h"

#include "wifi_communication.h"
#include "udp_client_communication.h"

int main(){
    stdio_init_all();

    sleep_ms(5000);
    printf("Starting Remote..\n");

    init_wifi_with_ssid_password("SimPhone", "a1234567");
    init_udp_client("172.20.10.2", 42069);

    int counter = 0;
    char data[50];

    while(1){
        counter++;

        snprintf(data, sizeof(data), "Sending data number: %d", counter);
        send_udp_data(data);

        sleep_ms(10);
    }
}