#include <stdio.h>
#include <pico/stdlib.h>
#include "wifi_communication.h"
#include "client_communication.h"
#include "server_communication.h"

int main(){
    stdio_init_all();

    sleep_ms(5000);

    init_wifi();
    init_tcp_client();


    while(1){
        sleep_ms(5000);
        tcp_send_data("Sending Data");
    }
}