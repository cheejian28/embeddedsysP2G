#include <stdio.h>
#include <pico/stdlib.h>
#include "wifi_communication.h"
#include "client_communication.h"
#include "server_communication.h"

void message_handler(const char *message);

int main(){
    stdio_init_all();

    sleep_ms(5000);

    init_wifi();
    start_tcp_server(message_handler);

    while(1){
    }
}

void message_handler(const char *message){
    printf("Main Program Received: %s\n", message);
}