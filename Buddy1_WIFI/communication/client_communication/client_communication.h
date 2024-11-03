#ifndef CLIENT_COMMUNICATION_H

#define CLIENT_COMMUNICATION_H

#include <stdbool.h>

#define IP_ADDRESS "172.20.10.13"

#define TCP_PORT 4242
#define BUF_SIZE 2048

bool init_tcp_client();
bool init_tcp_client_with_ip(const char *ip_address);
bool tcp_send_data(const char *data);

// void freertos_tcp_client_task(void *pvParameters);
#endif