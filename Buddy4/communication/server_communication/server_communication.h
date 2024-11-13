#ifndef SERVER_COMMUNICATION_H

#define SERVER_COMMUNICATION_H

#include <stdbool.h>

#define TCP_PORT 4242
#define BUF_SIZE 2048

typedef void (*message_callback_t)(const char* message);

// bool start_tcp_server();
bool start_tcp_server(message_callback_t cb);


// FREERTOS FUNCTIONS
void set_callback(message_callback_t cb);
void checkServerConnection();

// void freertos_init_tcp_server_tasks();
// void freertos_tcp_server_task(void *pvParameters);

#endif