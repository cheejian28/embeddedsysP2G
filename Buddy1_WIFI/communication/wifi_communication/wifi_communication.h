#ifndef WIFI_COMMUNICATION_H

#define WIFI_COMMUNICATION_H

#include <stdbool.h>

#define WIFI_SSID "SimPhone"
#define WIFI_PASSWORD "a1234567"

#define CAR_IP_ADDRESS "172.20.10.9"
#define TCP_PORT 4242

#define BUF_SIZE 2048


bool init_wifi();

void freertos_init_wifi_tasks();
void freertos_wifi_task(void *pvParameters);
#endif