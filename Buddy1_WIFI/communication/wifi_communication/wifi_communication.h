#ifndef WIFI_COMMUNICATION_H

#define WIFI_COMMUNICATION_H

#include <stdbool.h>

#define WIFI_SSID "SimPhone"
#define WIFI_PASSWORD "a1234567"

#define TCP_PORT 4242

#define BUF_SIZE 2048


bool init_wifi();
bool init_wifi_with_ssid_password(const char *ssid, const char *password);

// FREERTOS FUNCTIONS
void set_ssid_password(char *ssid, char *password);
void checkWifiConnection();

// void freertos_init_wifi_tasks();
// void freertos_wifi_task(void *pvParameters);
#endif