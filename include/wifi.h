#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h> 

bool connect_to_wifi(const char *ssid, const char *password, char *text_buffer);
bool is_connected();
void init_wifi(char *text_buffer);
void restart_wifi();
#endif