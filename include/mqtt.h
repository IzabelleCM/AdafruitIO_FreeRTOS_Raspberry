#ifndef MQTT_H
#define MQTT_H
#include <stdbool.h> 

void mqtt_setup(const char *client_id, const char *broker_ip, const char *user, const char *pass, char *text_buffer);
void mqtt_comm_publish(const char *topic, const uint8_t *data, size_t len);
bool mqtt_is_connected(void);

#endif
