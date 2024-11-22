#ifndef UDP_SERVER_COMMUNICATION_H

#define UDP_SERVER_COMMUNICATION_H

typedef void (*message_callback_t)(const char* message);

bool init_udp_server(const int port, message_callback_t cb);

#endif