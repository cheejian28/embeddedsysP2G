#ifndef UDP_CLIENT_COMMUNICATION_H

#define UDP_CLIENT_COMMUNICATION_H

bool init_udp_client(const char *ip_address, const int port);
bool send_udp_data(const char *data);

#endif