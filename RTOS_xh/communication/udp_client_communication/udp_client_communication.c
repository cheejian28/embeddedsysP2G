#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <string.h>
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "udp_client_communication.h"

struct udp_pcb *pcb;
static ip_addr_t dest_ip;
static int dest_port;

bool init_udp_client(const char *ip_address, const int port){
    pcb = udp_new();
    if (!pcb) {
        printf("[UDP Client] Failed to create UDP PCB\n");
        return false;
    }

    ip4addr_aton(ip_address, &dest_ip);
    dest_port = port;

    printf("[UDP Client] Successfully initialized UDP Client!\n");
    return true;
}

bool send_udp_data(const char *data){
    printf("Sending %s\n", data);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, strlen(data), PBUF_RAM);
    if (p) {
        memcpy(p->payload, data, strlen(data));
        printf("P VALUE: %s\n", p->payload);
        udp_sendto(pcb, p, &dest_ip, dest_port);
        pbuf_free(p);
    }
    return true;
}