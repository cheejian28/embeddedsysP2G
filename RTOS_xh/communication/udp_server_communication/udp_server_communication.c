#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <string.h>
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "udp_server_communication.h"

static struct udp_pcb *recv_pcb;
static message_callback_t callback;

void printNetworkInfo(const int port){
    struct netif *netif = netif_list; // Start with the first network interface
    while (netif) {
        if (netif_is_up(netif)) { // Check if the network interface is up
            char ip_str[16];
            ipaddr_ntoa_r(&netif->ip_addr, ip_str, sizeof(ip_str));
            printf("[UDP Server] Server started on %s:%d\n", ip_str, port);
            break; // Only print for the first active interface
        }
        netif = netif->next;
    }
}

void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        printf("Received: %s\n", (char *)p->payload);

        // Forward data to main function
        if(callback != NULL) callback(p->payload);

        pbuf_free(p);
    }
}

bool init_udp_server(const int port, message_callback_t cb){
    
    bool isWifiConnected = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
    if(!isWifiConnected){
        printf("[UDP Server] Failed to start UDP Server, wifi is not connected!\n");
        return false;
    }

    recv_pcb = udp_new();
    if (!recv_pcb) {
        printf("[UDP Server] Failed to create UDP PCB\n");
        return false;
    }

    udp_bind(recv_pcb, IP_ADDR_ANY, port);
    udp_recv(recv_pcb, udp_recv_callback, NULL);

    callback = cb;

    printNetworkInfo(port);

    return true;
}