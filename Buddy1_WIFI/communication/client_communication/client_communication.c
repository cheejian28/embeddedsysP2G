#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"


#include "client_communication.h"

#define TEST_ITERATIONS 10
#define POLL_TIME_S 5

typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    uint8_t buffer[BUF_SIZE];
    int buffer_len;
    int sent_len;
    bool complete;
    // int run_count;
    bool connected;
} TCP_CLIENT_T;


static TCP_CLIENT_T *state = NULL;

bool tcp_send_data(const char *data) {
    if (!state || !state->connected) {
        printf("[TCP Client] Failed to send Data. TCP client not initialized or not connected.\n");
        return false;
    }

    size_t data_len = strlen(data);
    
    // Begin critical section
    cyw43_arch_lwip_begin();
    
    // Write data to the TCP buffer
    err_t err = tcp_write(state->tcp_pcb, data, data_len, TCP_WRITE_FLAG_COPY);
    
    // Flush the TCP output buffer
    if (err == ERR_OK) {
        err = tcp_output(state->tcp_pcb);
    }
    
    // End critical section
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("[TCP Client] Failed to send data: %d\n", err);
        return false;
    }

    printf("[TCP Client] Data sent: %s\n", data);
    return true;
}

static err_t tcp_client_close(void *arg) {
    state = (TCP_CLIENT_T*)arg;
    err_t err = ERR_OK;

    state->complete = true;
    state->connected = false;

    if (state->tcp_pcb != NULL) {
        tcp_arg(state->tcp_pcb, NULL);
        tcp_poll(state->tcp_pcb, NULL, 0);
        tcp_sent(state->tcp_pcb, NULL);
        tcp_recv(state->tcp_pcb, NULL);
        tcp_err(state->tcp_pcb, NULL);
        err = tcp_close(state->tcp_pcb);
        if (err != ERR_OK) {
            printf("[TCP Client] close failed %d, calling abort\n", err);
            tcp_abort(state->tcp_pcb);
            err = ERR_ABRT;
        }
        state->tcp_pcb = NULL;
    }
    return err;
}

// Called with results of operation
// static err_t tcp_result(void *arg, int status) {
//     state = (TCP_CLIENT_T*)arg;
//     if (status == 0) {
//         printf("[TCP Client] test success\n");
//     } else {
//         printf("[TCP Client] test failed %d\n", status);
//     }
//     state->complete = true;
//     state->connected = false;
//     return tcp_client_close(arg);
// }

// static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
//     state = (TCP_CLIENT_T*)arg;
//     printf("[TCP Client] tcp_client_sent %u\n", len);
//     state->sent_len += len;

//     if (state->sent_len >= BUF_SIZE) {

//         state->run_count++;
//         if (state->run_count >= TEST_ITERATIONS) {
//             tcp_result(arg, 0);
//             return ERR_OK;
//         }

//         // We should receive a new buffer from the server
//         state->buffer_len = 0;
//         state->sent_len = 0;
//         printf("[TCP Client] Waiting for buffer from server\n");
//     }

//     return ERR_OK;
// }

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    state = (TCP_CLIENT_T*)arg;
    if (err != ERR_OK) {
        printf("[TCP Client] connect failed %d\n", err);
        return tcp_client_close(arg);
    }
    state->connected = true;

    printf("[TCP Client] Successfully connected to the server!\n");
    return ERR_OK;
}

static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
    state = (TCP_CLIENT_T *)arg;

    if (state->connected) {
        printf("[TCP Client] tcp_client_poll: Connection is still alive\n");
        return ERR_OK;
    } else {
        printf("[TCP Client] tcp_client_poll: Connection has been lost\n");
        return ERR_OK;
        // return tcp_result(arg, -1);
    }
}

static void tcp_client_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        printf("[TCP Client] tcp_client_err %d\n", err);
        tcp_client_close(arg);
    }
}

// err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
//     state = (TCP_CLIENT_T*)arg;
//     if (!p) {
//         printf("[TCP Client] Received empty p");
//         return tcp_result(arg, -1);
//     }
//     // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
//     // can use this method to cause an assertion in debug mode, if this method is called when
//     // cyw43_arch_lwip_begin IS needed
//     cyw43_arch_lwip_check();
//     if (p->tot_len > 0) {
//         // printf("recv %d err %d\n", p->tot_len, err);

//         for (struct pbuf *q = p; q != NULL; q = q->next) {
//             printf("[TCP Client] Received message: %.*s", q->len, (char *)q->payload);
//         }

//         // for (struct pbuf *q = p; q != NULL; q = q->next) {
//         //     DUMP_BYTES(q->payload, q->len);
//         // }
//         // Receive the buffer
//         const uint16_t buffer_left = BUF_SIZE - state->buffer_len;
//         state->buffer_len += pbuf_copy_partial(p, state->buffer + state->buffer_len,
//                                                p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
//         tcp_recved(tpcb, p->tot_len);
//     }
//     pbuf_free(p);

//     // If we have received the whole buffer, send it back to the server
//     if (state->buffer_len == BUF_SIZE) {
//         printf("[TCP Client] Writing %d bytes to server\n", state->buffer_len);
//         err_t err = tcp_write(tpcb, state->buffer, state->buffer_len, TCP_WRITE_FLAG_COPY);
//         if (err != ERR_OK) {
//             printf("[TCP Client] Failed to write data %d\n", err);
//             return tcp_result(arg, -1);
//         }
//     }
//     return ERR_OK;
// }

static bool tcp_client_open(void *arg) {
    state = (TCP_CLIENT_T*)arg;
    printf("[TCP Client] Connecting to %s port %u\n", ip4addr_ntoa(&state->remote_addr), TCP_PORT);
    state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr));
    if (!state->tcp_pcb) {
        printf("[TCP Client] failed to create pcb\n");
        return false;
    }

    tcp_arg(state->tcp_pcb, state);
    tcp_poll(state->tcp_pcb, tcp_client_poll, POLL_TIME_S * 2);
    // tcp_sent(state->tcp_pcb, tcp_client_sent);
    // tcp_recv(state->tcp_pcb, tcp_client_recv);
    tcp_err(state->tcp_pcb, tcp_client_err);

    state->buffer_len = 0;

    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, TCP_PORT, tcp_client_connected);
    cyw43_arch_lwip_end();

    // printf("Result from connection: %d\n", err);

    return err == ERR_OK;
}

// Perform initialisation
static TCP_CLIENT_T* tcp_client_init(const char *ip_address) {
    state = calloc(1, sizeof(TCP_CLIENT_T));
    if (!state) {
        printf("[TCP Client] failed to allocate state\n");
        return NULL;
    }
    ip4addr_aton(ip_address, &state->remote_addr);
    return state;
}

bool init_tcp_client_with_ip(const char *ip_address){

    if(state && state->connected){
        printf("[TCP Client] Failed to initialize TCP Client, It is already connected");
        return false;
    }

    state = tcp_client_init(ip_address);
    if (!state) {
        printf("[TCP Client] Failed to initialize TCP Client\n");
        return false;
    }

    if(!tcp_client_open(state)){
        printf("[TCP Client] Failed to connect to TCP Server\n");
        tcp_client_close(state);
        return false;
    }

    return true;
}


bool init_tcp_client(){
    return init_tcp_client_with_ip(IP_ADDRESS);
}

//=======================================================
//  FREERTOS TASK FOR TCP Client
//=======================================================

#ifndef configMINIMAL_STACK_SIZE
#define configMINIMAL_STACK_SIZE 128 // Or another appropriate size
#endif


char *ip_address = NULL;

void set_ip_address(char *ipaddr){
    ip_address = ipaddr;
}

void checkClientConnection(){
    if(!state || !state->connected){
        printf("[TCP Client TASK] Attempting to re-connect to Server..\n");
        init_tcp_client_with_ip(ip_address);
    }
}