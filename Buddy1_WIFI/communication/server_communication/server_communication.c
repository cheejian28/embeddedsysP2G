#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "server_communication.h"

#define TEST_ITERATIONS 10
#define POLL_TIME_S 5

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool complete;
    uint8_t buffer_sent[BUF_SIZE];
    uint8_t buffer_recv[BUF_SIZE];
    int sent_len;
    // int recv_len;
    // int run_count;
} TCP_SERVER_T;

static TCP_SERVER_T *state = NULL;
static message_callback_t message_callback;


static err_t tcp_server_close(void *arg) {
    state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;
    if (state->client_pcb != NULL) {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK) {
            printf("[TCP Server] close failed %d, calling abort\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
    printf("[TCP Server] TCP Connection has been closed!");
    return err;
}

static err_t tcp_server_result(void *arg, int status) {
    state = (TCP_SERVER_T*)arg;
    if (status == 0) {
        printf("[TCP Server] test success\n");
    } else {
        printf("[TCP Server] test failed %d\n", status);
    }
    state->complete = true;
    return tcp_server_close(arg);
}

// static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
//     state = (TCP_SERVER_T*)arg;
//     printf("[TCP Server] tcp_server_sent %u\n", len);
//     state->sent_len += len;

//     if (state->sent_len >= BUF_SIZE) {

//         // We should get the data back from the client
//         state->recv_len = 0;
//         printf("[TCP Server] Waiting for buffer from client\n");
//     }

//     return ERR_OK;
// }

err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb)
{
    state = (TCP_SERVER_T*)arg;
    for(int i=0; i< BUF_SIZE; i++) {
        state->buffer_sent[i] = rand();
    }

    state->sent_len = 0;
    printf("[TCP Server] Writing %ld bytes to client\n", BUF_SIZE);
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    err_t err = tcp_write(tpcb, state->buffer_sent, BUF_SIZE, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("[TCP Server] Failed to write data %d\n", err);
        return tcp_server_result(arg, -1);
    }
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    state = (TCP_SERVER_T*)arg;
    if (!p) {
        printf("[TCP Server] No message. Client might be disconnected.\n");
        return ERR_OK;
        // return tcp_server_result(arg, -1);
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();

    // Process the received data
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        // Create a buffer for the message
        char message[q->len + 1];  // +1 for null-termination
        memcpy(message, q->payload, q->len);
        message[q->len] = '\0';  // Null-terminate the string

        // Call the message callback function
        if (message_callback) {
            message_callback(message);
        }

        printf("[TCP Server] Received message: %s\n", message);
    }

    // Receive the buffer
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    return ERR_OK;
}

// static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
//     printf("[TCP Server] tcp_server_poll_fn\n");

//     return ERR_OK;
//     // return tcp_server_result(arg, -1); // no response is an error?
// }

static void tcp_server_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        printf("[TCP Server] tcp_client_err_fn %d\n", err);
        tcp_server_result(arg, err);
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        printf("[TCP Server] Failed to accept TCP connection\n");
        tcp_server_result(arg, err);
        return ERR_VAL;
    }
    printf("[TCP Server] Client connected\n");

    state->client_pcb = client_pcb;
    

    tcp_nagle_disable(client_pcb); // Disable nagle algorithm. Decrease latency slightly.

    tcp_arg(client_pcb, state);
    // tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    // tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
    // return tcp_server_send_data(arg, state->client_pcb);
}

static bool tcp_server_open(void *arg) {
    state = (TCP_SERVER_T*)arg;
    printf("[TCP Server] Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("[TCP Server] failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        printf("[TCP Server] failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        printf("[TCP Server] failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}

static TCP_SERVER_T* tcp_server_init() {
    state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        printf("[TCP Server] failed to allocate state\n");
        return NULL;
    }
    return state;
}

bool start_tcp_server(message_callback_t cb) {
    state = tcp_server_init();
    message_callback = cb;

    if (!state) {
        printf("[TCP Server] Failed to start server!\\n");
        return false;
    }
    if (!tcp_server_open(state)) {
        tcp_server_result(state, -1);
        printf("[TCP Server] Failed to start server\n");
        return false;
    }

    printf("[TCP Server] Sucessfully started server!\n");
    return true;
}