
#include "tcp_server.h"

#include <stdio.h>
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/init.h"
#include "lwip/pbuf.h"
#include "netif/etharp.h"
#include "usart.h"

#include "error.h"

struct tcp_pcb *eth_pcb[TCP_PORT_NUM] = {NULL};

HAL_StatusTypeDef tcp_transmit(char *msg_buff, uint32_t msg_len) {
    uint32_t i = TCP_PORT_START - TCP_PORT_START;

    if (eth_pcb[i] == NULL || eth_pcb[i]->local_port != (i + TCP_PORT_START))
        return HAL_ERROR;

    if (eth_pcb[i]->snd_buf < msg_len)
        return HAL_BUSY;

    if (eth_pcb[i]->state == ESTABLISHED) {
        err_t write_status = tcp_write(eth_pcb[i],
                                       msg_buff,
                                       msg_len,
                                       TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE);

        if (write_status != ERR_OK && write_status != ERR_CONN)
            return HAL_ERROR;

        tcp_output(eth_pcb[i]);
    }

    return HAL_OK;
}

static err_t tcp_recv_pcb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p != NULL) {
        tcp_recved(tpcb, p->tot_len);
        static uint16_t tcp_cmd_parsed_len = 0;
        static ParserState tcp_cmd_parser_state = 0;
        static uint8_t tcp_cmd_parser_buffer[HOST_CMD_STR_MAX_LEN];

        normal_dma_parser(p->payload, p->tot_len,
                          tcp_cmd_parser_buffer, sizeof(tcp_cmd_parser_buffer), &tcp_cmd_parser_state, &tcp_cmd_parsed_len,
                          config_command_parser);
        pbuf_free(p);
    }
    else if (err == ERR_OK) {
        eth_pcb[tpcb->local_port - TCP_PORT_START] = NULL;
        while (tcp_close(tpcb) != ERR_OK);
    }
    return ERR_OK;
}

static err_t tcp_accept_pcb(void *arg, struct tcp_pcb *apcb, err_t err) {
    eth_pcb[apcb->local_port - TCP_PORT_START] = apcb;
    tcp_recv(apcb, tcp_recv_pcb);
    return ERR_OK;
}

void TCP_Server_Init(void) {
    for (uint32_t i = 0; i < TCP_PORT_NUM; i++) {
        eth_pcb[i] = tcp_new();
        if (tcp_bind(eth_pcb[i], IP_ADDR_ANY, TCP_PORT_START + i) != ERR_OK)
            error_handler(ERROR_FLAG_ETH);
        eth_pcb[i] = tcp_listen(eth_pcb[i]);
        tcp_accept(eth_pcb[i], tcp_accept_pcb);
    }
}

void TCP_Server_Recovery(void) {
    for (uint32_t i = 0; i < TCP_PORT_NUM; i++) {
        eth_pcb[i] = tcp_listen(eth_pcb[i]);
        tcp_accept(eth_pcb[i], tcp_accept_pcb);
    }
}

void TCP_Server_Abort(void) {
    for (uint32_t i = 0; i < TCP_PORT_NUM; i++)
        if (eth_pcb[i] != NULL && eth_pcb[i]->state != LISTEN)
            tcp_abort(eth_pcb[i]);
}