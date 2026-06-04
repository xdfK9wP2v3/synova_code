

#include "tcp_server.h"

#include <stdio.h>
#include "usart.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/init.h"
#include "lwip/pbuf.h"
#include "netif/etharp.h"
#include "imu_msg.h"
#include "rtcm_msg.h"
#include "rtk.h"

struct tcp_pcb *eth_pcb[TCP_PORT_NUM] = {NULL};

char tcp_debug_msg[256];
uint32_t len;

static err_t tcp_recv_pcb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
  if (p != NULL) {
    tcp_recved(tpcb, p->tot_len);
    if (tpcb->local_port == TCP_PORT_RTCM) {
      if (rtk_config.rtk_type == RTK_MOB){
        record_rtk_rtcm_uart((uint8_t *)p->payload, p->tot_len);
      }
    } else {
      static uint16_t tcp_cmd_parsed_len = 0;
      static ParserState tcp_cmd_parser_state = 0;
      static uint8_t tcp_cmd_parser_buffer[HOST_CMD_STR_MAX_LEN];

      normal_dma_parser(p->payload, p->tot_len,
                        tcp_cmd_parser_buffer, sizeof(tcp_cmd_parser_buffer), &tcp_cmd_parser_state, &tcp_cmd_parsed_len,
                        config_command_parser);
    }
    memset(p->payload, 0, p->tot_len);
    pbuf_free(p);
  } else if (err == ERR_OK) {
    len = sprintf(tcp_debug_msg, "[TCP] port %d closed by client\r\n", tpcb->local_port);
    dbg_msg_transmit(tcp_debug_msg, len);
    eth_pcb[tpcb->local_port - TCP_PORT_START] = NULL;
    while (tcp_close(tpcb) != ERR_OK);
  }
  return ERR_OK;
}


static err_t tcp_accept_pcb(void *arg, struct tcp_pcb *apcb, err_t err) {
  eth_pcb[apcb->local_port - TCP_PORT_START] = apcb;
  len = sprintf(tcp_debug_msg, "[TCP] port %d connected\r\n", apcb->local_port);
  dbg_msg_transmit(tcp_debug_msg, len);
  tcp_recv(apcb, tcp_recv_pcb);
  return ERR_OK;
}


void TCP_Server_Init(void) {

#if (TCP_PORT_NUM > MEMP_NUM_TCP_PCB)
  len = sprintf(tcp_debug_msg, "[TCP] No enough PCB: TCP_PORT_NUM %d > MEMP_NUM_TCP_PCB %d\r\n", TCP_PORT_NUM, MEMP_NUM_TCP_PCB);
  msg_transmit(tcp_debug_msg, len);
  while (1);
#endif

  for (uint32_t i = 0; i < TCP_PORT_NUM; i++) {
    eth_pcb[i] = tcp_new();
    err_t tcp_status = tcp_bind(eth_pcb[i], IP_ADDR_ANY, TCP_PORT_START + i);
    if (tcp_status != ERR_OK) {
      error_handler(ERROR_FLAG_ETH);
    }
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
  for (uint32_t i = 0; i < TCP_PORT_NUM; i++) {
    if (eth_pcb[i] != NULL && eth_pcb[i]->state != LISTEN) {
      tcp_abort(eth_pcb[i]);
    }
  }
}