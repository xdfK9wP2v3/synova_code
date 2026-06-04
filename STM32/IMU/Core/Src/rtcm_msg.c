
#include "rtcm_msg.h"
#include "usart.h"
#include "imu_msg.h"
#include "tcp_server.h"
#include "lwip/tcp.h"

__attribute__((aligned(32))) RTCMmsg rtcm_msg_buffer[RTCM_MSG_BUF_SIZE];
uint32_t volatile next_rtcm_msg_idx = 0;

HAL_StatusTypeDef rtcm_transmit(uint8_t *msg_buff, uint8_t tcp_msg_type, uint32_t msg_len) {
  if (tcp_msg_type == RTK_RTCM_UART){
    SCB_CleanDCache_by_Addr((uint32_t *)msg_buff, 1024);
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(RTK_RTCM_HDL, msg_buff, msg_len);
    if (status != HAL_OK){
      dbg_msg_len = sprintf(dbg_msg_buff, "[RTCM-UART] DMA Failed once (errno %d)\r\n", status);
      dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
      return HAL_BUSY;
    }
    rtcm_uart_status = RTCM_BUSY;
  } else if (tcp_msg_type == RTK_RTCM_TCP){
    uint8_t pcb_idx = TCP_PORT_RTCM - TCP_PORT_START;
    if (eth_pcb[pcb_idx] && eth_pcb[pcb_idx]->local_port == TCP_PORT_RTCM && eth_pcb[pcb_idx]->state == ESTABLISHED) {
      if (eth_pcb[pcb_idx]->snd_buf > msg_len) {
        err_t write_status = tcp_write(eth_pcb[pcb_idx],
                                       msg_buff,
                                       msg_len,
                                       TCP_WRITE_FLAG_COPY);

        if (write_status != ERR_OK && write_status != ERR_CONN) {
          dbg_msg_len = sprintf(dbg_msg_buff, "[Port %u] TCP write errno: %2d, link status: %u\r\n",
                                eth_pcb[pcb_idx]->local_port,
                                write_status,
                                eth_pcb[pcb_idx]->state);
          dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
          return HAL_ERROR;
        }
        tcp_output(eth_pcb[pcb_idx]);
      } else {
//        dbg_msg_len = sprintf(dbg_msg_buff, "[Port %u] TCP snd_buf: %2d, msg_len: %lu\r\n",
//                              eth_pcb[pcb_idx]->local_port,
//                              eth_pcb[pcb_idx]->snd_buf,
//                              msg_len);
//        dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
        return HAL_BUSY;
      }
    } else if (eth_pcb[pcb_idx] && eth_pcb[pcb_idx]->state == LISTEN) {
      return HAL_OK;
    }
  }
  return HAL_OK;
}


uint32_t get_rtcm_msg(uint8_t *const buf, RTCMmsg *msg, uint32_t *const type) {
  *type = msg->type;
  memcpy(buf, (void *) msg->data.rtk_rtcm.rtcm, msg->data.rtk_rtcm.length);
  return msg->data.rtk_rtcm.length;
}
