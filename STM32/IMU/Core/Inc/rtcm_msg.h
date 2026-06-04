

#ifndef RTCM_MSG_H
#define RTCM_MSG_H

#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "help_macro.h"
#include "string.h"
#include "tcp_server.h"


typedef struct {
    enum {
        RTK_RTCM_TCP = 6,
        RTK_RTCM_UART = 7,
    } type;

    union {
        struct {
            uint8_t rtcm[1024];
            uint32_t length;
        } rtk_rtcm;
    } data;
} RTCMmsg;


#define RTCM_MSG_BUF_SIZE 256
__attribute__((aligned(32))) extern RTCMmsg rtcm_msg_buffer[RTCM_MSG_BUF_SIZE];
extern volatile uint32_t next_rtcm_msg_idx;

FORCE_INLINE volatile RTCMmsg *get_new_rtcm_msg() {
  uint32_t curr_event_msg_idx;
  ATOMIC_CYCLE_INCREASE(next_rtcm_msg_idx, curr_event_msg_idx, RTCM_MSG_BUF_SIZE);

  volatile RTCMmsg *msg = &rtcm_msg_buffer[curr_event_msg_idx];
  return msg;
}

FORCE_INLINE void record_rtk_rtcm_uart(const uint8_t *rtcm,
                                  const uint32_t length) {
  volatile RTCMmsg *msg = get_new_rtcm_msg();
  msg->type = RTK_RTCM_UART;
  msg->data.rtk_rtcm.length = length;
  memcpy((void *)msg->data.rtk_rtcm.rtcm, rtcm, length);
}


FORCE_INLINE void record_rtk_rtcm_tcp(const uint8_t *rtcm,
                                   const uint32_t length) {
  volatile RTCMmsg *msg = get_new_rtcm_msg();
  msg->type = RTK_RTCM_TCP;
  msg->data.rtk_rtcm.length = length;
  memcpy((void *) msg->data.rtk_rtcm.rtcm, rtcm, length);
}


uint32_t get_rtcm_msg(uint8_t *, RTCMmsg *, uint32_t *);
HAL_StatusTypeDef rtcm_transmit(uint8_t *, uint8_t, uint32_t);

#endif //RTCM_MSG_H
