

#ifndef DIGITAL_PLL_TCP_SERVER_H
#define DIGITAL_PLL_TCP_SERVER_H

#include "stdint.h"
#include "main.h"

#define TCP_PORT_START 6000

#define TCP_PORT_NUM (TCP_PORT_START - TCP_PORT_START + 1)


extern struct tcp_pcb* eth_pcb[TCP_PORT_NUM];
void TCP_Server_Init(void);
void TCP_Server_Recovery(void);
void TCP_Server_Abort(void);

HAL_StatusTypeDef tcp_transmit(char *, uint32_t);

#endif //DIGITAL_PLL_TCP_SERVER_H
