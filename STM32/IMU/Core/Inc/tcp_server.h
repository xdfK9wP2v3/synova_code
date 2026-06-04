

#ifndef LWIP_TEST_TCP_SERVER_H
#define LWIP_TEST_TCP_SERVER_H

#define TCP_PORT_START 5000

#define TCP_PORT_NMEA TCP_PORT_START
#define TCP_PORT_ADIS (TCP_PORT_START+1)
#define TCP_PORT_MAG (TCP_PORT_START+2)
#define TCP_PORT_LPS28 (TCP_PORT_START+3)
#define TCP_PORT_LPS22 (TCP_PORT_START+4)
#define TCP_PORT_RTCM (TCP_PORT_START+5)

#define TCP_PORT_NUM (TCP_PORT_RTCM - TCP_PORT_START + 1)


extern struct tcp_pcb* eth_pcb[TCP_PORT_NUM];
void TCP_Server_Init(void);
void TCP_Server_Recovery(void);
void TCP_Server_Abort(void);

#endif //LWIP_TEST_TCP_SERVER_H




