

#ifndef IMU_RTK_H
#define IMU_RTK_H

#include "stdint.h"

#define RTK_BS 0
#define RTK_MOB 1

extern __attribute__((aligned(32))) uint8_t rtk_nmea_data_bytes[256];
extern __attribute__((aligned(32))) uint8_t rtk_rtcm_data_bytes[1024];

void RTK_Init(void);
void RTK_NMEA_DMA(void);
void RTK_RTCM_RX_DMA(void);

#endif //IMU_RTK_H
