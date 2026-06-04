#ifndef IMU_ERROR_H
#define IMU_ERROR_H

#include "stdint.h"

#define ERROR_FLAG_NMI (1U << 0)
#define ERROR_FLAG_HardFault (1U << 1)
#define ERROR_FLAG_MemManage (1U << 2)
#define ERROR_FLAG_BusFault (1U << 3)
#define ERROR_FLAG_UsageFault (1U << 4)
#define ERROR_FLAG_HAL_ERROR (1U << 5)
#define ERROR_FLAG_PCLK_ERROR (1U << 6)
#define ERROR_FLAG_MSG_OVERTIME (1U << 7)
#define ERROR_FLAG_ADIS_INIT_FAIL (1U << 8)
#define ERROR_FLAG_MAG_INIT_FAIL (1U << 9)
#define ERROR_FLAG_LPS28_INIT_FAIL (1U << 10)
#define ERROR_FLAG_LPS22_INIT_FAIL (1U << 11)
#define ERROR_FLAG_RTK_INIT_FAIL (1U << 12)
#define ERROR_FLAG_RTK_HW_FAIL (1U << 13)

#define ERROR_FLAG_ETH (1U << 14)
#define ERROR_FLAG_SPI_FAIL (1U << 15)
#define ERROR_FLAG_I2C_FAIL (1U << 16)
#define ERROR_FLAG_UART_NMEA_FAIL (1U << 17)
#define ERROR_FLAG_UART_RTCM_FAIL (1U << 18)

#define ERROR_FLAG_UNKNOWN_MSG (1U << 19)
#define ERROR_FLAG_FLASH (1U << 20)

#define ERROR_FLAG_BORRST (1U << 29)
#define ERROR_FLAG_IWDG (1U << 30)

void shutdown_mode();

void reset_system();

void check_error();

void error_handler(uint32_t error_flag);

#endif //IMU_ERROR_H
