

#ifndef IMU_ADIS_H
#define IMU_ADIS_H

#include "stm32h7xx.h"

#define SPI_BUFFER_SIZE 32

// configuration
#define ADIS_FILT_CTRL_VALUE 0x0  // 0b000 to 0b111

#define ADIS_MSC_G_COMP (1 << 7) // 1:enable, 0: disable
#define ADIS_MSC_PERCUSS (1 << 6) // 1:enable, 0: disable
//#define ADIS_MSC_SYNC (0b001 << 2) // 000b: internal, 001b: direct, 010b: scaled, 011b: output, 101b: pulse
#define ADIS_MSC_SYNC_POLARITY (0b1 << 1) // 1: rising trigger, 0: falling trigger
#define ADIS_MSC_DR_POLARITY (0b0) // 1: high, 0: low

#define ADIS_MSC_CTRL_VALUE (ADIS_MSC_G_COMP | ADIS_MSC_PERCUSS | ADIS_MSC_SYNC_POLARITY | ADIS_MSC_DR_POLARITY)

#define ADIS_UP_SCALE_VALUE 0x07D0

#define ADIS_DEC_RATE_VALUE 0x0000  // maximum = 1999

#define ADIS_NULL_CNFG_VALUE 0x070A

#define ADIS_RESET_VALUE 0x0080

#define ADIS_SELF_TEST_VALUE 0x0004


// Register start address
#define ADIS_REG_DIAG_STAT 0x02

#define ADIS_REG_X_GYRO_LOW 0x04
#define ADIS_REG_X_GYRO_OUT 0x06
#define ADIS_REG_Y_GYRO_LOW 0x08
#define ADIS_REG_Y_GYRO_OUT 0x0A
#define ADIS_REG_Z_GYRO_LOW 0x0C
#define ADIS_REG_Z_GYRO_OUT 0x0E

#define ADIS_REG_X_ACCL_LOW 0x10
#define ADIS_REG_X_ACCL_OUT 0x12
#define ADIS_REG_Y_ACCL_LOW 0x14
#define ADIS_REG_Y_ACCL_OUT 0x16
#define ADIS_REG_Z_ACCL_LOW 0x18
#define ADIS_REG_Z_ACCL_OUT 0x1A

#define ADIS_REG_TEMP_OUT 0x1C

#define ADIS_REG_TIME_STAMP 0x1E

#define ADIS_REG_DATA_CNTR 0x22

#define ADIS_REG_X_DELTANG_LOW 0x24
#define ADIS_REG_X_DELTANG_OUT 0x26
#define ADIS_REG_Y_DELTANG_LOW 0x28
#define ADIS_REG_Y_DELTANG_OUT 0x2A
#define ADIS_REG_Z_DELTANG_LOW 0x2C
#define ADIS_REG_Z_DELTANG_OUT 0x2E

#define ADIS_REG_X_DELTVEL_LOW 0x30
#define ADIS_REG_X_DELTVEL_OUT 0x32
#define ADIS_REG_Y_DELTVEL_LOW 0x34
#define ADIS_REG_Y_DELTVEL_OUT 0x36
#define ADIS_REG_Z_DELTVEL_LOW 0x38
#define ADIS_REG_Z_DELTVEL_OUT 0x3A

#define ADIS_REG_XG_BIAS_LOW 0x40
#define ADIS_REG_XG_BIAS_HIGH 0x42
#define ADIS_REG_YG_BIAS_LOW 0x44
#define ADIS_REG_YG_BIAS_HIGH 0x46
#define ADIS_REG_ZG_BIAS_LOW 0x48
#define ADIS_REG_ZG_BIAS_HIGH 0x4A

#define ADIS_REG_XA_BIAS_LOW 0x4C
#define ADIS_REG_XA_BIAS_HIGH 0x4E
#define ADIS_REG_YA_BIAS_LOW 0x50
#define ADIS_REG_YA_BIAS_HIGH 0x52
#define ADIS_REG_ZA_BIAS_LOW 0x54
#define ADIS_REG_ZA_BIAS_HIGH 0x56

#define ADIS_REG_FILT_CTRL 0x5C

#define ADIS_REG_MSC_CTRL 0x60

#define ADIS_REG_UP_SCALE 0x62

#define ADIS_REG_DEC_RATE 0x64

#define ADIS_REG_NULL_CNFG 0x66

#define ADIS_REG_GLOB_CMD 0x68

#define ADIS_REG_FIRM_REV 0x6C
#define ADIS_REG_FIRM_DM 0x6E
#define ADIS_REG_FIRM_Y 0x70

#define ADIS_REG_PROD_ID 0x72

#define ADIS_REG_SERIAL_NUM 0x74

#define ADIS_REG_USER_SCR1 0x76
#define ADIS_REG_USER_SCR2 0x78
#define ADIS_REG_USER_SCR3 0x7A

#define ADIS_REG_FLSHCNT_LOW 0x7C
#define ADIS_REG_FLSHCNT_HIGH 0x7E


// Variables
extern uint16_t spi_rev_buff[];

// Functions
uint32_t hex_to_hexstr(char*, const uint16_t*, uint16_t);

void ADIS_Read_Regs(const uint8_t*, uint16_t*, uint8_t);

void ADIS_RW_16(uint16_t *din, uint16_t *dout);

void ADIS_Read_DMA(void);

void ADIS_Init();

#endif //IMU_ADIS_H
