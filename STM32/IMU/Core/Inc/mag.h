
#ifndef IMU_MAG_H
#define IMU_MAG_H

#define MAG_I2C_TIMEOUT 1000

#define MAG_I2C_ADDR (0x0E << 1)


#define MAG_REG_DR_STATUS 0x00

#define MAG_REG_OUT_START 0x01          // X_MSB, X_LSB, Y_ ..., Z_ ...,
#define MAG_REG_OUT_END 0x06

#define MAG_REG_WHO_AM_I 0x07

#define MAG_REG_SYSMOD 0x08

#define MAG_REG_OFF_START 0x09          // X_MSB, X_LSB, Y_ ..., Z_ ...,
#define MAG_REG_OFF_END 0x0E

#define MAG_REG_DIE_TEMP 0x0F

#define MAG_REG_STATUS_TO_TEMP_LEN 16

#define MAG_REG_CTRL_REG1 0x10
#define MAG_REG_CTRL_REG2 0x11


//#define MAG_OUT_RATE (0b000 << 5)
//#define MAG_OVER_SAMP (0b00 << 3)
#define MAG_FAST_READ (0 << 2)
#define MAG_TRIGGER_MEA (0 << 1)
#define MAG_OPERATING 0b1  // active mode
#define MAG_CTRL_REG1_VALUE (MAG_FAST_READ | MAG_TRIGGER_MEA | MAG_OPERATING)

#define MAG_SAMP_RST (1 << 7)
#define MAG_RAW_MODE (0 << 5)
#define MAG_MAG_RST (0 << 4)
#define MAG_CTRL_REG2_VALUE (MAG_SAMP_RST | MAG_RAW_MODE | MAG_MAG_RST)

void MAG_Init(void);

void MAG_Read_DMA(void);

#endif //IMU_MAG_H
