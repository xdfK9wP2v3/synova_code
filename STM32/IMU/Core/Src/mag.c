

#include "i2c.h"
#include "mag.h"
#include "imu_msg.h"

uint8_t get_mag_freq_bit() {
  switch (imu_freq_config.mag_freq) {
    case 10:
      return 0b00011;
    case 20:
      return 0b00010;
    case 40:
      return 0b00001;
    case 80:
      return 0b00000;
    default:
      error_handler(ERROR_FLAG_MAG_INIT_FAIL);
  }
}

void MAG_Init(void){
  dbg_msg_len = sprintf(dbg_msg_buff, "[MAG] Init ");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  I2C_Read_Byte(MAG_I2C_ADDR, MAG_REG_WHO_AM_I, i2c_buffer_bytes);
  if (i2c_buffer_bytes[0] != 0xC4) {
    dbg_msg_len = sprintf(dbg_msg_buff, "!! I2C Error\r\n");
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_MAG_INIT_FAIL);
  }
  dbg_msg_len = sprintf(dbg_msg_buff, "-> I2C OK ");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  I2C_Write_Byte(MAG_I2C_ADDR, MAG_REG_CTRL_REG2, MAG_CTRL_REG2_VALUE);
  I2C_Write_Byte(MAG_I2C_ADDR, MAG_REG_CTRL_REG1, MAG_CTRL_REG1_VALUE | get_mag_freq_bit() << 3);
  HAL_Delay(100);
  I2C_Read_Byte(MAG_I2C_ADDR, MAG_REG_SYSMOD, i2c_buffer_bytes);
  if (i2c_buffer_bytes[0] != 0x02) {
    dbg_msg_len = sprintf(dbg_msg_buff, "!! Wrong sys mode, expect 0x02, rev 0x%02X\r\n", i2c_buffer_bytes[0]);
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_MAG_INIT_FAIL);
  }
  dbg_msg_len = sprintf(dbg_msg_buff, "-> Config ");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  dbg_msg_len = sprintf(dbg_msg_buff, "-> Success\r\n");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
}


void MAG_Read_DMA(void) {
  if (HAL_I2C_Mem_Read_DMA(I2C_HDL,
                           MAG_I2C_ADDR,
                           MAG_REG_DR_STATUS,
                           I2C_MEMADD_SIZE_8BIT,
                           i2c_buffer_bytes,
                           MAG_REG_STATUS_TO_TEMP_LEN) == HAL_OK) {
    i2c_status = MAG_OCC;
  } else {
    dbg_msg_len = sprintf(dbg_msg_buff, "[MAG] DMA Failed once\r\n");
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_I2C_FAIL);
  }
}
