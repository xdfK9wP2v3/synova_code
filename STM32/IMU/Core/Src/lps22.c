

#include "lps22.h"
#include "i2c.h"
#include "imu_msg.h"

uint8_t get_lps22_freq_bit() {
  switch (imu_freq_config.lps22_freq) {
    case 10:
      return 0b010;
    case 25:
      return 0b011;
    case 50:
      return 0b100;
    case 75:
      return 0b101;
    default:
      error_handler(ERROR_FLAG_LPS22_INIT_FAIL);
  }
}

void LPS22_Init(void) {
  dbg_msg_len = sprintf(dbg_msg_buff, "[LPS22] Init ");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  I2C_Read_Byte(LPS22_I2C_ADDR, LPS22_REG_WHO_AM_I, i2c_buffer_bytes);
  if (i2c_buffer_bytes[0] != 0xB1) {
    dbg_msg_len = sprintf(dbg_msg_buff, "!! I2C Error\r\n");
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_LPS22_INIT_FAIL);
  }
  dbg_msg_len = sprintf(dbg_msg_buff, "-> I2C OK ");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  I2C_Write_Byte(LPS22_I2C_ADDR, LPS22_REG_CTRL_REG2, LPS22_CTRL_REG2_RST_VALUE);
  i2c_buffer_bytes[0] = 1 << 2;
  while (((i2c_buffer_bytes[0] & 0x4) >> 2) != 0) {
    I2C_Read_Byte(LPS22_I2C_ADDR, LPS22_REG_CTRL_REG2, i2c_buffer_bytes);
    HAL_Delay(50);
  }
  dbg_msg_len = sprintf(dbg_msg_buff, "-> Reset OK ");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  I2C_Write_Byte(LPS22_I2C_ADDR, LPS22_REG_CTRL_REG2, LPS22_CTRL_REG2_CFG_VALUE);
  I2C_Write_Byte(LPS22_I2C_ADDR, LPS22_REG_CTRL_REG1, LPS22_CTRL_REG1_VALUE | get_lps22_freq_bit() << 4);
  I2C_Write_Byte(LPS22_I2C_ADDR, LPS22_REG_CTRL_REG3, LPS22_CTRL_REG3_VALUE);
  HAL_Delay(100);
  dbg_msg_len = sprintf(dbg_msg_buff, "-> Config -> Success\r\n");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
}


void LPS22_Read_DMA(void) {
  if (HAL_I2C_Mem_Read_DMA(I2C_HDL,
                           LPS22_I2C_ADDR,
                           LPS22_REG_STATUS,
                           I2C_MEMADD_SIZE_8BIT,
                           i2c_buffer_bytes,
                           LPS22_REG_STATUS_PRESS_TEMP_LEN) == HAL_OK) {
    i2c_status = LPS22_OCC;
  } else {
    dbg_msg_len = sprintf(dbg_msg_buff, "[LPS22] DMA Failed once\r\n");
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_I2C_FAIL);
  }
}
