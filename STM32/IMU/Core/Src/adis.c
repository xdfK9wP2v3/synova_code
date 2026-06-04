

#include "adis.h"
#include "spi.h"
#include "usart.h"
#include "imu_msg.h"

HAL_StatusTypeDef spi_status;

uint16_t spi_rev_buff[SPI_BUFFER_SIZE];
uint16_t spi_str_buff[SPI_BUFFER_SIZE * 4];

const char adis_sync[][32] = {"internal clock",
                        "direct sync",
                        "scaled sync",
                        "output sync",
                        "",
                        "pulse sync"};

const uint8_t ADIS_state_addr[] = {ADIS_REG_DIAG_STAT};
const uint8_t ADIS_info_addr[] = {ADIS_REG_PROD_ID,
                                   ADIS_REG_SERIAL_NUM,
                                   ADIS_REG_FIRM_REV,
                                   ADIS_REG_FIRM_DM,
                                   ADIS_REG_FIRM_Y,
                                   ADIS_REG_FLSHCNT_LOW,
                                   ADIS_REG_FLSHCNT_HIGH};

const uint8_t ADIS_data_addr[] = {ADIS_REG_TIME_STAMP,
                                   ADIS_REG_DATA_CNTR,
                                   ADIS_REG_TEMP_OUT,
                                   ADIS_REG_X_GYRO_LOW,
                                   ADIS_REG_X_GYRO_OUT,
                                   ADIS_REG_Y_GYRO_LOW,
                                   ADIS_REG_Y_GYRO_OUT,
                                   ADIS_REG_Z_GYRO_LOW,
                                   ADIS_REG_Z_GYRO_OUT,
                                   ADIS_REG_X_ACCL_LOW,
                                   ADIS_REG_X_ACCL_OUT,
                                   ADIS_REG_Y_ACCL_LOW,
                                   ADIS_REG_Y_ACCL_OUT,
                                   ADIS_REG_Z_ACCL_LOW,
                                   ADIS_REG_Z_ACCL_OUT,
                                   0x00};
uint16_t ADIS_data_addr_size = sizeof(ADIS_data_addr) / sizeof(ADIS_data_addr[0]);

const uint16_t ADIS_dma_addr_16[] = {
        (ADIS_REG_DATA_CNTR & 0x7f) << 8,
        (ADIS_REG_TEMP_OUT & 0x7f) << 8,
//        (ADIS_REG_X_GYRO_LOW & 0x7f) << 8,
        (ADIS_REG_X_GYRO_OUT & 0x7f) << 8,
//        (ADIS_REG_Y_GYRO_LOW & 0x7f) << 8,
        (ADIS_REG_Y_GYRO_OUT & 0x7f) << 8,
//        (ADIS_REG_Z_GYRO_LOW & 0x7f) << 8,
        (ADIS_REG_Z_GYRO_OUT & 0x7f) << 8,
//        (ADIS_REG_X_ACCL_LOW & 0x7f) << 8,
        (ADIS_REG_X_ACCL_OUT & 0x7f) << 8,
//        (ADIS_REG_Y_ACCL_LOW & 0x7f) << 8,
        (ADIS_REG_Y_ACCL_OUT & 0x7f) << 8,
//        (ADIS_REG_Z_ACCL_LOW & 0x7f) << 8,
        (ADIS_REG_Z_ACCL_OUT & 0x7f) << 8,
        0x0000};
const uint16_t ADIS_dma_addr_size = sizeof(ADIS_dma_addr_16) / sizeof(ADIS_dma_addr_16[0]);


uint32_t hex_to_hexstr(char *const buf, const uint16_t *const data, const uint16_t size) {
  uint32_t len = 0;
  for (uint32_t i = 0; i < size; i++)
    len += sprintf(buf + len, "%04X,", data[i]);
  return len;
}


void ADIS_Read_Regs(const uint8_t *addr, uint16_t *value, uint8_t size) {
  uint16_t din_msg;
  uint16_t tx_empty = 0, rx_empty = 0;

  din_msg = (addr[0] & 0x7f) << 8;
  ADIS_RW_16(&din_msg, &rx_empty);

  for (uint8_t i = 1; i < size; i++) {
    din_msg = (addr[i] & 0x7f) << 8;
    ADIS_RW_16(&din_msg, &value[i - 1]);
  }
  ADIS_RW_16(&tx_empty, &value[(size - 1)]);
}


void ADIS_Write_Reg(uint8_t addr, uint16_t data) {
  uint16_t din_msg, rx_empty;

  din_msg = (((addr & 0x7f) | 0x80) << 8) | (data & 0xff);
  ADIS_RW_16(&din_msg, &rx_empty);
  din_msg = ((((addr + 1) & 0x7f) | 0x80) << 8) | (data >> 8);
  ADIS_RW_16(&din_msg, &rx_empty);
}


void ADIS_RW_16(uint16_t *din, uint16_t *dout) {
  spi_status = HAL_SPI_TransmitReceive(ADIS_SPI_HDL, (uint8_t *) din, (uint8_t *) dout, 1, SPI_TIMEOUT);
  if (spi_status != HAL_OK) {
    dbg_msg_len = sprintf(dbg_msg_buff, "[SPI] RW failed, errno: %u\r\n", spi_status);
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_SPI_FAIL);
  }
  HAL_Delay(1);
}


void ADIS_Init() {
  dbg_msg_len = sprintf(dbg_msg_buff, "[ADIS] Init ");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  ADIS_Write_Reg(ADIS_REG_GLOB_CMD, ADIS_RESET_VALUE);
  HAL_Delay(1000);
  dbg_msg_len = sprintf(dbg_msg_buff, "-> Reset ");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  ADIS_Read_Regs(ADIS_info_addr, spi_rev_buff, 1);
//  uint32_t len = hex_to_hexstr((char *) spi_str_buff, spi_rev_buff, ADIS_info_addr_size);
//  dbg_msg_transmit(spi_str_buff, len);
  if (spi_rev_buff[0] != 0x4056) {
    dbg_msg_len = sprintf(dbg_msg_buff, "!! SPI Error\r\n");
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_ADIS_INIT_FAIL);
  }
  dbg_msg_len = sprintf(dbg_msg_buff, "-> SPI OK ");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  ADIS_Write_Reg(ADIS_REG_FILT_CTRL, ADIS_FILT_CTRL_VALUE);
  ADIS_Write_Reg(ADIS_REG_MSC_CTRL, (ADIS_MSC_CTRL_VALUE & 0xFFFB) | (imu_freq_config.adis_sync_mode << 2));

  if (imu_freq_config.adis_sync_mode == 2) {
    ADIS_Write_Reg(ADIS_REG_UP_SCALE, ADIS_UP_SCALE_VALUE);
    sprintf(dbg_msg_buff, "-> Scaled SYNC mode enabled, up_scale (decimal): %d", ADIS_UP_SCALE_VALUE);
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
  }

  ADIS_Write_Reg(ADIS_REG_DEC_RATE, ADIS_DEC_RATE_VALUE);
  ADIS_Write_Reg(ADIS_REG_NULL_CNFG, ADIS_NULL_CNFG_VALUE);
  dbg_msg_len = sprintf(dbg_msg_buff, "-> Config (SYNC mode: %s) ", adis_sync[imu_freq_config.adis_sync_mode]);
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  ADIS_Write_Reg(ADIS_REG_GLOB_CMD, ADIS_SELF_TEST_VALUE);
  HAL_Delay(1000);
  ADIS_Read_Regs(ADIS_state_addr, spi_rev_buff, sizeof(ADIS_state_addr) / sizeof(ADIS_state_addr[0]));
  if (((spi_rev_buff[0] >> 5) & 0x1) == 1) {
    dbg_msg_len = sprintf(dbg_msg_buff, "!! Self Test failed\r\n");
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_ADIS_INIT_FAIL);
  }
  dbg_msg_len = sprintf(dbg_msg_buff, "-> Self-test Pass -> Success\r\n");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
}


void ADIS_Read_DMA(void) {
  spi_status = HAL_SPI_TransmitReceive_DMA(ADIS_SPI_HDL,
                                           (uint8_t *) ADIS_dma_addr_16,
                                           (uint8_t *) spi_rev_buff,
                                           ADIS_dma_addr_size);
  if (spi_status != HAL_OK) {
    dbg_msg_len = sprintf(dbg_msg_buff, "[ADIS] DMA Failed once (errno %d)\r\n", spi_status);
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_SPI_FAIL);
  }
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == ADIS_SPI_HDL) {
    SCB_InvalidateDCache_by_Addr((uint32_t *) spi_rev_buff, 16);
    record_adis_raw(global_adis_timestamp_next,
                    spi_rev_buff[1],
                    (int16_t) spi_rev_buff[2],
                    (int16_t) spi_rev_buff[3],
                    (int16_t) spi_rev_buff[4],
                    (int16_t) spi_rev_buff[5],
                    (int16_t) spi_rev_buff[6],
                    (int16_t) spi_rev_buff[7],
                    (int16_t) spi_rev_buff[8]);
    Pulse_Monitor(ADIS);
  }
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == ADIS_SPI_HDL) {
    dbg_msg_len = sprintf(dbg_msg_buff, "[ADIS] SPI ERROR\r\n");
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_SPI_FAIL);
  }
}
