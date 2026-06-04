

#include "rtk.h"
#include "imu_msg.h"
#include "usart.h"

HAL_StatusTypeDef rtk_status;
__attribute__((aligned(32))) uint8_t rtk_nmea_data_bytes[256];
__attribute__((aligned(32))) uint8_t rtk_rtcm_data_bytes[1024];

const char nmea_bitmapping[][8] = {"gngga", "gpgsa", "gpgsv", "gprmc", "gpvtg", "gpgst", "gphdt", "gpzda"};
const char nmea_freq_bitmapping[][8] = {"1", "0.1", "0.05"};
const char pps_enable_bitmapping[][8] = {"", "enable", "enable2", "enable3"};

const char rtcm_msg[][8] = {"1006", "1033", "1074", "1124", "1084", "1094"};
const char rtcm_msg_freq[][8] = {"1", "1", "0.1", "0.1", "0.1", "0.1"};

void RTK_Init(void) {
  dbg_msg_len = sprintf(dbg_msg_buff, "[RTK-%s] Init...\r\n", rtk_config.rtk_type == RTK_BS ? "BS" : "MOB");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  char rtk_msg[256];
  uint32_t rtk_msg_len;

  if (rtk_config.freset_flag == 'R') {
      rtk_msg_len = sprintf(rtk_msg, "freset\r\n");
      HAL_UART_Transmit(RTK_RTCM_HDL, (uint8_t *) rtk_msg, rtk_msg_len, UART_TIMEOUT);
      HAL_Delay(5000);
  }

  if (rtk_config.rtk_type == RTK_BS) {
    char rtk_bs_cfg[][128] = {"config com1 115200\r\n",
                              "config com2 115200\r\n",
                              "config com3 115200\r\n",
                              "config undulation 0.0\r\n",
                              "mode base time 60 5\r\n",
                              "config pps enable2 bds positive 100000 1000 0 0\r\n",

                              "gpzda com2 1\r\n",
                              "gpzda com3 1\r\n"};
    sprintf(rtk_bs_cfg[4], "mode base time %hu %hu\r\n", rtk_config.BSConfig.fix_time, rtk_config.BSConfig.refix_distance);
    sprintf(rtk_bs_cfg[5], "config pps %s bds positive 100000 1000 0 0\r\n", pps_enable_bitmapping[rtk_config.BSConfig.pps_type]);
    for (uint32_t i = 0; i < sizeof(rtk_bs_cfg)/ sizeof(rtk_bs_cfg[0]); i++) {
      rtk_msg_len = sprintf(rtk_msg, "%s", rtk_bs_cfg[i]);
      if (HAL_UART_Transmit(RTK_RTCM_HDL, (uint8_t *) rtk_msg, rtk_msg_len, UART_TIMEOUT) != HAL_OK)
        error_handler(ERROR_FLAG_RTK_INIT_FAIL);
      HAL_Delay(100);
    }

    for (uint32_t i = 0; i < 8; i++) {
      if (rtk_config.BSConfig.nmea_bitmap >> i & 1) {
        rtk_msg_len = sprintf(rtk_msg, "%s com3 %s\r\n", nmea_bitmapping[i], nmea_freq_bitmapping[rtk_config.BSConfig.nmea_freq]);
        if (HAL_UART_Transmit(RTK_RTCM_HDL, (uint8_t *) rtk_msg, rtk_msg_len, UART_TIMEOUT) != HAL_OK)
          error_handler(ERROR_FLAG_RTK_INIT_FAIL);
        HAL_Delay(100);
      }
    }
    for (uint32_t i = 0; i < sizeof(rtcm_msg)/ sizeof(rtcm_msg[0]); i++) {
      rtk_msg_len = sprintf(rtk_msg, "rtcm%s com1 %s\r\n", rtcm_msg[i], rtcm_msg_freq[i]);
      HAL_UART_Transmit(RTK_RTCM_HDL, (uint8_t *) rtk_msg, rtk_msg_len, UART_TIMEOUT);
      HAL_Delay(100);
    }
  }
  else {
    char rtk_mob_cfg[][128] = {"config com1 115200\r\n",
                               "config com2 115200\r\n",
                               "config com3 115200\r\n",
                               "config undulation 0.0\r\n",
                               "config sbas enable auto\r\n",
                               "mode rover automotive\r\n",
                               "config pps enable2 bds positive 100000 1000 0 0\r\n",
                               "config heading length 43 4\r\n",

                               "gpzda com2 1\r\n",
                               "gpzda com3 1\r\n"};
    sprintf(rtk_mob_cfg[6], "config pps %s bds positive 100000 1000 0 0\r\n", pps_enable_bitmapping[rtk_config.MOBConfig.pps_type]);
    sprintf(rtk_mob_cfg[7], "config heading length %hu %hu\r\n", rtk_config.MOBConfig.heading_length, rtk_config.MOBConfig.heading_error);
    for (uint32_t i = 0; i < sizeof(rtk_mob_cfg)/ sizeof(rtk_mob_cfg[0]); i++) {
      rtk_msg_len = sprintf(rtk_msg, "%s", rtk_mob_cfg[i]);
      if (HAL_UART_Transmit(RTK_RTCM_HDL, (uint8_t *) rtk_msg, rtk_msg_len, UART_TIMEOUT) != HAL_OK)
        error_handler(ERROR_FLAG_RTK_INIT_FAIL);
      HAL_Delay(100);
    }
    for (uint32_t i = 0; i < 8; i++) {
      if (rtk_config.MOBConfig.nmea_bitmap >> i & 1) {
        rtk_msg_len = sprintf(rtk_msg, "%s com3 %s\r\n", nmea_bitmapping[i], nmea_freq_bitmapping[rtk_config.MOBConfig.nmea_freq]);
        if (HAL_UART_Transmit(RTK_RTCM_HDL, (uint8_t *) rtk_msg, rtk_msg_len, UART_TIMEOUT) != HAL_OK)
          error_handler(ERROR_FLAG_RTK_INIT_FAIL);
        HAL_Delay(100);
      }
    }
  }

  rtk_msg_len = sprintf(rtk_msg, "%s", "saveconfig\r\n");
  HAL_UART_Transmit(RTK_RTCM_HDL, (uint8_t *) rtk_msg, rtk_msg_len, UART_TIMEOUT);
  HAL_Delay(100);
}


void RTK_NMEA_DMA(void){
    rtk_status = HAL_UART_Receive_DMA(RTK_NMEA_HDL, rtk_nmea_data_bytes, 32);
    if (rtk_status != HAL_OK){
      dbg_msg_len = sprintf(dbg_msg_buff, "[RTK-NMEA] DMA Failed\r\n");
      dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
      error_handler(ERROR_FLAG_UART_NMEA_FAIL);
    }
}


void RTK_RTCM_RX_DMA(void){
  rtk_status = HAL_UART_Receive_DMA(RTK_RTCM_HDL, rtk_rtcm_data_bytes, 8);
  if (rtk_status != HAL_OK) {
    dbg_msg_len = sprintf(dbg_msg_buff, "[RTCM-RX] DMA Failed\r\n");
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_UART_RTCM_FAIL);
  }
}
