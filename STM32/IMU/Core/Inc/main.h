/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "error.h"
#include "mem_helper.h"
#include "help_macro.h"
#include "flash.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef enum {
  I2C_READY = 0,
  MAG_OCC = 1,
  LPS28_OCC = 2,
  LPS22_OCC = 3,
} i2cStatus;

typedef enum {
    RTCM_READY = 0,
    RTCM_BUSY = 1,
} uartStatus;

typedef enum {
    T1C1 = 0,
    NMEA = 1,
    RTCM = 2,
    ADIS = 3,
    MAG = 4,
    LPS28 = 5,
    LPS22 = 6,
} monitorsType;
#define MNT_ENUM_NUM 7

extern uint8_t i2c_status;
extern volatile uint8_t rtcm_uart_status;
extern char dbg_msg_buff[256];
extern uint32_t dbg_msg_len;
extern volatile uint32_t sys_second;
extern uint32_t filtered_ticks_interval[MNT_ENUM_NUM];
extern uint64_t last_ticks[MNT_ENUM_NUM];
extern uint32_t sys_error;
extern uint32_t sys_init_flag;

typedef struct {
    uint32_t second, microsecond;
    uint64_t ticks;
} OnChipTime;

extern OnChipTime global_i2c_timestamp_next;
extern OnChipTime global_adis_timestamp_next;

typedef struct {
    uint32_t ip_addr;
    uint32_t ip_mask;
    uint32_t ip_gate;

    uint32_t mac_prefix;
} ETH_ADDR_CONFIG;
extern ETH_ADDR_CONFIG eth_addr_config;

typedef struct {
    uint8_t rtk_type;
    uint8_t freset_flag;
    struct {
      uint8_t id;
      uint16_t fix_time;
      uint16_t refix_distance;
      uint8_t pps_type;
      uint8_t nmea_bitmap;
      uint8_t nmea_freq;
    } BSConfig;
    struct {
      uint8_t pps_type;
      uint16_t heading_length;
      uint16_t heading_error;
      uint8_t nmea_bitmap;
      uint8_t nmea_freq;
    } MOBConfig;
} RTKConfig;
extern RTKConfig rtk_config;

typedef struct {
    uint8_t adis_sync_mode;
    uint16_t mag_freq, lps28_freq, lps22_freq;
} IMUFreqConfig;
extern IMUFreqConfig imu_freq_config;
extern uint8_t flash_data_available;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ADIS_SPI_HDL &hspi2
#define SYS_STATE_SPI &hspi5

#define I2C_HDL &hi2c3
#define UART_DBG_HDL &huart1
#define RTK_RTCM_HDL &huart6
#define RTK_NMEA_HDL &huart3

#define SPI_TIMEOUT 1000ul
#define I2C_TIMEOUT 1000ul
#define UART_TIMEOUT 200ul

#define TICKS_PER_SEC 1000000ul

extern uint32_t adis_base_tick;
extern uint32_t mag_base_tick;
extern uint32_t lps28_base_tick;
extern uint32_t lps22_base_tick;

#define ENABLE_WDG
#define SOFTWARE_WDG_RATE 32
extern volatile __DTCM uint32_t tick_wdg, rtc_wdg, msg_wdg;

#define ERROR_FLAGS_ADDR 0

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Pulse_Monitor(uint8_t pulse_idx);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ALIVE_Pin GPIO_PIN_8
#define ALIVE_GPIO_Port GPIOF
#define RTKFIX_Pin GPIO_PIN_0
#define RTKFIX_GPIO_Port GPIOB
#define RTKPVT_Pin GPIO_PIN_1
#define RTKPVT_GPIO_Port GPIOB
#define RTKERR_Pin GPIO_PIN_2
#define RTKERR_GPIO_Port GPIOB
#define MAGI_Pin GPIO_PIN_13
#define MAGI_GPIO_Port GPIOF
#define MAGI_EXTI_IRQn EXTI15_10_IRQn
#define LPS28I_Pin GPIO_PIN_14
#define LPS28I_GPIO_Port GPIOF
#define LPS28I_EXTI_IRQn EXTI15_10_IRQn
#define LPS22I_Pin GPIO_PIN_15
#define LPS22I_GPIO_Port GPIOF
#define LPS22I_EXTI_IRQn EXTI15_10_IRQn
#define ADISI_Pin GPIO_PIN_0
#define ADISI_GPIO_Port GPIOG
#define ADISI_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
