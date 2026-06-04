/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "lptim.h"
#include "lwip.h"
#include "memorymap.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mem_helper.h"
#include "rtk.h"
#include "adis.h"
#include "mag.h"
#include "lps28.h"
#include "lps22.h"
#include "imu_msg.h"
#include "rtcm_msg.h"
#include "tcp_server.h"
#include "i2c_queue.h"
//#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t i2c_status = I2C_READY;
volatile uint8_t rtcm_uart_status = RTCM_READY;
uint32_t sys_init_flag = 0;
uint32_t adis_base_tick;
uint32_t mag_base_tick;
uint32_t lps28_base_tick;
uint32_t lps22_base_tick;
uint32_t filtered_ticks_interval[MNT_ENUM_NUM] = {0};
uint64_t last_ticks[MNT_ENUM_NUM] = {0};
uint32_t sys_error = 0;
ETH_ADDR_CONFIG eth_addr_config;
RTKConfig rtk_config;
IMUFreqConfig imu_freq_config;
uint8_t flash_data_available = 0;
volatile uint32_t sys_second = 0;
OnChipTime global_i2c_timestamp_next;
OnChipTime global_adis_timestamp_next;

char dbg_msg_buff[256];
uint32_t dbg_msg_len;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  load_memory();
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  find_flash_config();

  if (flash_data_available) {
    load_config_from_flash();
    load_eth_addr_from_flash();
  } else {
    rtk_config = (RTKConfig) {.rtk_type = RTK_MOB, .MOBConfig.heading_length=43, .MOBConfig.pps_type=1, .MOBConfig.heading_error=4, .MOBConfig.nmea_bitmap=0xFF, .MOBConfig.nmea_freq=10};
    imu_freq_config = (IMUFreqConfig) {.adis_sync_mode = 0, .mag_freq = 80, .lps28_freq = 10, .lps22_freq = 10};
    const uint8_t default_ip_addr[] = {192, 168, 11, 11};
    const uint8_t default_ip_mask[] = {255, 255, 255, 0};
    const uint8_t default_ip_gate[] = {192, 168, 11, 1};
    // OUI of STMicroelectronics: 00:80:E1; Default code for IMU board: 0A
    const uint8_t mac_prefix[] = {0x00, 0x80, 0xE1, 0x0A};

    memcpy(&eth_addr_config.ip_addr, default_ip_addr, 4);
    memcpy(&eth_addr_config.ip_mask, default_ip_mask, 4);
    memcpy(&eth_addr_config.ip_gate, default_ip_gate, 4);
    memcpy(&eth_addr_config.mac_prefix, mac_prefix, 4);
  }

  adis_base_tick = TICKS_PER_SEC / 2000;
  mag_base_tick = TICKS_PER_SEC / imu_freq_config.mag_freq;
  lps28_base_tick = TICKS_PER_SEC / imu_freq_config.lps28_freq;
  lps22_base_tick = TICKS_PER_SEC / imu_freq_config.lps22_freq;
  filtered_ticks_interval[T1C1] = TICKS_PER_SEC;
  filtered_ticks_interval[ADIS] = adis_base_tick;
  filtered_ticks_interval[MAG] = mag_base_tick;
  filtered_ticks_interval[LPS28] = lps28_base_tick;
  filtered_ticks_interval[LPS22] = lps22_base_tick;

  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();
  __attribute__((aligned(32))) char msg_buffer[256];
  const uint32_t APB1_CLKFreq = HAL_RCC_GetPCLK1Freq();
  uint32_t tcp_msg_type;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LWIP_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI5_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_USART6_UART_Init();
  MX_LPTIM1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  check_error();
  dbg_msg_len = sprintf(dbg_msg_buff, "[MAIN] Last sys errno 0x%lX\r\n", sys_error);
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  ADIS_Init();
  MAG_Init();
  LPS28_Init();
  LPS22_Init();
  RTK_Init();
  TCP_Server_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  __HAL_UART_CLEAR_OREFLAG(RTK_NMEA_HDL);
  __HAL_UART_CLEAR_OREFLAG(RTK_RTCM_HDL);
#ifdef ENABLE_WDG
  MX_IWDG1_Init();
#endif
  dbg_msg_len = sprintf(dbg_msg_buff, "[MAIN] Device Init OK, ip: %08lX\r\n", eth_addr_config.ip_addr);
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_LPTIM_Counter_Start_IT(&hlptim1, APB1_CLKFreq / 128 - 1);
  sys_init_flag = 1;

  record_mag_task(get_on_chip_time());
  record_lps28_task(get_on_chip_time());
  record_lps22_task(get_on_chip_time());

  RTK_NMEA_DMA();
  if (rtk_config.rtk_type == RTK_BS)
    RTK_RTCM_RX_DMA();

#pragma ide diagnostic ignored "EndlessLoop"
  while (1) {
    static uint32_t pre_imu_msg_tcp_idx = 0;
    static uint32_t pre_rtcm_msg_tcp_idx = 0;
    static uint32_t pre_i2c_task_idx = 0;

    if (pre_imu_msg_tcp_idx != next_imu_msg_idx) {
      uint32_t data_msg_len = get_imu_msg(msg_buffer, &imu_msg_buffer[pre_imu_msg_tcp_idx], &tcp_msg_type);
      if (tcp_transmit(msg_buffer, tcp_msg_type, data_msg_len) == HAL_OK)
        pre_imu_msg_tcp_idx = (pre_imu_msg_tcp_idx + 1) % IMU_MSG_BUF_SIZE;
    }

    if (i2c_status == I2C_READY && pre_i2c_task_idx != next_i2c_task_idx) {
      I2CTask task = i2c_task_buffer[pre_i2c_task_idx];
      global_i2c_timestamp_next = task.timestamp;
      switch (task.task) {
        case MAG_OCC:
          MAG_Read_DMA();
          break;
        case LPS28_OCC:
          LPS28_Read_DMA();
          break;
        case LPS22_OCC:
          LPS22_Read_DMA();
          break;
        default:
          break;
      }
      pre_i2c_task_idx = (pre_i2c_task_idx + 1) % I2C_TASK_BUF_SIZE;
    }

    if (rtk_config.rtk_type == RTK_BS || (rtk_config.rtk_type == RTK_MOB && rtcm_uart_status == RTCM_READY)) {
      __attribute__((aligned(32))) static uint8_t rtcm_msg_buf[1024];
      if (pre_rtcm_msg_tcp_idx != next_rtcm_msg_idx) {
        uint32_t rtcm_len = get_rtcm_msg(rtcm_msg_buf, &rtcm_msg_buffer[pre_rtcm_msg_tcp_idx], &tcp_msg_type);
        if (rtcm_transmit(rtcm_msg_buf, tcp_msg_type, rtcm_len) == HAL_OK)
          pre_rtcm_msg_tcp_idx = (pre_rtcm_msg_tcp_idx + 1) % RTCM_MSG_BUF_SIZE;
      }
    }

    MX_LWIP_Process();

    msg_wdg = SOFTWARE_WDG_RATE;
    HAL_GPIO_TogglePin(ALIVE_GPIO_Port, ALIVE_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 60;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void Pulse_Monitor(uint8_t pulse_idx) {
  uint64_t current_tick = get_on_chip_time().ticks;
  if (last_ticks[pulse_idx] != 0)
    filtered_ticks_interval[pulse_idx] = (filtered_ticks_interval[pulse_idx] + (current_tick - last_ticks[pulse_idx])) / 2;
  last_ticks[pulse_idx] = current_tick;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30020000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512B;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  dbg_msg_len = sprintf(dbg_msg_buff, "[MAIN] error\r\n");
  dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
  error_handler(ERROR_FLAG_HAL_ERROR);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
