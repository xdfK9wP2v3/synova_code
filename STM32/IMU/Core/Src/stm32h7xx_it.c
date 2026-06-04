/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_msg.h"
#include "tcp_server.h"
#include "lwip.h"
#include "lwip/tcp.h"
#include "iwdg.h"
#include "error.h"
#include "adis.h"
#include "tim.h"
#include "i2c_queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile __DTCM uint32_t tick_wdg = SOFTWARE_WDG_RATE, rtc_wdg = SOFTWARE_WDG_RATE, msg_wdg = SOFTWARE_WDG_RATE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern I2C_HandleTypeDef hi2c3;
extern LPTIM_HandleTypeDef hlptim1;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi5_tx;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi5;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  error_handler(ERROR_FLAG_NMI);
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  error_handler(ERROR_FLAG_HardFault);
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  error_handler(ERROR_FLAG_MemManage);
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  error_handler(ERROR_FLAG_BusFault);
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  error_handler(ERROR_FLAG_UsageFault);
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
  if (sys_init_flag == 1 && __HAL_GPIO_EXTI_GET_FLAG(ADISI_Pin) != RESET) {
    global_adis_timestamp_next = get_on_chip_time();
    ADIS_Read_DMA();
  }
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ADISI_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c3_rx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi5_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt.
  */
void TIM1_BRK_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_IRQn 0 */

  /* USER CODE END TIM1_BRK_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_IRQn 1 */

  /* USER CODE END TIM1_BRK_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
//////////////////////////////////////////////
  tick_wdg = SOFTWARE_WDG_RATE;
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
//////////////////////////////////////////////
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts.
  */
void TIM1_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
  record_sync_raw(get_on_chip_time());
  Pulse_Monitor(T1C1);
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  ++sys_second;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  if (!sys_init_flag) {
    HAL_GPIO_EXTI_IRQHandler(MAGI_Pin);
    HAL_GPIO_EXTI_IRQHandler(LPS28I_Pin);
    HAL_GPIO_EXTI_IRQHandler(LPS22I_Pin);
    return;
  }

  if (__HAL_GPIO_EXTI_GET_IT(MAGI_Pin) != RESET)
  {
    record_mag_task(get_on_chip_time());
    HAL_GPIO_EXTI_IRQHandler(MAGI_Pin);
  }

  if (__HAL_GPIO_EXTI_GET_IT(LPS28I_Pin) != RESET)
  {
    record_lps28_task(get_on_chip_time());
    HAL_GPIO_EXTI_IRQHandler(LPS28I_Pin);
  }

  if (__HAL_GPIO_EXTI_GET_IT(LPS22I_Pin) != RESET)
  {
    record_lps22_task(get_on_chip_time());
    HAL_GPIO_EXTI_IRQHandler(LPS22I_Pin);
  }
  /* USER CODE END EXTI15_10_IRQn 0 */

  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c3_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles I2C3 event interrupt.
  */
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C3 error interrupt.
  */
void I2C3_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_ER_IRQn 0 */

  /* USER CODE END I2C3_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_ER_IRQn 1 */

  /* USER CODE END I2C3_ER_IRQn 1 */
}

/**
  * @brief This function handles SPI5 global interrupt.
  */
void SPI5_IRQHandler(void)
{
  /* USER CODE BEGIN SPI5_IRQn 0 */

  /* USER CODE END SPI5_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi5);
  /* USER CODE BEGIN SPI5_IRQn 1 */

  /* USER CODE END SPI5_IRQn 1 */
}

/**
  * @brief This function handles LPTIM1 global interrupt.
  */
void LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM1_IRQn 0 */
#ifdef ENABLE_WDG
  static uint32_t feed_dog_cnt = 0;
    if (feed_dog_cnt == 0) {
        HAL_IWDG_Refresh(&hiwdg1);
        feed_dog_cnt = 2 * SOFTWARE_WDG_RATE;
    }
    --feed_dog_cnt;
#endif

  if (tick_wdg > 0)
    --tick_wdg;
  else
    error_handler(ERROR_FLAG_PCLK_ERROR);

//  if (rtc_wdg > 0)
//    --rtc_wdg;
//  else
//    error_handler(ERROR_FLAG_RTC);
//

  if (msg_wdg > 0)
    --msg_wdg;
  else
    error_handler(ERROR_FLAG_MSG_OVERTIME);

  if (!sys_init_flag)  {
    HAL_LPTIM_IRQHandler(&hlptim1);
    return;
  }

  OnChipTime time = get_on_chip_time();

  static uint32_t last_second = 0;
  if (sys_error != 0 && (time.second - last_second) >= 1) {
    record_error_raw(time);
    last_second = time.second;
  }

  static uint16_t LED_state;
  LED_state = 0;
  uint64_t ticks = time.ticks;
  int blink = (ticks / (TICKS_PER_SEC / 5)) % 2 == 0;

  // T1C1
  uint32_t T1C1_tor = 50ul;
  if (last_ticks[T1C1] == 0 || ticks - last_ticks[T1C1] > TICKS_PER_SEC + T1C1_tor) {
    last_ticks[T1C1] = 0;
    filtered_ticks_interval[T1C1] = TICKS_PER_SEC;
  }
  else {
    if (filtered_ticks_interval[T1C1] <= TICKS_PER_SEC + T1C1_tor && filtered_ticks_interval[T1C1] >= TICKS_PER_SEC - T1C1_tor) {
      LED_state |= 1 << 0;
    }
    else
      LED_state |= blink << 0;
  }


  // FIX
  if (HAL_GPIO_ReadPin(RTKPVT_GPIO_Port, RTKPVT_Pin) == GPIO_PIN_SET)
    LED_state |= ((HAL_GPIO_ReadPin(RTKFIX_GPIO_Port, RTKFIX_Pin) == GPIO_PIN_SET) || blink) << 1;

  // TCP
  uint8_t tcp_all_estab = 1;
  for (int i = 0; i < TCP_PORT_NUM - 1; i++) {
    tcp_all_estab &= (eth_pcb[i] && eth_pcb[i]->local_port == (i + TCP_PORT_START) && eth_pcb[i]->state == ESTABLISHED);
  }
  if (tcp_all_estab)
    LED_state |= 1 << 2;
  else if (netif_is_link_up(&gnetif))
    LED_state |= blink << 2;

  // RTK
  if (last_ticks[RTCM] == 0 || ticks - last_ticks[RTCM] > TICKS_PER_SEC) {
    LED_state |= 1 << 3;
    last_ticks[RTCM] = 0;
  }
  else if (last_ticks[NMEA] == 0 || ticks - last_ticks[NMEA] > TICKS_PER_SEC) {
    LED_state |= 1 << 5;
    last_ticks[NMEA] = 0;
  }
  else
    LED_state |= 1 << 4;

  // ADIS
  uint32_t adis_tor = adis_base_tick / 100;
  if (last_ticks[ADIS] == 0 || ticks - last_ticks[ADIS] > adis_base_tick * 5) {
    last_ticks[ADIS] = 0;
    filtered_ticks_interval[ADIS] = adis_base_tick;
  }
  else {
    if (filtered_ticks_interval[ADIS] > adis_base_tick + adis_tor)
      LED_state |= 1 << 6;
    else if (filtered_ticks_interval[ADIS] <= adis_base_tick + adis_tor && filtered_ticks_interval[ADIS] >= adis_base_tick - adis_tor)
      LED_state |= 1 << 7;
    else
      LED_state |= 1 << 8;
  }

  // MAG
  uint32_t mag_tor = mag_base_tick / 20;
  int32_t mag_bias = -220;
  uint32_t mag_calib = mag_base_tick + mag_bias;
  if (last_ticks[MAG] == 0 || ticks - last_ticks[MAG] > mag_calib * 5) {
    last_ticks[MAG] = 0;
    filtered_ticks_interval[MAG] = mag_calib;
  }
  else {
    if (filtered_ticks_interval[MAG] > mag_calib + mag_tor)
      LED_state |= 1 << 9;
    else if (filtered_ticks_interval[MAG] <= mag_calib + mag_tor && filtered_ticks_interval[MAG] >= mag_calib - mag_tor)
      LED_state |= 1 << 10;
    else
      LED_state |= 1 << 11;
  }

  // LPS
  uint32_t lps_tor = lps28_base_tick / 20;
  int32_t lps28_bias = 3500;
  int32_t lps22_bias = -1700;
  uint32_t lps28_calib = lps28_base_tick + lps28_bias;
  uint32_t lps22_calib = lps22_base_tick + lps22_bias;
  if (last_ticks[LPS28] == 0 || ticks - last_ticks[LPS28] > lps28_calib * 5) {
    last_ticks[LPS28] = 0;
    filtered_ticks_interval[LPS28] = lps28_calib;
  }
  else if (last_ticks[LPS22] == 0 || ticks - last_ticks[LPS22] > lps22_calib * 5) {
    last_ticks[LPS22] = 0;
    filtered_ticks_interval[LPS22] = lps22_calib;
  }
  else {
    if (filtered_ticks_interval[LPS28] > lps28_calib + lps_tor || filtered_ticks_interval[LPS22] > lps22_calib + lps_tor)
      LED_state |= 1 << 12;
    else if (filtered_ticks_interval[LPS28] <= lps28_calib + lps_tor &&
             filtered_ticks_interval[LPS28] >= lps28_calib - lps_tor &&
             filtered_ticks_interval[LPS22] <= lps22_calib + lps_tor &&
             filtered_ticks_interval[LPS22] >= lps22_calib - lps_tor)
      LED_state |= 1 << 13;
    else
      LED_state |= 1 << 14;
  }

  // ERR
  if (HAL_GPIO_ReadPin(RTKERR_GPIO_Port, RTKERR_Pin) == GPIO_PIN_SET)
    FLAG_SET(sys_error, ERROR_FLAG_RTK_HW_FAIL);

  LED_state |= (sys_error != 0) << 15;

  LED_state = ~LED_state;
  HAL_SPI_Transmit_IT(SYS_STATE_SPI, (uint8_t *) &LED_state, 1);
  /* USER CODE END LPTIM1_IRQn 0 */
  HAL_LPTIM_IRQHandler(&hlptim1);
  /* USER CODE BEGIN LPTIM1_IRQn 1 */

  /* USER CODE END LPTIM1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
