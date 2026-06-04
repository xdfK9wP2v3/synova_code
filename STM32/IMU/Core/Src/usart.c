/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "rtk.h"
#include "imu_msg.h"
#include "rtcm_msg.h"
#include "rtc.h"
#include "error.h"
#include "flash.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream6;
    hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA2_Stream0;
    hdma_usart3_tx.Init.Request = DMA_REQUEST_USART3_TX;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream2;
    hdma_usart6_rx.Init.Request = DMA_REQUEST_USART6_RX;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream3;
    hdma_usart6_tx.Init.Request = DMA_REQUEST_USART6_TX;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
FORCE_INLINE void line_splitter(const uint8_t curr_char,
                                uint8_t *const parser_buffer, const uint16_t parser_buffer_size, ParserState *const parser_state, uint16_t *const parsed_len,
                                void (*const line_parser)(const uint8_t *, uint16_t)) {
  if (curr_char == '\0')
    return; // ignore null character
  else if (curr_char == '$')
    parser_buffer[0] = '$', *parsed_len = 1, *parser_state = PARSER_WAIT_EOL;  // start of line (ignore any un-parser part)
  else if (*parsed_len >= parser_buffer_size || curr_char <= 0x20 || curr_char == 0x7F)
    *parsed_len = 0, *parser_state = PARSER_WAIT_SOL;  // if the line is too long (no more space of curr_char) or it contains control character (include space), reset the parser
  else if (curr_char == '*') {
    if (*parser_state == PARSER_WAIT_EOL)
      parser_buffer[(*parsed_len)++] = '*', *parser_state = PARSER_WAIT_CS1; // end of the message, begin to wait checksum
    else
      *parsed_len = 0, *parser_state = PARSER_WAIT_SOL; // illegal case, reset the parser
  }
  else {
    // copy the character to dma_buffer
    if (*parser_state == PARSER_WAIT_EOL)
      parser_buffer[(*parsed_len)++] = curr_char;
    else if (*parser_state == PARSER_WAIT_CS1)
      parser_buffer[(*parsed_len)++] = curr_char, *parser_state = PARSER_WAIT_CS2;
    else if (*parser_state == PARSER_WAIT_CS2) {
      parser_buffer[(*parsed_len)++] = curr_char;

      if (*parsed_len > 4) { // at least 1 payload
        // get checksum
        uint8_t checksum = 0;
        for (uint8_t *p = parser_buffer + 1, *end = parser_buffer + *parsed_len - 3; p < end; ++p)
          checksum ^= *p;

        // check if checksum correct
        const uint8_t line_checksum = unsigned_ctoi_hex(parser_buffer[*parsed_len - 2]) << 4 | unsigned_ctoi_hex(parser_buffer[*parsed_len - 1]);
        if (line_checksum == checksum)
          line_parser(parser_buffer, *parsed_len);
      }

      // finished parsing, reset parser
      *parsed_len = 0, *parser_state = PARSER_WAIT_SOL;
    }
  }
}

void circular_dma_parser(const uint8_t *const dma_buffer, const uint16_t dma_buffer_size, uint16_t *const prev_pos, const uint16_t curr_pos,
                         uint8_t *const parser_buffer, const uint16_t parser_buffer_size, ParserState *const parser_state, uint16_t *const parsed_len,
                         void (*const line_parser)(const uint8_t *, uint16_t)) {
  for (uint16_t it = *prev_pos; it != curr_pos % dma_buffer_size; it = (it + 1) % dma_buffer_size)
    line_splitter(dma_buffer[it], parser_buffer, parser_buffer_size, parser_state, parsed_len, line_parser);
  *prev_pos = curr_pos % dma_buffer_size;
}

void normal_dma_parser(const uint8_t *const dma_buffer, const uint32_t len,
                       uint8_t *const parser_buffer, const uint16_t parser_buffer_size, ParserState *const parser_state, uint16_t *const parsed_len,
                       void (*const line_parser)(const uint8_t *, uint16_t)) {
  for (uint32_t it = 0; it < len; ++it)
    line_splitter(dma_buffer[it], parser_buffer, parser_buffer_size, parser_state, parsed_len, line_parser);
}

void config_command_parser(const uint8_t *const parser_buffer, const uint16_t parsed_len) {
//  if (!FLAG_CHECK(sys_state, SYS_STATE_MAIN_TIM_ENABLE_FLAG)) // if system didn't initial, ignore config
//    return;

  const uint16_t payload_len = parsed_len - 4;
  if (payload_len < 6) { // at least have a header
    record_command_event(get_on_chip_time(), CMD_FAIL, UNKNOWN_HOST_COMMAND);
    return;
  }

  if (strncmp((const char *const) parser_buffer, "$CONFIG", 7) == 0) {
    if (payload_len < 6 + 1 + 5) { // at least have config header
      record_command_event(get_on_chip_time(), CMD_FAIL, UNKNOWN_HOST_COMMAND);
      return;
    }

    // $CONFIG,RTKB,freset,id,time,distance,pps_type,nmea_bitmap,nmea_freq*
    if (strncmp((const char *const) parser_buffer + 7, ",RTKB,", 6) == 0) {
      uint8_t freset_flag;
      uint8_t id;
      uint16_t fix_time;
      uint16_t refix_distance;
      uint8_t pps_type;
      uint8_t nmea_bitmap;
      uint8_t nmea_freq;
      if (sscanf((const char *const) parser_buffer, "$CONFIG,RTKB,%c,%hhu,%hu,%hu,%hhu,%2hhX,%hhu*",  // NOLINT(*-err34-c)
                 &freset_flag,
                 &id,
                 &fix_time,
                 &refix_distance,
                 &pps_type,
                 &nmea_bitmap,
                 &nmea_freq) == 7) {
        if (fix_time > 0 && refix_distance > 0 && (pps_type >= 1 && pps_type <= 3) && (nmea_freq >= 0 && nmea_freq <= 2)) {
          rtk_config.rtk_type = RTK_BS;
          rtk_config.freset_flag = freset_flag;
          rtk_config.BSConfig.id = id;
          rtk_config.BSConfig.fix_time = fix_time;
          rtk_config.BSConfig.refix_distance = refix_distance;
          rtk_config.BSConfig.pps_type = pps_type;
          rtk_config.BSConfig.nmea_bitmap = nmea_bitmap;
          rtk_config.BSConfig.nmea_freq = nmea_freq;
          record_command_event(get_on_chip_time(), CMD_OK, CONFIG_RTK_COMMAND);
          return;
        }
      }
      record_command_event(get_on_chip_time(), CMD_FAIL, CONFIG_RTK_COMMAND);
      return;
    }

    // $CONFIG,RTKM,freset,pps_type,heading_length,heading_error,nmea_bitmap,nmea_freq*
    else if (strncmp((const char *const) parser_buffer + 7, ",RTKM,", 6) == 0) {
      uint8_t freset_flag;
      uint8_t pps_type;
      uint16_t heading_length;
      uint16_t heading_error;
      uint8_t nmea_bitmap;
      uint8_t nmea_freq;
      if (sscanf((const char *const) parser_buffer, "$CONFIG,RTKM,%c,%hhu,%hu,%hu,%2hhX,%hhu*",  // NOLINT(*-err34-c)
                 &freset_flag,
                 &pps_type,
                 &heading_length,
                 &heading_error,
                 &nmea_bitmap,
                 &nmea_freq) == 6) {
        if ((pps_type >= 1 && pps_type <= 3) && heading_length > 0 && heading_error > 0 && (nmea_freq >= 0 && nmea_freq <= 2)) {
          rtk_config.rtk_type = RTK_MOB;
          rtk_config.freset_flag = freset_flag;
          rtk_config.MOBConfig.pps_type = pps_type;
          rtk_config.MOBConfig.heading_length = heading_length;
          rtk_config.MOBConfig.heading_error = heading_error;
          rtk_config.MOBConfig.nmea_bitmap = nmea_bitmap;
          rtk_config.MOBConfig.nmea_freq = nmea_freq;
          record_command_event(get_on_chip_time(), CMD_OK, CONFIG_RTK_COMMAND);
          return;
        }
      }
      record_command_event(get_on_chip_time(), CMD_FAIL, CONFIG_RTK_COMMAND);
      return;
    }
    // $CONFIG,IMUF,adis_sync_mode,mag_freq,lps28_freq,lps22_freq*
    else if (strncmp((const char *const) parser_buffer + 7, ",IMUF,", 6) == 0) {
      uint8_t adis_sync_mode;
      uint16_t mag_freq, lps28_freq, lps22_freq;
      if (sscanf((const char *const) parser_buffer, "$CONFIG,IMUF,%hhu,%hu,%hu,%hu*", // NOLINT(*-err34-c)
                 &adis_sync_mode, &mag_freq, &lps28_freq, &lps22_freq) == 4) {
        if ((adis_sync_mode >= 0 && adis_sync_mode <= 4) &&
                (mag_freq == 10 || mag_freq == 20 || mag_freq == 40 || mag_freq == 80) &&
                (lps28_freq == 10 || lps28_freq == 25 || lps28_freq == 50 || lps28_freq == 75) &&
                (lps22_freq == 10 || lps22_freq == 25 || lps22_freq == 50 || lps22_freq == 75)
        ) {
          imu_freq_config.adis_sync_mode = adis_sync_mode;
          imu_freq_config.mag_freq = mag_freq;
          imu_freq_config.lps28_freq = lps28_freq;
          imu_freq_config.lps22_freq = lps22_freq;

          record_command_event(get_on_chip_time(), CMD_OK, CONFIG_FREQ_COMMAND);
          return;
        }
      }
      record_command_event(get_on_chip_time(), CMD_FAIL, CONFIG_FREQ_COMMAND);
      return;
    }
    else if (strncmp((const char *const) parser_buffer + 7, ",IPMAC,", 7) == 0) {
      uint32_t ip_addr_buffer[4], ip_mask_buffer[4], ip_gate_buffer[4], mac_addr_buffer[4];
      if (sscanf((const char *const) parser_buffer, "$CONFIG,IPMAC,%3lu.%3lu.%3lu.%3lu,%3lu.%3lu.%3lu.%3lu,%3lu.%3lu.%3lu.%3lu,%2lx:%2lx:%2lx:%2lx*", // NOLINT(*-err34-c)
                 &ip_addr_buffer[0], &ip_addr_buffer[1], &ip_addr_buffer[2], &ip_addr_buffer[3],
                 &ip_mask_buffer[0], &ip_mask_buffer[1], &ip_mask_buffer[2], &ip_mask_buffer[3],
                 &ip_gate_buffer[0], &ip_gate_buffer[1], &ip_gate_buffer[2], &ip_gate_buffer[3],
                 &mac_addr_buffer[0], &mac_addr_buffer[1], &mac_addr_buffer[2], &mac_addr_buffer[3]) == 16) {
        uint8_t new_ip_addr[4], new_ip_mask[4], new_ip_gate[4], new_mac_addr[4];
        uint32_t *src[] = {ip_addr_buffer, ip_mask_buffer, ip_gate_buffer, mac_addr_buffer};
        uint8_t *dst[] = {new_ip_addr, new_ip_mask, new_ip_gate, new_mac_addr};
        for (uint32_t i = 0; i < 4; ++i)
          for (uint32_t j = 0; j < 4; ++j)
            if (src[i][j] >= 256) {
              record_command_event(get_on_chip_time(), CMD_FAIL, CONFIG_IP_MAC_ADDRESS_COMMAND);
              return;
            }
            else
              dst[i][j] = (uint8_t) src[i][j];

        memcpy(&eth_addr_config.ip_addr, new_ip_addr, 4);
        memcpy(&eth_addr_config.ip_mask, new_ip_mask, 4);
        memcpy(&eth_addr_config.ip_gate, new_ip_gate, 4);
        memcpy(&eth_addr_config.mac_prefix, new_mac_addr, 4);

        record_command_event(get_on_chip_time(), CMD_OK, CONFIG_IP_MAC_ADDRESS_COMMAND);
        return;
      }
      record_command_event(get_on_chip_time(), CMD_FAIL, CONFIG_IP_MAC_ADDRESS_COMMAND);
      return;
    }
  }
  else if(strncmp((const char *const) parser_buffer, "$REQCFG", 7) == 0) {
    if (payload_len < 6 + 1 + 5) { // at least have return config header
      record_command_event(get_on_chip_time(), CMD_FAIL, UNKNOWN_HOST_COMMAND);
      return;
    }

    if(payload_len == 12 && strncmp((const char *const) parser_buffer + 7, ",IPMAC", 6) == 0) {
      record_return_config_event(get_on_chip_time(), RETURN_IPMAC_CONFIG);
      return;
    }
    if(payload_len == 13 && strncmp((const char *const) parser_buffer + 7, ",RTKCFG", 7) == 0) {
      record_return_config_event(get_on_chip_time(), RETURN_RTK_CONFIG);
      return;
    }
  }
  else if (payload_len == 6 && strncmp((const char *const) parser_buffer, "$CLRERR", 7) == 0) {
    sys_error = 0;
    record_command_event(get_on_chip_time(), CMD_OK, CLEAR_ERROR_COMMAND);
    return;
  }
  else if (payload_len == 6 && strncmp((const char *const) parser_buffer, "$SAVCFG", 7) == 0) {
    shutdown_mode();
    if (save_config_to_flash() != HAL_OK)
      error_handler(ERROR_FLAG_FLASH);
    reset_system();
  }
  else if (payload_len == 6 && strncmp((const char *const) parser_buffer, "$REBOOT", 7) == 0) {
    reset_system();
  }
  else if (payload_len == 6 && strncmp((const char *const) parser_buffer, "$BKURST", 7) == 0) {
    shutdown_mode();
    __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
    reset_system();
  }
  else if (payload_len == 6 && strncmp((const char *const) parser_buffer, "$RSTALL", 7) == 0) {
    shutdown_mode();
    if (erase_flash_config_sector() != HAL_OK)
      error_handler(ERROR_FLAG_FLASH);
    __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
    reset_system();
  }
  record_command_event(get_on_chip_time(), CMD_FAIL, UNKNOWN_HOST_COMMAND);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == RTK_NMEA_HDL) {
    SCB_InvalidateDCache_by_Addr((uint32_t *) rtk_nmea_data_bytes, 32);
    record_rtk_nmea((char *) rtk_nmea_data_bytes);
    Pulse_Monitor(NMEA);
    RTK_NMEA_DMA();
  } else if (huart == RTK_RTCM_HDL) {
    SCB_InvalidateDCache_by_Addr((uint32_t *) rtk_rtcm_data_bytes, 1024);
    record_rtk_rtcm_tcp(rtk_rtcm_data_bytes, 8);
    Pulse_Monitor(RTCM);
    RTK_RTCM_RX_DMA();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if (huart == RTK_RTCM_HDL) {
    rtcm_uart_status = RTCM_READY;
    Pulse_Monitor(RTCM);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
  if (huart == RTK_NMEA_HDL) {
    uint32_t err_code = HAL_UART_GetError(RTK_NMEA_HDL);
    if ((err_code & HAL_UART_ERROR_DMA) != 0 || (err_code & HAL_UART_ERROR_RTO) != 0) {
      dbg_msg_len = sprintf(dbg_msg_buff, "[RTK] NMEA Errno: 0x%lX\r\n", err_code);
      dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
      error_handler(ERROR_FLAG_UART_NMEA_FAIL);
    }
  } else if (huart == RTK_RTCM_HDL) {
    uint32_t err_code = HAL_UART_GetError(RTK_RTCM_HDL);
    if ((err_code & HAL_UART_ERROR_DMA) != 0 || (err_code & HAL_UART_ERROR_RTO) != 0) {
      dbg_msg_len = sprintf(dbg_msg_buff, "[RTK] RTCM Errno: 0x%lX\r\n", err_code);
      dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
      error_handler(ERROR_FLAG_UART_RTCM_FAIL);
    }
  }
}
/* USER CODE END 1 */
