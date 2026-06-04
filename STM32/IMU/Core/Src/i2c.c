/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "imu_msg.h"
#include "mag.h"
#include "lps28.h"
#include "lps22.h"

__attribute__((aligned(32))) uint8_t i2c_buffer_bytes[32];
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

/* I2C3 init function */
void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x307075B1;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOH_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PH7     ------> I2C3_SCL
    PH8     ------> I2C3_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();

    /* I2C3 DMA Init */
    /* I2C3_RX Init */
    hdma_i2c3_rx.Instance = DMA1_Stream4;
    hdma_i2c3_rx.Init.Request = DMA_REQUEST_I2C3_RX;
    hdma_i2c3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c3_rx);

    /* I2C3_TX Init */
    hdma_i2c3_tx.Instance = DMA1_Stream7;
    hdma_i2c3_tx.Init.Request = DMA_REQUEST_I2C3_TX;
    hdma_i2c3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c3_tx);

    /* I2C3 interrupt Init */
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PH7     ------> I2C3_SCL
    PH8     ------> I2C3_SDA
    */
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_8);

    /* I2C3 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void I2C_Write_Byte(uint8_t addr, uint16_t reg, uint8_t value) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(I2C_HDL,
                                                 addr,
                                                 reg,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 &value,
                                                 1,
                                                 I2C_TIMEOUT);
    if (status != HAL_OK){
        char msg_buffer[256];
        uint32_t len;
        len = sprintf(msg_buffer, "[I2C] write failed, errno: %u\r\n", status);
        dbg_msg_transmit(msg_buffer, len);
        while (1);
    }
}


void I2C_Read_Byte(uint8_t addr, uint16_t reg, uint8_t *value) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2C_HDL,
                                                addr,
                                                reg,
                                                I2C_MEMADD_SIZE_8BIT,
                                                value,
                                                1,
                                                I2C_TIMEOUT);
    if (status != HAL_OK){
        char msg_buffer[256];
        uint32_t len;
        len = sprintf(msg_buffer, "[I2C] read failed, errno: %u\r\n", status);
        dbg_msg_transmit(msg_buffer, len);
        error_handler(ERROR_FLAG_I2C_FAIL);
    }
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == I2C_HDL) {
    SCB_InvalidateDCache_by_Addr((uint32_t *) i2c_buffer_bytes, 8);

    if (i2c_status == MAG_OCC) {
      record_mag_raw(global_i2c_timestamp_next,
                     i2c_buffer_bytes[0],
                     (int8_t) i2c_buffer_bytes[15],
                     (int16_t) (i2c_buffer_bytes[1] << 8 | i2c_buffer_bytes[2]),
                     (int16_t) (i2c_buffer_bytes[3] << 8 | i2c_buffer_bytes[4]),
                     (int16_t) (i2c_buffer_bytes[5] << 8 | i2c_buffer_bytes[6]));
      Pulse_Monitor(MAG);
    } else if (i2c_status == LPS28_OCC) {
      record_lps28_raw(global_i2c_timestamp_next,
                       i2c_buffer_bytes[0],
                       ((i2c_buffer_bytes[3] << 16) | (i2c_buffer_bytes[2] << 8) | i2c_buffer_bytes[1]) / 4096.0,
                       ((i2c_buffer_bytes[5] << 8) | i2c_buffer_bytes[4]) / 100.0);
      Pulse_Monitor(LPS28);
    } else if (i2c_status == LPS22_OCC) {
      record_lps22_raw(global_i2c_timestamp_next,
                       i2c_buffer_bytes[0],
                       ((i2c_buffer_bytes[3] << 16) | (i2c_buffer_bytes[2] << 8) | i2c_buffer_bytes[1]) / 4096.0,
                       ((i2c_buffer_bytes[5] << 8) | i2c_buffer_bytes[4]) / 100.0);
      Pulse_Monitor(LPS22);
    }
    i2c_status = I2C_READY;
  }
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  if (hi2c == I2C_HDL) {
    dbg_msg_len = sprintf(dbg_msg_buff, "[I2C] I2C ERROR\r\n");
    dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
    error_handler(ERROR_FLAG_I2C_FAIL);
  }
}
/* USER CODE END 1 */
