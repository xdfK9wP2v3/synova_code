/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

typedef enum {
    PARSER_WAIT_SOL = 0,  // outside a line, waiting the Start Of Line ($)
    PARSER_WAIT_EOL = 1,  // inside the payload, waiting the End Of Line (*)
    PARSER_WAIT_CS1 = 2,  // waiting checksum 1
    PARSER_WAIT_CS2 = 3   // waiting checksum 2
} ParserState;

#define HOST_CMD_STR_MAX_LEN 256

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

FORCE_INLINE uint32_t unsigned_atoi_dec(uint32_t *const out, const uint8_t *const begin, const uint8_t *const end) {
    *out = 0;
    const uint8_t *it = begin;
    while (it < end && '0' <= *it && *it <= '9')
        *out = 10 * (*out) + (uint32_t) (*(it++) - '0');
    return it - begin;
}

FORCE_INLINE int32_t unsigned_ctoi_hex(const uint8_t ch) {
    if ('0' <= ch && ch <= '9')
        return ch - '0';
    if ('A' <= ch && ch <= 'F')
        return ch - 'A' + 10;
    if ('a' <= ch && ch <= 'f')
        return ch - 'a' + 10;
    return -1;
}

void circular_dma_parser(const uint8_t *dma_buffer, uint16_t dma_buffer_size, uint16_t *prev_pos, uint16_t curr_pos,
                         uint8_t *parser_buffer, uint16_t parser_buffer_size, ParserState *parser_state, uint16_t *parsed_len,
                         void (*line_parser)(const uint8_t *, uint16_t));

void normal_dma_parser(const uint8_t *dma_buffer, uint32_t len,
                       uint8_t *parser_buffer, uint16_t parser_buffer_size, ParserState *parser_state, uint16_t *parsed_len,
                       void (*line_parser)(const uint8_t *, uint16_t));

void nmea_parser(const uint8_t *parser_buffer, uint16_t parsed_len);

void config_command_parser(const uint8_t *parser_buffer, uint16_t parsed_len);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

