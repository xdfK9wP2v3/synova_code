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
#include "flash.h"

#include <string.h>
#include <stdio.h>

extern TIM_HandleTypeDef MAIN_TIM;
extern RTC_HandleTypeDef hrtc;

__DMA_BUFFER uint8_t NMEA_DMA_buffer[UART_DMA_BUFFER_SIZE];
__DMA_BUFFER uint8_t BKP_DMA_buffer[UART_DMA_BUFFER_SIZE];

volatile time_t gps_sys_start_timestamp;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void) {

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
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/* USART3 init function */

void MX_USART3_UART_Init(void) {

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
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (uartHandle->Instance == USART1) {
        /* USER CODE BEGIN USART1_MspInit 0 */

        /* USER CODE END USART1_MspInit 0 */

        /** Initializes the peripherals clock
        */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /* USART1 clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 DMA Init */
        /* USART1_TX Init */
        hdma_usart1_tx.Instance = DMA1_Stream1;
        hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
        hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_tx.Init.Mode = DMA_NORMAL;
        hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart1_tx);

        /* USART1_RX Init */
        hdma_usart1_rx.Instance = DMA1_Stream2;
        hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
        hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart1_rx);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        /* USER CODE BEGIN USART1_MspInit 1 */

        /* USER CODE END USART1_MspInit 1 */
    }
    else if (uartHandle->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */

        /** Initializes the peripherals clock
        */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /* USART3 clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USART3 DMA Init */
        /* USART3_RX Init */
        hdma_usart3_rx.Instance = DMA1_Stream0;
        hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle) {

    if (uartHandle->Instance == USART1) {
        /* USER CODE BEGIN USART1_MspDeInit 0 */

        /* USER CODE END USART1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

        /* USART1 DMA DeInit */
        HAL_DMA_DeInit(uartHandle->hdmatx);
        HAL_DMA_DeInit(uartHandle->hdmarx);

        /* USART1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        /* USER CODE BEGIN USART1_MspDeInit 1 */

        /* USER CODE END USART1_MspDeInit 1 */
    }
    else if (uartHandle->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);

        /* USART3 DMA DeInit */
        HAL_DMA_DeInit(uartHandle->hdmarx);

        /* USART3 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
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

            // finished parse, reset parser
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

/* HAL uart handler */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t curr_pos) {
    if (huart == &NMEA_UART) {
        static uint16_t prev_nmea_pos = 0;
        static uint16_t nmea_parsed_len = 0;
        static ParserState nmea_parser_state = 0;
        static uint8_t nmea_parser_buffer[HOST_CMD_STR_MAX_LEN];

        circular_dma_parser(NMEA_DMA_buffer, UART_DMA_BUFFER_SIZE, &prev_nmea_pos, curr_pos,
                            nmea_parser_buffer, sizeof(nmea_parser_buffer), &nmea_parser_state, &nmea_parsed_len,
                            nmea_parser);
    }
    else if (huart == &BKP_UART) {
        static uint16_t prev_bkp_pos = 0;
        static uint16_t bkp_parsed_len = 0;
        static ParserState bkp_parser_state = 0;
        static uint8_t bkp_parser_buffer[HOST_CMD_STR_MAX_LEN];

        circular_dma_parser(BKP_DMA_buffer, UART_DMA_BUFFER_SIZE, &prev_bkp_pos, curr_pos,
                            bkp_parser_buffer, sizeof(bkp_parser_buffer), &bkp_parser_state, &bkp_parsed_len,
                            config_command_parser);
    }
    else
        return;

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (HAL_UART_DMAStop(huart) != HAL_OK)
        error_handler(ERROR_FLAG_UART);

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    __HAL_UART_CLEAR_IT(huart, UART_IT_ERR);

    if (huart == &NMEA_UART) {
        if (HAL_UARTEx_ReceiveToIdle_DMA(&NMEA_UART, NMEA_DMA_buffer, UART_DMA_BUFFER_SIZE) != HAL_OK)
            error_handler(ERROR_FLAG_UART);
    }
    else if (huart == &BKP_UART) {
        if (HAL_UARTEx_ReceiveToIdle_DMA(&BKP_UART, BKP_DMA_buffer, UART_DMA_BUFFER_SIZE) != HAL_OK)
            error_handler(ERROR_FLAG_UART);
    }
}

__DTCM uint32_t utc_input_stable_wdg = 0;

void nmea_parser(const uint8_t *const parser_buffer, const uint16_t parsed_len) {
    if (parsed_len < 1 + 5 + 1 + 1 + 2)// at least have NMEA header ($ --ZDA , * CS)
        return;

    if (parser_buffer[3] == 'Z' && parser_buffer[4] == 'D' && parser_buffer[5] == 'A' && parser_buffer[6] == ',') { // $--ZDA,...*CS
        uint32_t YYYY = 0, MM = 0, DD = 0;
        uint32_t hh = 0, mm = 0, ss = 0, ms = 0;

        const uint8_t *p = parser_buffer + 6, *q = p, *const end = parser_buffer + parsed_len;
        uint8_t comma_idx = 0, error = 0;
        while (p < end)
            if (*(++p) == ',') {
                switch (comma_idx) {
                    case 0: // hhmmss.sss
                        if (p - q >= 7) {
                            if (unsigned_atoi_dec(&hh, q + 1, q + 3) != 2)
                                ++error;
                            if (unsigned_atoi_dec(&mm, q + 3, q + 5) != 2)
                                ++error;
                            if (unsigned_atoi_dec(&ss, q + 5, q + 7) != 2)
                                ++error;
                            if (p - q > 8 && q[7] == '.') // ms part
                                for (uint32_t ms_len = unsigned_atoi_dec(&ms, q + 8, MIN(q + 11, p)); ms_len < 3; ++ms_len)
                                    ms *= 10;
                        }
                        else
                            ++error;
                        break;
                    case 1: // DD
                        if (p - q != 3 || unsigned_atoi_dec(&DD, q + 1, q + 3) != 2)
                            ++error;
                        break;
                    case 2: // MM
                        if (p - q != 3 || unsigned_atoi_dec(&MM, q + 1, q + 3) != 2)
                            ++error;
                        break;
                    case 3: // YYYY
                        if (p - q == 3) {
                            if (unsigned_atoi_dec(&YYYY, q + 1, q + 3) != 2)
                                ++error;
                            YYYY += 2000;
                        }
                        else if (p - q == 5) {
                            if (unsigned_atoi_dec(&YYYY, q + 1, q + 5) != 4)
                                ++error;
                        }
                        else
                            ++error;
                        break;
                    default:
                        break;
                }
                q = p;
                ++comma_idx;
            }

        if (error == 0 && comma_idx >= 4) { // ZDA parse successful
            TimePoint tp;
            get_time_point_low_priority(&tp);

            record_utc_input_event(sys_state, &tp, YYYY, MM, DD, hh, mm, ss, ms);

            static uint32_t utc_input_stable_cnt = 0;
            static uint32_t gps_timestamp_stable = 0;

            // if utc timestamp is available, but is significant different from input, disable the utc timestamp
            time_t timestamp = get_timestamp(YYYY, MM, DD, hh, mm, ss);
            if (FLAG_CHECK(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG))
                if (utc_sys_start_timestamp < timestamp - tp.second - 3 || timestamp - tp.second + 3 < utc_sys_start_timestamp)
                    ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG);

            int second_shift;
            const uint32_t curr_ms = tp.tick * 1000 / timing_scale.ticks_per_second;
            if (FLAG_CHECK(sys_state, SYS_STATE_PULSE_PHASE_FAIR_FLAG) &&
                sub_second_sync(ms, curr_ms, PULSE_NMEA_ADVANCE, PULSE_NMEA_DELAY, 1000, &second_shift)) {

                // get unix sync_timestamp
                time_t sync_timestamp = timestamp + second_shift;

                utc_input_stable_wdg = UTC_INPUT_TIMEOUT * SOFTWARE_WDG_RATE;
                if (FLAG_CHECK(sys_state, SYS_STATE_UTC_INPUT_AVAILABLE_FLAG)) {
                    if (utc_input_stable_cnt < UTC_INPUT_STABLE_THRESHOLD)
                        ++utc_input_stable_cnt;
                    else {
                        ATOMIC_FLAG_SET(sys_state, SYS_STATE_UTC_INPUT_STABLE_FLAG);

                        const uint32_t new_gps_sys_start_timestamp = sync_timestamp - tp.second; // get the new system start UNIX sync_timestamp
                        if (new_gps_sys_start_timestamp == gps_sys_start_timestamp) {
                            if (gps_timestamp_stable < UTC_INPUT_TIMESTAMP_STABLE_THRESHOLD)
                                ++gps_timestamp_stable;
                            else
                                ATOMIC_FLAG_SET(sys_state, SYS_STATE_UTC_INPUT_TIMESTAMP_STABLE_FLAG);
                        }
                        else {
                            ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_INPUT_TIMESTAMP_STABLE_FLAG);
                            gps_timestamp_stable = 0;
                        }
                        gps_sys_start_timestamp = new_gps_sys_start_timestamp;
                    }
                }
                else {
                    ATOMIC_FLAG_SET(sys_state, SYS_STATE_UTC_INPUT_AVAILABLE_FLAG);
                    utc_input_stable_cnt = 0, gps_timestamp_stable = 0;
                }
            }
            else {
                ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_INPUT_AVAILABLE_FLAG);
                ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_INPUT_STABLE_FLAG);
                ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_INPUT_TIMESTAMP_STABLE_FLAG);
            }
        }
    }
}

void config_command_parser(const uint8_t *const parser_buffer, const uint16_t parsed_len) {
    if (!FLAG_CHECK(sys_state, SYS_STATE_MAIN_TIM_ENABLE_FLAG)) // if system didn't initial, ignore config
        return;

    TimePoint tp;
    get_time_point_low_priority(&tp);

    const uint16_t payload_len = parsed_len - 4;
    if (payload_len < 6) { // at least have a header
        record_command_event(sys_state, &tp, CMD_FAIL, UNKNOWN_HOST_COMMAND);
        return;
    }

    if (strncmp((const char *const) parser_buffer, "$CONFIG", 7) == 0) {
        if (payload_len < 6 + 1 + 5) { // at least have config header
            record_command_event(sys_state, &tp, CMD_FAIL, UNKNOWN_HOST_COMMAND);
            return;
        }

        if (strncmp((const char *const) parser_buffer + 7, ",TIMEC,", 7) == 0) {
            uint32_t channel, pre_divider;
            int32_t shift;
            char flag;
            if (sscanf((const char *const) parser_buffer, "$CONFIG,TIMEC,%2lu,%10lu,%10ld,%c*", &channel, &pre_divider, &shift, &flag) == 4) { // NOLINT(*-err34-c)
                if (1 <= channel && channel <= TIMER_CAPTURE_NUM && (flag == 'N' || flag == 'A' || flag == 'D')) {
                    TimerCaptureConfig *const timer_config = &timer_capture_configs[channel - 1];

                    if (flag == 'N')
                        ATOMIC_FLAG_CLEAR(timer_config->flags, TC_SECOND_ALIGN | TC_DROP_LAST);
                    else if (flag == 'A')
                        ATOMIC_FLAG_SET_CLEAR(timer_config->flags, TC_SECOND_ALIGN, TC_DROP_LAST);
                    else
                        ATOMIC_FLAG_SET(timer_config->flags, TC_SECOND_ALIGN | TC_DROP_LAST);

                    timer_config->pre_divider = pre_divider;
                    timer_config->shift = shift;

                    record_command_event(sys_state, &tp, CMD_OK, CONFIG_TIMER_COMMAND);
                    return;
                }
            }
            record_command_event(sys_state, &tp, CMD_FAIL, CONFIG_TIMER_COMMAND);
            return;
        }
        else if (strncmp((const char *const) parser_buffer + 7, ",TRIGR,", 7) == 0) {
            uint32_t channel, pulse_width, dead_zone;
            int32_t rf_compensation;
            uint64_t masks[TRIGGER_SIGNAL_SIZE];
            uint64_t mask_checksum;
            char polarity, pulse_flag, deadz_flag;
            if (sscanf((const char *const) parser_buffer, "$CONFIG,TRIGR,%2lu,%10lu,%10lu,%10ld,%16llx,%16llx,%16llx,%16llx,%16llx,%16llx,%c,%c%c*", // NOLINT(*-err34-c)
                       &channel, &pulse_width, &dead_zone, &rf_compensation,
                       &masks[0], &masks[1], &masks[2], &masks[3], &masks[4], &mask_checksum,
                       &polarity, &pulse_flag, &deadz_flag) == 13) {
                uint64_t mask_check = 0xAAAAAAAAAAAAAAAA;
                for (int i = 0; i < TRIGGER_SIGNAL_SIZE; ++i)
                    mask_check ^= masks[i];
                if (1 <= channel && channel <= OUTPUT_TRIGGER_NUM &&
                    (pulse_width < (1 << 16)) &&
                    (dead_zone < (1 << 16)) &&
                    (-(1 << 15) <= rf_compensation && rf_compensation < (1 << 15)) &&
                    (polarity == 'H' || polarity == 'L') &&
                    (pulse_flag == 'D' || pulse_flag == 'S' || pulse_flag == 'E' || pulse_flag == 'F') &&
                    (deadz_flag == 'D' || deadz_flag == 'P' || deadz_flag == 'L' || deadz_flag == 'F') &&
                    mask_check == mask_checksum) {

                    OutputTriggerPulseConfig *const pulse_config = &output_trigger_configs[channel - 1].pulse;

                    if (pulse_flag == 'D')
                        ATOMIC_FLAG_CLEAR(pulse_config->flags, OT_ACTIVATE_EXPEND_FLAG | OT_ACTIVATE_FALL_PRIORITY_FLAG);
                    else if (pulse_flag == 'S')
                        ATOMIC_FLAG_SET_CLEAR(pulse_config->flags, OT_ACTIVATE_FALL_PRIORITY_FLAG, OT_ACTIVATE_EXPEND_FLAG);
                    else if (pulse_flag == 'E')
                        ATOMIC_FLAG_SET_CLEAR(pulse_config->flags, OT_ACTIVATE_EXPEND_FLAG, OT_ACTIVATE_FALL_PRIORITY_FLAG);
                    else
                        ATOMIC_FLAG_SET(pulse_config->flags, OT_ACTIVATE_EXPEND_FLAG | OT_ACTIVATE_FALL_PRIORITY_FLAG);

                    if (deadz_flag == 'D')
                        ATOMIC_FLAG_CLEAR(pulse_config->flags, OT_DEADZ_ALLOW_PEND_FLAG | OT_DEADZ_PEND_FALL_PRIORITY_FLAG | OT_DEADZ_PEND_FIRST_PRIORITY_FLAG);
                    else if (deadz_flag == 'P')
                        ATOMIC_FLAG_SET_CLEAR(pulse_config->flags, OT_DEADZ_ALLOW_PEND_FLAG, OT_DEADZ_PEND_FALL_PRIORITY_FLAG | OT_DEADZ_PEND_FIRST_PRIORITY_FLAG);
                    else if (deadz_flag == 'L')
                        ATOMIC_FLAG_SET_CLEAR(pulse_config->flags, OT_DEADZ_ALLOW_PEND_FLAG | OT_DEADZ_PEND_FALL_PRIORITY_FLAG, OT_DEADZ_PEND_FIRST_PRIORITY_FLAG);
                    else
                        ATOMIC_FLAG_SET(pulse_config->flags, OT_ACTIVATE_EXPEND_FLAG | OT_DEADZ_PEND_FALL_PRIORITY_FLAG | OT_DEADZ_PEND_FIRST_PRIORITY_FLAG);

                    if (polarity == 'H')
                        ATOMIC_FLAG_CLEAR(pulse_config->flags, OT_POLARITY_LOW);
                    else
                        ATOMIC_FLAG_SET(pulse_config->flags, OT_POLARITY_LOW);

                    for (int i = 0; i < TRIGGER_SIGNAL_SIZE; ++i)
                        pulse_config->signal_masks[i] = masks[i];

                    pulse_config->dead_zone = (uint16_t) dead_zone;
                    pulse_config->pulse_width = (uint16_t) pulse_width;
                    pulse_config->rf_compensation = (int16_t) rf_compensation;

                    record_command_event(sys_state, &tp, CMD_OK, CONFIG_TRIGGER_COMMAND);
                    return;
                }
            }
            record_command_event(sys_state, &tp, CMD_FAIL, CONFIG_TRIGGER_COMMAND);
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
                            record_command_event(sys_state, &tp, CMD_FAIL, CONFIG_IP_MAC_ADDRESS_COMMAND);
                            return;
                        }
                        else
                            dst[i][j] = (uint8_t) src[i][j];

                record_command_event(sys_state, &tp, CMD_OK, CONFIG_IP_MAC_ADDRESS_COMMAND);

                /** Turn off all system to avoid conflict **/
                shutdown_mode();

                /** Set configs **/
                memcpy(&eth_addr_config.ip_addr, new_ip_addr, 4);
                memcpy(&eth_addr_config.ip_mask, new_ip_mask, 4);
                memcpy(&eth_addr_config.ip_gate, new_ip_gate, 4);
                memcpy(&eth_addr_config.mac_prefix, new_mac_addr, 4);

                /** Reboot the system **/
                if (save_config_to_flash() != HAL_OK)
                    error_handler(ERROR_FLAG_FLASH);
                reset_system();

                return;
            }
            record_command_event(sys_state, &tp, CMD_FAIL, CONFIG_IP_MAC_ADDRESS_COMMAND);
            return;
        }
        else if (strncmp((const char *const) parser_buffer + 7, ",CAPIN,", 7) == 0) {
            const uint8_t *const edges = parser_buffer + 7 + 7;
            for (uint32_t channel = 0; channel < INPUT_CAPTURE_NUM; ++channel)
                if (edges[channel] != 'R' && edges[channel] != 'F' && edges[channel] != 'B') {
                    record_command_event(sys_state, &tp, CMD_FAIL, CONFIG_CAPIN_COMMAND);
                    return;
                }
            if (edges[INPUT_CAPTURE_NUM] != '*') {
                record_command_event(sys_state, &tp, CMD_FAIL, CONFIG_CAPIN_COMMAND);
                return;
            }

            /** Turn off all system to avoid conflict **/
            shutdown_mode();
            for (uint32_t channel = 0; channel < INPUT_CAPTURE_NUM; ++channel) {
                InputCaptureConfig *const capture_config = &input_capture_configs[channel];
                /** Set configs **/
                if (edges[channel] == 'R')
                    capture_config->capture_edge = CAP_RISE;
                else if (edges[channel] == 'F')
                    capture_config->capture_edge = CAP_FALL;
                else
                    capture_config->capture_edge = CAP_BOTH;
            }

            record_command_event(sys_state, &tp, CMD_OK, CONFIG_CAPIN_COMMAND);
            /** Reboot the system **/
            if (save_config_to_flash() != HAL_OK)
                error_handler(ERROR_FLAG_FLASH);
            reset_system();
            return;
        }
    }
    else if (strncmp((const char *const) parser_buffer, "$RTNCFG", 7) == 0) {
        if (payload_len < 6 + 1 + 5) { // at least have return config header
            record_command_event(sys_state, &tp, CMD_FAIL, UNKNOWN_HOST_COMMAND);
            return;
        }

        if (payload_len == 12 && strncmp((const char *const) parser_buffer + 7, ",IPMAC", 6) == 0) {
            record_return_config_event(sys_state, &tp, RETURN_IPMAC_CONFIG, 0);
            return;
        }
        else if (payload_len == 12 && strncmp((const char *const) parser_buffer + 7, ",STIMS", 6) == 0) {
            record_return_config_event(sys_state, &tp, RETURN_SYSTEM_TIMING_SCALING, 0);
            return;
        }
        else if (payload_len == 12 && strncmp((const char *const) parser_buffer + 7, ",CAPIN", 6) == 0){
            record_return_config_event(sys_state, &tp, RETURN_CAPIN_CONFIG, 0);
            return;
        }
        else if (strncmp((const char *const) parser_buffer + 7, ",TIMEC", 6) == 0) {
            uint32_t channel;
            if (sscanf((const char *const) parser_buffer, "$RTNCFG,TIMEC,%2lu*", &channel) == 1) { // NOLINT(*-err34-c)
                if (1 <= channel && channel <= TIMER_CAPTURE_NUM) {
                    record_return_config_event(sys_state, &tp, RETURN_TIMER_CONFIG, channel);
                    return;
                }
            }
        }
        else if (strncmp((const char *const) parser_buffer + 7, ",TRIGR", 6) == 0) {
            uint32_t channel;
            if (sscanf((const char *const) parser_buffer, "$RTNCFG,TRIGR,%2lu*", &channel) == 1) { // NOLINT(*-err34-c)
                if (1 <= channel && channel <= OUTPUT_TRIGGER_NUM) {
                    record_return_config_event(sys_state, &tp, RETURN_TRIGGER_CONFIG, channel);
                    return;
                }
            }
        }
    }
    else if (payload_len == 6 && strncmp((const char *const) parser_buffer, "$CLRERR", 7) == 0) {
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_ERROR_PENDING_FLAG);
        sys_error = 0;
        record_command_event(sys_state, &tp, CMD_OK, CLEAR_ERROR_COMMAND);
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
    record_command_event(sys_state, &tp, CMD_FAIL, UNKNOWN_HOST_COMMAND);
}

/* USER CODE END 1 */
