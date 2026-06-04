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
#include <stdint.h>
#include <stdarg.h>

#include "frac.h"
#include "error.h"
#include "sys_var.h"
#include "event_msg.h"
#include "help_macro.h"
#include "mem_helper.h"
#include "sys_states.h"
#include "system_time.h"
#include "config_constants.h"
#include "peripheral_mapping.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* GCC built in method */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//#define ENABLE_DEBUG_MSG

#ifdef ENABLE_DEBUG_MSG

#define DEBUG_MSG_BUFFER_SIZE 512

void debug_printf(const char *format, ...);

#endif
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WWDG_WIN_SIZE 4
#define ALIVE_Pin GPIO_PIN_8
#define ALIVE_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */
#define ENABLE_WDG

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
