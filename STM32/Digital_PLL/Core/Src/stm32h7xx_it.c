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
#include "wwdg.h"
#include "iwdg.h"
#include "pulse_handler.h"
#include "tick_handler.h"
#include "trigger_handler.h"
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
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern LPTIM_HandleTypeDef hlptim1;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi5;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern WWDG_HandleTypeDef hwwdg1;
/* USER CODE BEGIN EV */
extern uint32_t utc_input_stable_wdg;
extern TIM_HandleTypeDef MAIN_TIM;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {
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
void HardFault_Handler(void) {
    /* USER CODE BEGIN HardFault_IRQn 0 */
    error_handler(ERROR_FLAG_HardFault);
    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */
    error_handler(ERROR_FLAG_MemManage);
    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void) {
    /* USER CODE BEGIN BusFault_IRQn 0 */
    error_handler(ERROR_FLAG_BusFault);
    /* USER CODE END BusFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {
    /* USER CODE BEGIN UsageFault_IRQn 0 */
    error_handler(ERROR_FLAG_UsageFault);
    /* USER CODE END UsageFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void) {
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void) {
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void) {
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void) {
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
  * @brief This function handles Window watchdog interrupt.
  */
void WWDG_IRQHandler(void) {
    /* USER CODE BEGIN WWDG_IRQn 0 */

    // ignore wwdg during shutting down mode
    if (UNLIKELY(FLAG_CHECK(sys_state, SYS_STATE_SHUTTING_DOWN_FLAG)))
        HAL_WWDG_Refresh(&hwwdg1);
    /* USER CODE END WWDG_IRQn 0 */
    HAL_WWDG_IRQHandler(&hwwdg1);
    /* USER CODE BEGIN WWDG_IRQn 1 */

    /* USER CODE END WWDG_IRQn 1 */
}

/**
  * @brief This function handles RTC wake-up interrupt through EXTI line 19.
  */
void RTC_WKUP_IRQHandler(void) {
    /* USER CODE BEGIN RTC_WKUP_IRQn 0 */

    /* USER CODE END RTC_WKUP_IRQn 0 */
    HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
    /* USER CODE BEGIN RTC_WKUP_IRQn 1 */

    /* USER CODE END RTC_WKUP_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void) {
    /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

    /* USER CODE END DMA1_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
    /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

    /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void) {
    /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

    /* USER CODE END DMA1_Stream1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
    /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

    /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void) {
    /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

    /* USER CODE END DMA1_Stream2_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
    /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

    /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void) {
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void) {
    /* USER CODE BEGIN USART3_IRQn 0 */

    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */

    /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles SPI5 global interrupt.
  */
void SPI5_IRQHandler(void) {
    /* USER CODE BEGIN SPI5_IRQn 0 */

    /* USER CODE END SPI5_IRQn 0 */
    HAL_SPI_IRQHandler(&hspi5);
    /* USER CODE BEGIN SPI5_IRQn 1 */

    /* USER CODE END SPI5_IRQn 1 */
}

/**
  * @brief This function handles LPTIM1 global interrupt.
  */
void LPTIM1_IRQHandler(void) {
    /* USER CODE BEGIN LPTIM1_IRQn 0 */
#ifdef ENABLE_WDG
    static uint32_t feed_dog_cnt = 0;
    if (feed_dog_cnt == 0) {
        HAL_IWDG_Refresh(&hiwdg1);
        feed_dog_cnt = 2 * SOFTWARE_WDG_RATE;
    }
    --feed_dog_cnt;
#endif

    if (UNLIKELY(!FLAG_CHECK(sys_state, SYS_STATE_MAIN_TIM_ENABLE_FLAG))) {
        static uint32_t main_timer_start_countdown = 2 * SOFTWARE_WDG_RATE;
        if (--main_timer_start_countdown == 0)
            error_handler(ERROR_FLAG_MAIN_TIMER_START_FAIL);
    }
    else {
        if (tick_wdg > 0)
            --tick_wdg;
        else
            error_handler(ERROR_FLAG_PCLK_ERROR);
    }

    if (rtc_wdg > 0)
        --rtc_wdg;
    else
        error_handler(ERROR_FLAG_RTC);

    if (msg_wdg > 0)
        --msg_wdg;
    else
        error_handler(ERROR_FLAG_MSG_OVERTIME);

    if (utc_input_stable_wdg > 0)
        --utc_input_stable_wdg;
    else {
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_INPUT_AVAILABLE_FLAG);
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_INPUT_STABLE_FLAG);
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_INPUT_TIMESTAMP_STABLE_FLAG);
    }

    /* LED state output */
    static uint16_t LED_state;
    LED_state = 0;
    if (LIKELY(FLAG_CHECK(sys_state, SYS_STATE_MAIN_TIM_ENABLE_FLAG))) {
        int blank = (sys_tim.tick / (timing_scale.ticks_per_second / 10)) % 2 == 0;

        // second
        if (sys_tim.tick < timing_scale.ticks_per_second / 10)
            LED_state |= 1 << 0;
        // UTC timestamp
        if (FLAG_CHECK(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG))
            LED_state |= 1 << 1;
        // pulse input
        if (FLAG_CHECK(sys_state, SYS_STATE_PULSE_STABLE_FLAG) || (FLAG_CHECK(sys_state, SYS_STATE_PULSE_AVAILABLE_FLAG) & blank))
            LED_state |= 1 << 2;
        if (FLAG_CHECK(sys_state, SYS_STATE_PULSE_STABLE_FLAG)) {
            // freq evaluate
            if (FLAG_CHECK(sys_state, SYS_STATE_PULSE_FREQ_GOOD_FLAG))
                LED_state |= 1 << 4;
            else if (FLAG_CHECK(sys_state, SYS_STATE_PULSE_FREQ_FAIR_FLAG))
                LED_state |= 1 << 5;
            else
                LED_state |= 1 << 3;
            // phase evaluate
            if (FLAG_CHECK(sys_state, SYS_STATE_PULSE_PHASE_GOOD_FLAG))
                LED_state |= 1 << 7;
            else if (FLAG_CHECK(sys_state, SYS_STATE_PULSE_PHASE_FAIR_FLAG))
                LED_state |= 1 << 8;
            else
                LED_state |= 1 << 6;
        }
        // RTC
        if (FLAG_CHECK(sys_state, SYS_STATE_RTC_FREQ_COARSE_SYNC_FLAG)) {
            if (FLAG_CHECK(sys_state, SYS_STATE_RTC_FREQ_FINE_SYNC_FLAG)) {
                if (FLAG_CHECK(sys_state, SYS_STATE_RTC_PHASE_SYNC_FLAG) || blank)
                    LED_state |= 1 << 10;
            }
            else if (FLAG_CHECK(sys_state, SYS_STATE_RTC_PHASE_SYNC_FLAG) || blank)
                LED_state |= 1 << 11;
        }
        else if (!FLAG_CHECK(sys_state, SYS_STATE_RTC_PHASE_SYNC_FLAG) || blank)
            LED_state |= 1 << 9;
        // UTC input
        if (FLAG_CHECK(sys_state, SYS_STATE_UTC_INPUT_TIMESTAMP_STABLE_FLAG))
            LED_state |= 1 << 13;
        else if (FLAG_CHECK(sys_state, SYS_STATE_UTC_INPUT_STABLE_FLAG))
            LED_state |= 1 << 14;
        else if (!FLAG_CHECK(sys_state, SYS_STATE_UTC_INPUT_AVAILABLE_FLAG) || blank)
            LED_state |= 1 << 12;
        // Error
        if (FLAG_CHECK(sys_state, SYS_STATE_ERROR_PENDING_FLAG))
            LED_state |= 1 << 15;
    }
    LED_state = ~LED_state;
    HAL_SPI_Transmit_IT(&SYS_STATE_SPI, (uint8_t *) &LED_state, 1);

    /* USER CODE END LPTIM1_IRQn 0 */
    HAL_LPTIM_IRQHandler(&hlptim1);
    /* USER CODE BEGIN LPTIM1_IRQn 1 */

    /* USER CODE END LPTIM1_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void) {
    /* USER CODE BEGIN OTG_FS_IRQn 0 */

    /* USER CODE END OTG_FS_IRQn 0 */
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
    /* USER CODE BEGIN OTG_FS_IRQn 1 */

    /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles TIM1 update interrupt.
  */
__ITCM void TIM1_UP_IRQHandler(void) {
    // clear the interrupt flag bit (allows new interrupt input during following process)
    __HAL_TIM_CLEAR_IT(&MAIN_TIM, TIM_IT_UPDATE);

    const uint32_t prev_clk_per_tick = __HAL_TIM_GET_AUTORELOAD(&MAIN_TIM) + 1;

    if (LIKELY(FLAG_CHECK(sys_state, SYS_STATE_MAIN_TIM_ENABLE_FLAG))) {
        // handle all input capture events
        captures_handler(prev_clk_per_tick, FULL_TICK);

#ifdef ENABLE_WDG
        if (tick_per_wwdg > 0 && ((sys_tim.second % tick_per_wwdg) * (timing_scale.ticks_per_second % tick_per_wwdg) + sys_tim.tick + 1) % tick_per_wwdg == 0)
            HAL_WWDG_Refresh(&hwwdg1);
#endif

        // update the time variable
        const uint32_t next_tick_idx = (sys_tim.tick + 1) % timing_scale.ticks_per_second;
        if (UNLIKELY(next_tick_idx == 0)) {
            TimePoint tp = {.second = sys_tim.second + 1, .tick = 0, .subtick = 0, .tick_fraction = prev_clk_per_tick,
                    .clocks_since_prev_second = sys_tim.clocks_since_prev_second + prev_clk_per_tick};
            record_second_event(sys_state, &tp, sys_tim.second);
            ++sys_tim.second;
            sys_tim.clocks_since_prev_second = 0;
            if (FLAG_CHECK(sys_state, SYS_STATE_ERROR_PENDING_FLAG)) {
                tp.clocks_since_prev_second = 0;
                record_pending_error_event(sys_state, &tp);
            }
        }
        else
            sys_tim.clocks_since_prev_second += prev_clk_per_tick;
        sys_tim.tick = next_tick_idx;
    }
    else
        ATOMIC_FLAG_SET(sys_state, SYS_STATE_MAIN_TIM_ENABLE_FLAG); // initial timer

    tick_wdg = SOFTWARE_WDG_RATE;

    // get new tick clock count
    uint32_t next_clk_per_tick = get_next_clk_per_tick(prev_clk_per_tick);

    trigger_handler(next_clk_per_tick, &MAIN_TIM);

    // setup counter for next tick
    if (UNLIKELY(__HAL_TIM_GET_COUNTER(&MAIN_TIM) + TICK_SAFE_THRESHOLD >= next_clk_per_tick / 2 + 1))
        error_handler(ERROR_FLAG_TICK_OVERTIME);
    __HAL_TIM_SET_AUTORELOAD(&MAIN_TIM, next_clk_per_tick - 1);
    __HAL_TIM_SET_COMPARE(&MAIN_TIM, TIM_CHANNEL_2, next_clk_per_tick / 2 + 1);
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
__ITCM void TIM1_CC_IRQHandler(void) {
    // wait main clock start
    if (UNLIKELY(!FLAG_CHECK(sys_state, SYS_STATE_MAIN_TIM_ENABLE_FLAG))) {
        __HAL_TIM_CLEAR_IT(&MAIN_TIM, TIM_FLAG_CC2 | TIM_FLAG_CC1);
        return;
    }

    if (LIKELY(__HAL_TIM_GET_FLAG(&MAIN_TIM, TIM_FLAG_CC1)))  // pulse input
        pulse_handler(&MAIN_TIM, &hrtc);

    if (UNLIKELY(__HAL_TIM_GET_FLAG(&MAIN_TIM, TIM_FLAG_CC2))) { // half-tick handler
        const uint32_t prev_clk_per_tick = __HAL_TIM_GET_AUTORELOAD(&MAIN_TIM) + 1;
        captures_handler(prev_clk_per_tick, HALF_TICK);
        __HAL_TIM_CLEAR_IT(&MAIN_TIM, TIM_FLAG_CC2);
    }
}
/* USER CODE END 1 */
