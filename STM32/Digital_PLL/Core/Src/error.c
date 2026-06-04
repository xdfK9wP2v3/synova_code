#include "main.h"
#include "rtc.h"
#include "iwdg.h"

void shutdown_mode() {
    __set_BASEPRI(0x01 << 1);
#ifdef ENABLE_WDG
    HAL_IWDG_Refresh(&hiwdg1);
#endif
    ATOMIC_FLAG_SET(sys_state, SYS_STATE_SHUTTING_DOWN_FLAG);
}

void reset_system(){
    __set_FAULTMASK(1); // close all interrupts (include hard fault)
    HAL_NVIC_SystemReset();
}

void check_error() {
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) {
        FLAG_SET(sys_state, SYS_STATE_ERROR_PENDING_FLAG);
        FLAG_SET(sys_error, ERROR_FLAG_BORRST);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != RESET) {
        FLAG_SET(sys_state, SYS_STATE_ERROR_PENDING_FLAG);
        FLAG_SET(sys_error, ERROR_FLAG_IWDG);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST) != RESET) {
        FLAG_SET(sys_state, SYS_STATE_ERROR_PENDING_FLAG);
        FLAG_SET(sys_error, ERROR_FLAG_WWDG);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) {
        const uint32_t error_flag = HAL_RTCEx_BKUPRead(&hrtc, ERROR_FLAGS_ADDR);
        if (error_flag != 0) {
            FLAG_SET(sys_state, SYS_STATE_ERROR_PENDING_FLAG);
            FLAG_SET(sys_error, error_flag);
        }
    }

    // clear all error flags after read out
    HAL_RTCEx_BKUPWrite(&hrtc, ERROR_FLAGS_ADDR, 0);
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void error_handler(const uint32_t error_flag) {
    __set_FAULTMASK(1); // close all interrupt (include hard fault, for reset)
    HAL_RTCEx_BKUPWrite(&hrtc, ERROR_FLAGS_ADDR, error_flag);
    HAL_NVIC_SystemReset();       // system reset
}
