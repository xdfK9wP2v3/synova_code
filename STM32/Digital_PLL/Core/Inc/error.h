#ifndef DIGITAL_PLL_ERROR_H
#define DIGITAL_PLL_ERROR_H

#define ERROR_FLAG_NMI (1U << 0)
#define ERROR_FLAG_HardFault (1U << 1)
#define ERROR_FLAG_MemManage (1U << 2)
#define ERROR_FLAG_BusFault (1U << 3)
#define ERROR_FLAG_UsageFault (1U << 4)
#define ERROR_FLAG_HAL_ERROR (1U << 5)
#define ERROR_FLAG_PCLK_ERROR (1U << 6)
#define ERROR_FLAG_MAIN_TIMER_START_FAIL (1U << 7)
#define ERROR_FLAG_TICK_OVERTIME (1U << 8)
#define ERROR_FLAG_FLASH (1U << 9)
#define ERROR_FLAG_RTC (1U << 10)
#define ERROR_FLAG_UART (1U << 11)
#define ERROR_FLAG_MSG_OVERTIME (1U << 12)
#define ERROR_FLAG_ETH (1U << 13)

#define ERROR_FLAG_BORRST (1U << 29)
#define ERROR_FLAG_IWDG (1U << 30)
#define ERROR_FLAG_WWDG (1U << 31)

void shutdown_mode();

void reset_system();

void check_error();

void error_handler(uint32_t error_flag);

#endif //DIGITAL_PLL_ERROR_H
