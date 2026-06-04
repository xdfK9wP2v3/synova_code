/*
 * flags for indicating system state
 */

#ifndef INC_SYS_STATES_H_
#define INC_SYS_STATES_H_

// input pulse
#define SYS_STATE_PULSE_AVAILABLE_FLAG (((uint32_t)1) << 0)
#define SYS_STATE_PULSE_STABLE_FLAG (((uint32_t)1) << 1)

// main clock frequency v.s. pulse frequency
#define SYS_STATE_PULSE_FREQ_FAIR_FLAG (((uint32_t)1) << 2)
#define SYS_STATE_PULSE_FREQ_GOOD_FLAG (((uint32_t)1) << 3)

// main clock phase v.s. pulse phase
#define SYS_STATE_PULSE_PHASE_FAIR_FLAG (((uint32_t)1) << 4)
#define SYS_STATE_PULSE_PHASE_GOOD_FLAG (((uint32_t)1) << 5)

// RTC v.s. main clock
#define SYS_STATE_RTC_FREQ_COARSE_SYNC_FLAG (((uint32_t)1) << 6)
#define SYS_STATE_RTC_FREQ_FINE_SYNC_FLAG (((uint32_t)1) << 7)
#define SYS_STATE_RTC_PHASE_SYNC_FLAG (((uint32_t)1) << 8)

// UTC INPUT
#define SYS_STATE_UTC_INPUT_AVAILABLE_FLAG (((uint32_t)1) << 9)
#define SYS_STATE_UTC_INPUT_STABLE_FLAG (((uint32_t)1) << 10)
#define SYS_STATE_UTC_INPUT_TIMESTAMP_STABLE_FLAG (((uint32_t)1) << 11)

// UTC timestamp
#define SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG (((uint32_t)1) << 12)

#define SYS_STATE_SHUTTING_DOWN_FLAG (((uint32_t)1) << 27)

// has a pending error from previous reset
#define SYS_STATE_ERROR_PENDING_FLAG (((uint32_t)1) << 28)

// flash data available
#define SYS_STATE_FLASH_DATA_AVAILABLE (((uint32_t)1) << 29)

// hot start mode (the backup domain didn't reset)
#define SYS_STATE_HOT_START_FLAG (((uint32_t)1) << 30)

// main clock enable
#define SYS_STATE_MAIN_TIM_ENABLE_FLAG (((uint32_t)1) << 31)

#endif /* INC_SYS_STATES_H_ */
