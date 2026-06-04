#ifndef INC_CONFIG_CONSTANTS_H_
#define INC_CONFIG_CONSTANTS_H_

#include "frac.h"

#define TICK_SAFE_THRESHOLD 128

// Buffer size
#define UART_DMA_BUFFER_SIZE 256

// the max error rate of main timer and input pulse
#define CRYSTAL_OSCILLATOR_ERROR_RATE 1000
#define PULSE_STABLE_ERROR_RATE 10000000ul

// Pulse and NMEA message time difference (in ms)
#define PULSE_NMEA_DELAY 200
#define PULSE_NMEA_ADVANCE 100

// number of continue Pulse for considering input pulse is stable
#define PULSE_STABLE_THRESHOLD 3

// Pulse frequency update rate (MEA)
extern const uint32_frac CLKS_PER_PULSE_UPDATE_RATE, CLKS_PER_PULSE_FAST_UPDATE_RATE;

// Pulse v.s. Main Clock phase correcting rate (expect finish time in second)
extern const uint32_frac PHASE_CORRECT_SECONDS, PHASE_CORRECT_FAST_SECONDS;

// Pulse v.s. Main Clock phase correcting max clock per tick (rate of a tick length)
extern const uint32_frac MAX_CLKS_SHIFT_PER_TICK_RATE;

// Pulse v.s. Main Clock frequency evaluate
#define PULSE_FREQ_NUM_THRESHOLD 10
#define PULSE_FREQ_FAIR_THRESHOLD 50  // unit in 0.1ppm
#define PULSE_FREQ_GOOD_THRESHOLD 2   // unit in 0.1ppm

// Pulse v.s. Main Clock phase evaluate
#define PULSE_PHASE_NUM_THRESHOLD 10
#define PULSE_PHASE_FAIR_THRESHOLD 50 // unit in 1ppm
#define PULSE_PHASE_GOOD_THRESHOLD 2  // unit in 1ppm

// RTC
#define RTC_SYNC_CHECK_PERIOD 2             // period to check RTC sync (frequency & phase), also the sampling time for coarse calibration in second
#define RTC_FINE_CALIBRATION_PERIOD 500     // minimal sampling time for calculating calibration in second
#define RTC_MAX_PHASE_ERROR_RATE 1000       // the max phase error of RTC accept is 1s / RTC_MAX_PHASE_ERROR_RATE
#define RTC_FREQ_COARSE_NUM_THRESHOLD 2     // evaluate threshold, unit in RTC_SYNC_CHECK_PERIOD
#define RTC_FREQ_FINE_SECOND_THRESHOLD 2000 // evaluate threshold, unit in second: error rate <= 1 / RTC_MAX_PHASE_ERROR_RATE / RTC_FREQ_FINE_SECOND_THRESHOLD
#define RTC_PHASE_SECOND_THRESHOLD 4        // evaluate threshold, unit in second
#define RTC_PHASE_FORCE_SYNC_THRESHOLD 10   // max difference between main timer and RTC in second
#define RTC_IDLE_REPORT_INTERVAL 30         // send a message to host, even if no event (report the RTC data)

// UTC input
#define UTC_INPUT_TIMEOUT 2                         // max time between two valid UTC time input
#define UTC_INPUT_STABLE_THRESHOLD 3                // number of continue UTC input for considering UTC input is stable
#define UTC_INPUT_TIMESTAMP_STABLE_THRESHOLD 10     // number of consistent UTC input for considering timestamp is stable

// backup domain register address mapping
#define INITIALIZED_MAGIC_NUMBER 0xa5a5a5a5
#define HOT_START_ADDR 0
#define UTC_WORK_ADDR 1
#define FREQ_INITIALIZED_ADDR 2
#define FREQ_INTEGER_ADDR 3
#define FREQ_NUMER_ADDR 4
#define FREQ_DENOM_ADDR 5
#define FREQ_PPS_ADDR 6
#define ERROR_FLAGS_ADDR 31

// Software WDG calling rate per second
#define SOFTWARE_WDG_RATE 32

// Components number
#define INPUT_CAPTURE_NUM 12
#define TIMER_CAPTURE_NUM 32
#define OUTPUT_TRIGGER_NUM 12
#define TRIGGER_SIGNAL_SIZE 5

// Input Capture states flags
#define IC_PREV_IO_HIGH_LEVEL_FLAG (1U << 0)
#define IC_RISE_FLAG (1U << 1)
#define IC_OVER_CAPTURE_FLAG (1U << 2)
#define IC_CAPTURE_FLAG (1U << 3)
#define IC_PENDING_FLAG (1U << 4)

// Timer Capture config flags
#define TC_SECOND_ALIGN (1U << 0)
#define TC_DROP_LAST (1U << 1)

// Output Trigger config flags
#define OT_ACTIVATE_EXPEND_FLAG (1ul << 0)
#define OT_ACTIVATE_FALL_PRIORITY_FLAG (1ul << 1)
#define OT_DEADZ_ALLOW_PEND_FLAG (1ul << 2)
#define OT_DEADZ_PEND_FALL_PRIORITY_FLAG (1ul << 3)
#define OT_DEADZ_PEND_FIRST_PRIORITY_FLAG (1ul << 4)
#define OT_POLARITY_LOW (1ul << 5)

// Flash config
#define CONFIG_VERSION (~2)
#define CONFIG_BANK FLASH_BANK_2
#define CONFIG_SECTOR FLASH_SECTOR_7
#define CONFIG_BASE_ADDR ((uint32_t) 0x081E0000)
#define FLASH_LINE_WIDTH (256 / 8) // flash is 256bits / group = 32bytes / group

#endif /* INC_CONFIG_CONSTANTS_H_ */
