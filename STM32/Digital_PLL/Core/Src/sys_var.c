#include "sys_var.h"
#include "mem_helper.h"
#include "config_constants.h"

volatile __DTCM uint32_t sys_state = 0;
uint32_t sys_error = 0;

__DTCM Range clks_per_pulse_range;

__DTCM TimingScale_t timing_scale;

/* pulse variable */
volatile __DTCM uint32_t clks_since_last_pulse;     // clock since last pulse input

/* tick phase correction */
volatile __DTCM int32_frac phase_shift_per_tick;

__DTCM uint32_t tick_per_wwdg;

volatile __DTCM uint32_t second_clock_jitter_period;
volatile __DTCM uint64_t tick_pseudo_magic, second_pseudo_magic;

/* global time variable */
volatile __DTCM SystemMainTime sys_tim;

volatile __DTCM time_t utc_sys_start_timestamp;

__DTCM InputCaptureConfig input_capture_configs[INPUT_CAPTURE_NUM];
__DTCM InputCaptureState input_capture_states[INPUT_CAPTURE_NUM];
__DTCM TimerCaptureConfig timer_capture_configs[TIMER_CAPTURE_NUM];
__DTCM OutputTriggerConfig output_trigger_configs[OUTPUT_TRIGGER_NUM];
__DTCM OutputTriggerState output_trigger_states[OUTPUT_TRIGGER_NUM];

ETH_ADDR_CONFIG eth_addr_config;
