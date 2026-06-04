/*
 * define for system variable cross all functions
 */

#ifndef INC_SYS_VAR_H_
#define INC_SYS_VAR_H_

#include "stdint.h"
#include "frac.h"
#include "time.h"
#include "trigger.h"
#include "config_constants.h"

extern volatile uint32_t sys_state;
extern uint32_t sys_error;

typedef struct {
    uint32_t min, max;
} Range;
extern Range clks_per_pulse_range;

FORCE_INLINE int in_range(const uint32_t value, const Range range) {
    return range.min <= value && value <= range.max;
}

typedef struct {
    uint32_t ticks_per_second;
    uint32_t pulse_per_second;
    volatile uint32_mixed_frac clks_per_pulse;
} TimingScale_t;
extern TimingScale_t timing_scale;

extern volatile uint32_t clks_since_last_pulse;
extern volatile int32_frac phase_shift_per_tick;

extern uint32_t tick_per_wwdg;

extern volatile uint32_t second_clock_jitter_period;
extern volatile uint64_t tick_pseudo_magic, second_pseudo_magic;

extern volatile SystemMainTime sys_tim;

extern volatile time_t utc_sys_start_timestamp;

extern InputCaptureConfig input_capture_configs[INPUT_CAPTURE_NUM];
extern InputCaptureState input_capture_states[INPUT_CAPTURE_NUM];
extern TimerCaptureConfig timer_capture_configs[TIMER_CAPTURE_NUM];
extern OutputTriggerConfig output_trigger_configs[OUTPUT_TRIGGER_NUM];
extern OutputTriggerState output_trigger_states[OUTPUT_TRIGGER_NUM];

typedef struct {
    uint32_t ip_addr;
    uint32_t ip_mask;
    uint32_t ip_gate;

    uint32_t mac_prefix;
} ETH_ADDR_CONFIG;
extern ETH_ADDR_CONFIG eth_addr_config;

#endif /* INC_SYS_VAR_H_ */
