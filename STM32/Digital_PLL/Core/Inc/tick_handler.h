#ifndef DIGITAL_PLL_TICK_HANDLER_H
#define DIGITAL_PLL_TICK_HANDLER_H

#include "main.h"
#include "trigger_handler.h"

FORCE_INLINE uint32_t pseudo_jitter(const uint32_t numer, const uint32_t index, const uint32_t period, const uint32_t pseudo_magic) {
    const uint32_t pseudo = ((uint64_t) (index % period) * pseudo_magic) % period;
    /** pseudo in [0, denom), and (pseudo / denom) ~ U(0, 1) **/
    return (pseudo < numer) ? 1 : 0;
}

FORCE_INLINE uint32_t pseudo_mixed_frac_round(const uint32_mixed_frac *const value, const uint32_t tick_shift) {
    uint32_t round_value = value->integer;

    /** Tick level jitter (1 / timing_scale.ticks_per_second) **/
    const uint32_mixed_frac tick_part = mul_frac(&value->frac, timing_scale.ticks_per_second);
    round_value += pseudo_jitter(tick_part.integer, sys_tim.tick + tick_shift, timing_scale.ticks_per_second, tick_pseudo_magic);

    /** Second level jitter (1 / (second_clock_jitter_period * timing_scale.ticks_per_second)) **/
    const uint32_mixed_frac second_part = mul_frac(&tick_part.frac, second_clock_jitter_period);
    round_value += pseudo_jitter(mixed_frac_round(&second_part),
                                 (sys_tim.second % second_clock_jitter_period) * timing_scale.ticks_per_second + sys_tim.tick + tick_shift,
                                 second_clock_jitter_period * timing_scale.ticks_per_second, second_pseudo_magic);

    return round_value;
}

FORCE_INLINE uint32_t get_next_clk_per_tick(const uint32_t prev_clk_per_tick) {
    // accumulate clks_since_last_pulse if pulse is available
    if (LIKELY(FLAG_CHECK(sys_state, SYS_STATE_PULSE_AVAILABLE_FLAG)))
        clks_since_last_pulse += prev_clk_per_tick;

    // pulse become unavailable since timeout
    if (UNLIKELY(clks_since_last_pulse > clks_per_pulse_range.max)) {
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_AVAILABLE_FLAG | SYS_STATE_PULSE_STABLE_FLAG);
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_FREQ_FAIR_FLAG | SYS_STATE_PULSE_FREQ_GOOD_FLAG);
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_PHASE_FAIR_FLAG | SYS_STATE_PULSE_PHASE_GOOD_FLAG);
    }

    // clock / second == (clock / pulse) * (pulse / second)
    const uint32_mixed_frac clks_per_second = mixed_frac_mul(&timing_scale.clks_per_pulse, timing_scale.pulse_per_second);
    // clock / tick == (clock / second) / (tick / second)
    const uint32_mixed_frac clks_per_tick = div_mixed_frac(&timing_scale.clks_per_pulse, timing_scale.ticks_per_second);

    /** get the clock per tick, with jitter for the fraction part **/
    uint32_t next_clk_per_tick = pseudo_mixed_frac_round(&clks_per_tick, 0);


    /** shift clock phase by add or subtract some clock per tick **/
    // if no more phase shift, clear remaining tick for phase shifting
    if (UNLIKELY(phase_shift_per_tick.numer == 0))
        phase_shift_per_tick.denom = 0;
    if (LIKELY(phase_shift_per_tick.denom > 0)) {
        const uint32_mixed_frac abs_phase_shift_per_tick = frac_abs(&phase_shift_per_tick);
        uint32_t abs_phase_shift = pseudo_mixed_frac_round(&abs_phase_shift_per_tick, timing_scale.ticks_per_second * 3 / 2);

        // get max phase shift for tick (avoid too short or too long tick)
        const uint32_t max_shift_per_tick = round_frac_mul(&MAX_CLKS_SHIFT_PER_TICK_RATE, clks_per_tick.integer) + 1;
        if (abs_phase_shift >= max_shift_per_tick)  // limit max shift per tick
            abs_phase_shift = max_shift_per_tick;
        else
            --phase_shift_per_tick.denom; // reduce the remaining tick only if shift is in the range

        const int32_t phase_shift = phase_shift_per_tick.numer > 0 ? (int32_t) abs_phase_shift : -(int32_t) abs_phase_shift;
        next_clk_per_tick += phase_shift, phase_shift_per_tick.numer -= phase_shift;
    }
    return next_clk_per_tick;
}

FORCE_INLINE void captures_handler(const uint32_t prev_clk_per_tick, const SubTickState subTickState) {
    for (uint32_t i = 0; i < INPUT_CAPTURE_NUM; ++i)
        input_capture_handler(&input_capture_configs[i], &input_capture_states[i], prev_clk_per_tick, subTickState);
}

FORCE_INLINE void trigger_handler(const uint32_t next_clk_per_tick, const TIM_HandleTypeDef *const main_tim) {
    uint64_t capture_signals[TRIGGER_SIGNAL_SIZE] = {0};
    // set up capture signals
    for (uint32_t i = 0; i < INPUT_CAPTURE_NUM; ++i)
        input_capture_flag_setter(capture_signals, 0 + 4 * i, &input_capture_states[i], i + 1);
    for (uint32_t i = 0; i < TIMER_CAPTURE_NUM; ++i)
        timer_capture_flag_setter(capture_signals, 192 + 1 * i, &timer_capture_configs[i]);
    for (uint32_t i = 0; i < OUTPUT_TRIGGER_NUM; ++i)
        output_capture_flag_setter(capture_signals, 128 + 2 * i, &output_trigger_states[i]);

    // TODO: handle LogicRelay

    // output
    for (uint32_t i = 0; i < OUTPUT_TRIGGER_NUM; ++i)
        output_trigger_handler(&output_trigger_configs[i], &output_trigger_states[i], capture_signals, i + 1, next_clk_per_tick, main_tim);
}

#endif //DIGITAL_PLL_TICK_HANDLER_H
