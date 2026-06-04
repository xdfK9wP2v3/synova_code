#ifndef DIGITAL_PLL_PULSE_HANDLER_H
#define DIGITAL_PLL_PULSE_HANDLER_H

#include "main.h"

FORCE_INLINE void pulse_handler(TIM_HandleTypeDef *htim, RTC_HandleTypeDef *hrtc) {
    const uint32_t captured = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
    const uint32_t prev_clk_per_tick = __HAL_TIM_GET_AUTORELOAD(htim) + 1;
    /*********************************************************************************************************************************************
     * Resolving ambiguity due to update and capture events                                                                                      *
     * if pulse is vary close to TIM update, it's possible to capture pulse just after TIM autoreload while CC IRQ still call before UPDATE IRQ  *
     * ( may because of delay between peripherals and CPU, they fall into same Interpreter Flag change, and CC has high sub-priority)            *
     * by checking if TIM_FLAG_UPDATE isn't clear, while captured is significantly small to detect this case                                     *
     *********************************************************************************************************************************************/
    const uint32_t curr_pulse_tick_phase = ((htim->Instance->SR & TIM_FLAG_UPDATE) == TIM_FLAG_UPDATE && captured < prev_clk_per_tick / 2) ? captured + prev_clk_per_tick : captured;
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);

    /** pulse phase (subtick part) of previous pulse **/
    static __DTCM uint32_t prev_pulse_tick_phase;

    /** pulse statistics for evaluation **/
    static __DTCM uint32_t pulse_stable_count, prev_clks_per_pulse;
    static __DTCM uint32_t pulse_freq_fair_count, pulse_freq_good_count;
    static __DTCM uint32_t pulse_phase_fair_count, pulse_phase_good_count;

    if (LIKELY(FLAG_CHECK(sys_state, SYS_STATE_PULSE_AVAILABLE_FLAG))) { // if it has a pulse before, measure of clock / pulse is available
        /** get observed clock / pulse by difference between previous pulse to current pulse **/
        const uint32_t curr_clks_per_pulse = clks_since_last_pulse + curr_pulse_tick_phase - prev_pulse_tick_phase;

        /** checking pulse interval ( by difference to crystal oscillator theoretical value and previous pulse interval ) **/
        const uint32_t stable_error_tol = prev_clks_per_pulse / PULSE_STABLE_ERROR_RATE + 1;
        if (LIKELY(in_range(curr_clks_per_pulse, clks_per_pulse_range) &&
                   (pulse_stable_count == 0 || (prev_clks_per_pulse - stable_error_tol <= curr_clks_per_pulse && curr_clks_per_pulse <= prev_clks_per_pulse + stable_error_tol)))) {
            /** check if there enough continuous reasonable pulse **/
            if (UNLIKELY(pulse_stable_count <= PULSE_STABLE_THRESHOLD))
                ++pulse_stable_count;
            else {
                ATOMIC_FLAG_SET(sys_state, SYS_STATE_PULSE_STABLE_FLAG);

                /** Update the clocks / pulse by EMA **/
                ExponentialMovingAverage(&timing_scale.clks_per_pulse, curr_clks_per_pulse,
                                         FLAG_CHECK(sys_state, SYS_STATE_PULSE_FREQ_FAIR_FLAG) ? &CLKS_PER_PULSE_UPDATE_RATE : &CLKS_PER_PULSE_FAST_UPDATE_RATE);

                /** save clocks / pulse into backup domain **/
                HAL_RTCEx_BKUPWrite(hrtc, FREQ_INTEGER_ADDR, timing_scale.clks_per_pulse.integer);
                HAL_RTCEx_BKUPWrite(hrtc, FREQ_NUMER_ADDR, timing_scale.clks_per_pulse.frac.numer);
                HAL_RTCEx_BKUPWrite(hrtc, FREQ_DENOM_ADDR, timing_scale.clks_per_pulse.frac.denom);
                HAL_RTCEx_BKUPWrite(hrtc, FREQ_INITIALIZED_ADDR, INITIALIZED_MAGIC_NUMBER);

                /** Update phase shift to be done in later ticks **/
                // TODO: remove phase correct affect on phase error measure
                const uint32_mixed_frac cpp = timing_scale.clks_per_pulse;
                const uint32_t round_clks_per_pulse = mixed_frac_round(&cpp);
                const uint32_t clks_from_second_to_pulse = (sys_tim.clocks_since_prev_second + curr_pulse_tick_phase) % round_clks_per_pulse;
                const int32_t phase_error = (int32_t) clks_from_second_to_pulse - (clks_from_second_to_pulse > round_clks_per_pulse / 2 ? (int32_t) round_clks_per_pulse : 0);
                phase_shift_per_tick.numer = phase_error;
                phase_shift_per_tick.denom = round_frac_mul(FLAG_CHECK(sys_state, SYS_STATE_PULSE_PHASE_FAIR_FLAG) ? &PHASE_CORRECT_SECONDS : &PHASE_CORRECT_FAST_SECONDS,
                                                            timing_scale.ticks_per_second);

                /** record pulse input event **/
                TimePoint tp;
                get_time_point_high_priority(&sys_tim, &tp, curr_pulse_tick_phase, prev_clk_per_tick);
                record_pulse_event(sys_state, &tp,
                                   curr_clks_per_pulse, phase_error, timing_scale.clks_per_pulse.integer,
                                   timing_scale.clks_per_pulse.frac.numer, timing_scale.clks_per_pulse.frac.denom);

                /** Statistics and Evaluation of frequency locking accuracy **/
                const uint32_t abs_freq_error = round_clks_per_pulse > curr_clks_per_pulse ? (round_clks_per_pulse - curr_clks_per_pulse) : (curr_clks_per_pulse - round_clks_per_pulse);
                if (LIKELY(abs_freq_error < round_clks_per_pulse * PULSE_FREQ_FAIR_THRESHOLD / 10000000ul)) {
                    // freq reached fair
                    if (UNLIKELY(pulse_freq_fair_count < PULSE_FREQ_NUM_THRESHOLD))
                        ++pulse_freq_fair_count; // continue counting
                    else {
                        // continue reached fair
                        ATOMIC_FLAG_SET(sys_state, SYS_STATE_PULSE_FREQ_FAIR_FLAG);
                        if (LIKELY(abs_freq_error < round_clks_per_pulse * PULSE_FREQ_GOOD_THRESHOLD / 10000000ul)) {
                            // freq reached fair
                            if (UNLIKELY(pulse_freq_good_count < PULSE_FREQ_NUM_THRESHOLD))
                                ++pulse_freq_good_count; // continue counting
                            else
                                ATOMIC_FLAG_SET(sys_state, SYS_STATE_PULSE_FREQ_GOOD_FLAG); // continue reached good
                        }
                        else {
                            // freq is fair but not good
                            pulse_freq_good_count = 0;
                            ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_FREQ_GOOD_FLAG);
                        }
                    }
                }
                else {
                    // freq is bad
                    pulse_freq_fair_count = 0, pulse_freq_good_count = 0;
                    ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_FREQ_FAIR_FLAG | SYS_STATE_PULSE_FREQ_GOOD_FLAG);
                }

                /** Statistics and Evaluation of phase locking accuracy **/
                const uint32_t abs_phase_error = (uint32_t) ABS(phase_error);
                if (LIKELY(abs_phase_error < round_clks_per_pulse * PULSE_PHASE_FAIR_THRESHOLD / 1000000ul)) {
                    // phase reached fair
                    if (UNLIKELY(pulse_phase_fair_count < PULSE_PHASE_NUM_THRESHOLD))
                        ++pulse_phase_fair_count; // continue counting
                    else {
                        // continue reached fair
                        ATOMIC_FLAG_SET(sys_state, SYS_STATE_PULSE_PHASE_FAIR_FLAG);
                        if (LIKELY(abs_phase_error < round_clks_per_pulse * PULSE_PHASE_GOOD_THRESHOLD / 1000000ul)) {
                            // phase reached fair
                            if (UNLIKELY(pulse_phase_good_count < PULSE_PHASE_NUM_THRESHOLD))
                                ++pulse_phase_good_count; // continue counting
                            else
                                ATOMIC_FLAG_SET(sys_state, SYS_STATE_PULSE_PHASE_GOOD_FLAG); // continue reached good
                        }
                        else {
                            // phase is fair but not good
                            pulse_phase_good_count = 0;
                            ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_PHASE_GOOD_FLAG);
                        }
                    }
                }
                else {
                    // phase is bad
                    pulse_phase_fair_count = 0, pulse_phase_good_count = 0;
                    ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_PHASE_FAIR_FLAG | SYS_STATE_PULSE_PHASE_GOOD_FLAG);
                }
            }
        }
        else {
            /** pulse interval is NOT reasonable, restart pulse tracking **/
            ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_AVAILABLE_FLAG | SYS_STATE_PULSE_STABLE_FLAG);
            ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_FREQ_FAIR_FLAG | SYS_STATE_PULSE_FREQ_GOOD_FLAG);
            ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_PULSE_PHASE_FAIR_FLAG | SYS_STATE_PULSE_PHASE_GOOD_FLAG);
        }

        prev_clks_per_pulse = curr_clks_per_pulse;
    }
    else {
        ATOMIC_FLAG_SET(sys_state, SYS_STATE_PULSE_AVAILABLE_FLAG);
        /************************************************************************
         * When setting PULSE_AVAILABLE, clear all statistics.                  *
         * Because all statistical actions occur after PULSE_AVAILABLE is set,  *
         *         all statistics are cleared when a new pulse is started.      *
         ************************************************************************/
        pulse_stable_count = 0;
        pulse_freq_fair_count = 0, pulse_freq_good_count = 0;
        pulse_phase_fair_count = 0, pulse_phase_good_count = 0;
    }

    /** clear and saving variables for calculate interval between this pulse and next pulse **/
    clks_since_last_pulse = 0;
    prev_pulse_tick_phase = curr_pulse_tick_phase;
}

#endif //DIGITAL_PLL_PULSE_HANDLER_H
