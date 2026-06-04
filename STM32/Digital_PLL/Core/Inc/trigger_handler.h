

#ifndef DIGITAL_PLL_TRIGGER_HANDLER_H
#define DIGITAL_PLL_TRIGGER_HANDLER_H

#include "sys_var.h"
#include "event_msg.h"

typedef enum {
    FULL_TICK = 0,
    HALF_TICK = 1
} SubTickState;

FORCE_INLINE void input_capture_handler(const InputCaptureConfig *const config, InputCaptureState *const state,
                                        const uint32_t prev_clk_per_tick, const SubTickState subTickState) {
    // get TIMER struct from the config
    const TIM_HandleTypeDef *const TIMx = config->interface.htim;
    const uint32_t CHANNELx = get_TIM_CHANNEL_x(config->interface.channel);
    const uint32_t CCxIF = get_TIM_CCxIF(config->interface.channel);
    const uint32_t CCxOF = get_TIM_CCxOF(config->interface.channel);

    GPIO_TypeDef *const GPIOx = config->interface.gpio_group;
    const uint16_t PINx = config->interface.gpio_pin;

    assert_param(!(!FLAG_CHECK(state->flags, IC_CAPTURE_FLAG) && FLAG_CHECK(state->flags, IC_OVER_CAPTURE_FLAG)));
    assert_param(!(subTickState == HALF_TICK && FLAG_CHECK(state->flags, IC_CAPTURE_FLAG)));                // read & clear capture event at full-tick
    assert_param(!(subTickState == HALF_TICK && FLAG_CHECK(state->flags, IC_OVER_CAPTURE_FLAG)));           // read & clear capture event at full-tick
    assert_param(!(subTickState == FULL_TICK && FLAG_CHECK(state->flags, IC_PENDING_FLAG)));                // pending only handle at half-tick

    const int has_new_capture = __HAL_TIM_GET_FLAG(TIMx, CCxIF);
    const uint32_t capture_reg = has_new_capture ? __HAL_TIM_GET_COMPARE(TIMx, CHANNELx) : -1; // NOTICE: reading CCRx would clear CCxIF

    if (has_new_capture && subTickState == FULL_TICK && capture_reg < prev_clk_per_tick / 2) {
        // pending capture to next tick
        FLAG_SET(state->flags, IC_PENDING_FLAG);
        state->pending_capture = capture_reg;
    }
    else if (has_new_capture || FLAG_CHECK(state->flags, IC_PENDING_FLAG)) {
        if (has_new_capture && (FLAG_CHECK(state->flags, IC_CAPTURE_FLAG) || FLAG_CHECK(state->flags, IC_PENDING_FLAG)))
            FLAG_SET(state->flags, IC_OVER_CAPTURE_FLAG);  // if prev record didn't clear, current capture is a over capture

        if (!FLAG_CHECK(state->flags, IC_CAPTURE_FLAG)) {
            // check if there is a hardware over capture
            if (__HAL_TIM_GET_FLAG(TIMx, CCxOF)) {
                FLAG_SET(state->flags, IC_OVER_CAPTURE_FLAG);
                __HAL_TIM_CLEAR_IT(TIMx, CCxOF);
            }

            // get capture value (if is a pending capture, load from state)
            const uint32_t capture_val = FLAG_CHECK(state->flags, IC_PENDING_FLAG) ? state->pending_capture : capture_reg;

            // get capture time point
            get_time_point_high_priority(&sys_tim, &state->time_point, capture_val, prev_clk_per_tick);

            // get current edge by prev pin state
            if (config->capture_edge == CAP_BOTH)
                FLAG_EDIT(state->flags, IC_RISE_FLAG, !FLAG_CHECK(state->flags, IC_PREV_IO_HIGH_LEVEL_FLAG));
            else
                FLAG_EDIT(state->flags, IC_RISE_FLAG, config->capture_edge == CAP_RISE);

            FLAG_CLEAR(state->flags, IC_PENDING_FLAG);
            FLAG_SET(state->flags, IC_CAPTURE_FLAG);
        }

        // update the IC_PREV_IO_HIGH_LEVEL_FLAG
        if (config->capture_edge == CAP_BOTH) {
            do {
                __HAL_TIM_CLEAR_IT(TIMx, CCxOF | CCxIF); // force clear flag to ensure the IC_PREV_IO_HIGH_LEVEL_FLAG is the last edge
                FLAG_EDIT(state->flags, IC_PREV_IO_HIGH_LEVEL_FLAG, HAL_GPIO_ReadPin(GPIOx, PINx) == GPIO_PIN_SET);
                if (__HAL_TIM_GET_FLAG(TIMx, CCxIF)) {
                    FLAG_SET(state->flags, IC_OVER_CAPTURE_FLAG);
                    continue; // NOLINT(*-terminating-continue)
                }
            } while (0);
        }
    }
}

FORCE_INLINE void input_capture_flag_setter(uint64_t *const flags, const uint8_t position, InputCaptureState *const state, uint32_t channel) {
    // assume (position % 64) + 4 <= 64
    flags[position / 64] &= ~(0xF << (position % 64)); // clear previous flags
    if (FLAG_CHECK(state->flags, IC_CAPTURE_FLAG)) {
        EdgeType edge = FLAG_CHECK(state->flags, IC_RISE_FLAG) ?
                        (FLAG_CHECK(state->flags, IC_OVER_CAPTURE_FLAG) ? RAISE_EDGE_AND_UNKNOWN_EDGES : RAISE_EDGE) :
                        (FLAG_CHECK(state->flags, IC_OVER_CAPTURE_FLAG) ? FALL_EDGE_AND_UNKNOWN_EDGES : FALL_EDGE);

        const uint64_t edge_mask_flag[] = {
                0x01,       // RAISE_EDGE                       (direct RISE edge capture)
                0x02,       // FALL_EDGE                        (direct FALL edge capture)
                0x8 | 0x1,  // RAISE_EDGE_AND_UNKNOWN_EDGES     (direct RISE edge capture + indirect FALL edge capture)
                0x4 | 0x2   // FALL_EDGE_AND_UNKNOWN_EDGES      (direct FALL edge capture + indirect RISE edge capture)
        };
        flags[position / 64] |= edge_mask_flag[edge] << (position % 64);
        FLAG_CLEAR(state->flags, IC_CAPTURE_FLAG | IC_OVER_CAPTURE_FLAG);
        record_input_capture_event(sys_state, &state->time_point, channel, edge);
    }
}

FORCE_INLINE void timer_capture_flag_setter(uint64_t *const flags, const uint8_t position, const TimerCaptureConfig *const config) {
    const uint32_t divider = config->pre_divider + 1;

    int signal = 0;

    if (FLAG_CHECK(config->flags, TC_SECOND_ALIGN)) {
        const int32_t shift = config->shift % (int32_t) timing_scale.ticks_per_second;
        const int64_t remainder = ((int64_t) sys_tim.tick + (int64_t) timing_scale.ticks_per_second + shift) % timing_scale.ticks_per_second;
        signal = (remainder % divider == 0);
        if (signal && FLAG_CHECK(config->flags, TC_DROP_LAST)) {
            const uint32_t critical_tick = (timing_scale.ticks_per_second - shift) % timing_scale.ticks_per_second;
            signal = (critical_tick <= sys_tim.tick ? timing_scale.ticks_per_second : 0) + critical_tick - sys_tim.tick >= divider;
        }
    }
    else {
        // equal to (sys_tim.second * timing_scale.ticks_per_second + sys_tim.tick + shift) % divider == 0
        const int32_t shift = config->shift % (int32_t) divider;
        const uint32_t second_shift = (((uint64_t) sys_tim.second % divider) * (timing_scale.ticks_per_second % divider)) % divider;
        const uint32_t remainder = second_shift + sys_tim.tick % divider;
        signal = (((int32_t) remainder + divider + shift) % divider == 0);
    }

    if (signal)
        FLAG_SET(flags[position / 64], 1 << (position % 64));
}

FORCE_INLINE void output_capture_flag_setter(uint64_t *const flags, const uint8_t position, const OutputTriggerState *const trigger) {
    // assume (position % 64) + 2 <= 64
    flags[position / 64] &= ~(0x3 << (position % 64)); // clear previous flags
    if (trigger->state == TRIGGER_RISE)
        flags[position / 64] |= 0x1 << (position % 64);
    if (trigger->state == TRIGGER_FALL)
        flags[position / 64] |= 0x2 << (position % 64);
}

FORCE_INLINE void setup_trigger_physics_io(const uint32_t next_clk_per_tick, const uint32_t channel, const TIM_HandleTypeDef *const main_tim,
                                           const OutputTriggerConfig *const config, const EdgeType edge) {
    assert_param(edge == RAISE_EDGE || edge == FALL_EDGE);

    TimePoint tp;
    get_time_point_high_priority(&sys_tim, &tp, 0, next_clk_per_tick);
    record_trigger_output_event(sys_state, &tp, channel, edge, config->pulse.rf_compensation);

    if (config->interface.gpio_group != NULL && config->interface.htim != NULL) {
        TIM_HandleTypeDef *const TIMx = config->interface.htim;
        const uint32_t CHANNELx = get_TIM_CHANNEL_x(config->interface.channel);
        const uint32_t trigger_point = (next_clk_per_tick + config->pulse.rf_compensation) % next_clk_per_tick;

        while (config->pulse.rf_compensation > 0 && __HAL_TIM_GET_COUNTER(main_tim) <= trigger_point); // ensure won't trigger in this tick
        __HAL_TIM_SET_COMPARE(TIMx, CHANNELx, trigger_point);

        if ((edge == FALL_EDGE && !FLAG_CHECK(config->pulse.flags, OT_POLARITY_LOW)) || (edge == RAISE_EDGE && FLAG_CHECK(config->pulse.flags, OT_POLARITY_LOW)))
            TIM_SET_OCMode(TIMx, CHANNELx, TIM_OCMODE_INACTIVE); // (fall && high) || (rise && low) ==> H -> L
        else
            TIM_SET_OCMode(TIMx, CHANNELx, TIM_OCMODE_ACTIVE);   // (rise && high) || (fall && low) ==> L -> H
    }
}

FORCE_INLINE void output_trigger_handler(
        const OutputTriggerConfig *const config, OutputTriggerState *const trigger, const uint64_t *const capture_signals, const uint32_t channel,
        const uint32_t next_clk_per_tick, const TIM_HandleTypeDef *const main_tim) {

    int signal = 0;
    for (uint32_t j = 0; j < TRIGGER_SIGNAL_SIZE; ++j)
        signal = signal || ((capture_signals[j] & config->pulse.signal_masks[j]) != 0);

    //  instantaneous state transfer chain
    //  TRIGGER_RISE ==> (TRIGGER_ACTI) ==> TRIGGER_FALL
    //  TRIGGER_FALL ==> (TRIGGER_DEAD ==> TRIGGER_WAIT) ==> TRIGGER_RISE
    switch (trigger->state) {
        case TRIGGER_RISE:
            trigger->state = TRIGGER_ACTI;
            if (LIKELY(trigger->pulse_count == 0))
                trigger->pulse_count = config->pulse.pulse_width + 1;  // +1 as TRIGGER_ACTI would always -1 first
            // move to TRIGGER_ACTI immediately (in case pulse_width is 0)
        case TRIGGER_ACTI:
            if (UNLIKELY(signal && FLAG_CHECK(config->pulse.flags, OT_ACTIVATE_EXPEND_FLAG))) {
                if (FLAG_CHECK(config->pulse.flags, OT_ACTIVATE_FALL_PRIORITY_FLAG))
                    trigger->pulse_count = config->pulse.pulse_width + 1; // add one for the hidden rise state (ensure the fall edge aligned)
                else
                    trigger->pulse_count += config->pulse.pulse_width;
            }
            else if (UNLIKELY((--trigger->pulse_count == 0) || (signal && FLAG_CHECK(config->pulse.flags, OT_ACTIVATE_FALL_PRIORITY_FLAG)))) {
                setup_trigger_physics_io(next_clk_per_tick, channel, main_tim, config, FALL_EDGE);
                trigger->pulse_count = 0;
                trigger->state = TRIGGER_FALL;
            }

            break;

        case TRIGGER_FALL:
            trigger->state = TRIGGER_DEAD;
            trigger->deadz_count = config->pulse.dead_zone + 1;  // +1 as TRIGGER_DEAD would always -1 first
            // move to TRIGGER_DEAD immediately (in case deadz_count is 0)
        case TRIGGER_DEAD:
            if (--trigger->deadz_count == 0)
                trigger->state = TRIGGER_WAIT;
                // move to TRIGGER_WAIT immediately (in case new signal is in)
            else {
                if (trigger->pulse_count > 0 && FLAG_CHECK(config->pulse.flags, OT_DEADZ_PEND_FALL_PRIORITY_FLAG))
                    --trigger->pulse_count;
                if (signal) {
                    if (trigger->pulse_count == 0) {
                        if (FLAG_CHECK(config->pulse.flags, OT_DEADZ_ALLOW_PEND_FLAG)) {
                            if (FLAG_CHECK(config->pulse.flags, OT_DEADZ_PEND_FALL_PRIORITY_FLAG))
                                trigger->pulse_count = config->pulse.pulse_width;
                            else
                                trigger->pulse_count = config->pulse.pulse_width + 1; // add 1 as TRIGGER_RISE => TRIGGER_ACTI would -1
                        }
                    }
                    else {
                        if (FLAG_CHECK(config->pulse.flags, OT_DEADZ_PEND_FALL_PRIORITY_FLAG) && !FLAG_CHECK(config->pulse.flags, OT_DEADZ_PEND_FIRST_PRIORITY_FLAG))
                            trigger->pulse_count = config->pulse.pulse_width;
                    }
                }
                break;
            }
        case TRIGGER_WAIT:
            if (UNLIKELY(signal || trigger->pulse_count > 0)) {
                setup_trigger_physics_io(next_clk_per_tick, channel, main_tim, config, RAISE_EDGE);
                trigger->state = TRIGGER_RISE;
            }
            else
                break;
    }
}

#endif //DIGITAL_PLL_TRIGGER_HANDLER_H
