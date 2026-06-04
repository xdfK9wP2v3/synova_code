#ifndef DIGITAL_PLL_TRIGGER_H
#define DIGITAL_PLL_TRIGGER_H

#include "stdint.h"
#include "help_macro.h"
#include "system_time.h"
#include "config_constants.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint8_t channel;
    GPIO_TypeDef *gpio_group;
    uint16_t gpio_pin;
    uint8_t alternate;
} PhysicalInterface;

typedef enum {
    CAP_RISE = 0x01,
    CAP_FALL = 0x02,
    CAP_BOTH = 0x01 | 0x02,
} CaptureEdge;

typedef struct {
    PhysicalInterface interface;
    CaptureEdge capture_edge;
} InputCaptureConfig;

typedef struct {
    uint32_t flags;
    uint32_t pending_capture;
    TimePoint time_point;
} InputCaptureState;

typedef struct {
    uint32_t pre_divider;
    int32_t shift;
    uint32_t flags;
} TimerCaptureConfig;

typedef struct {

} LogicRelay;

typedef struct {
    uint16_t pulse_width;       // (pulse_width + 1) active tick
    uint16_t dead_zone;         // (dead_zone + 1) tick between any two pulse
    int16_t rf_compensation;
    uint32_t flags;
    uint64_t signal_masks[TRIGGER_SIGNAL_SIZE];
} OutputTriggerPulseConfig;

typedef struct {
    PhysicalInterface interface;
    OutputTriggerPulseConfig pulse;
} OutputTriggerConfig;

typedef struct {
    enum {
        TRIGGER_WAIT = 0,  // waiting for signal
        TRIGGER_RISE = 1,  // rising edge (at the end of this tick)
        TRIGGER_ACTI = 2,  // during pulse (active state)
        TRIGGER_FALL = 3,  // falling edge (at the end of this tick)
        TRIGGER_DEAD = 4   // dead-zone
    } state;
    uint32_t pulse_count, deadz_count;
} OutputTriggerState;

FORCE_INLINE void set_physical_interface(PhysicalInterface *const interface,
                                         TIM_HandleTypeDef *const htim, const uint8_t channel,
                                         GPIO_TypeDef *const gpio_group, const uint16_t gpio_pin, const uint8_t alternate) {
    interface->htim = htim;
    interface->channel = channel;
    interface->gpio_group = gpio_group;
    interface->gpio_pin = gpio_pin;
    interface->alternate = alternate;
}

FORCE_INLINE void init_input_capture_state(InputCaptureState *const input_capture) {
    input_capture->flags = 0;
}

FORCE_INLINE void init_trigger_state(OutputTriggerState *const trigger) {
    trigger->state = TRIGGER_WAIT;
    trigger->pulse_count = 0;
    trigger->deadz_count = 0;
}

FORCE_INLINE void init_input_capture_config_zero(InputCaptureConfig *const config) {
    config->capture_edge = CAP_BOTH;
}

FORCE_INLINE void init_timer_capture_config_zero(TimerCaptureConfig *const config) {
    config->flags = 0;
    config->pre_divider = 0;
    config->shift = 0;
}

FORCE_INLINE void init_output_trigger_config_zero(OutputTriggerConfig *const config) {
    config->pulse.flags = 0;
    config->pulse.pulse_width = 0;
    config->pulse.dead_zone = 0;
    config->pulse.rf_compensation = 0;
    for (int i = 0; i < TRIGGER_SIGNAL_SIZE; ++i)
        config->pulse.signal_masks[i] = 0;
}

FORCE_INLINE void TIM_SET_OCMode(TIM_HandleTypeDef *const TIMx, const uint32_t channel, const uint32_t OCMode) {
    switch (channel) {
        case TIM_CHANNEL_1:
            MODIFY_REG(TIMx->Instance->CCMR1, TIM_CCMR1_OC1M, OCMode);
            break;
        case TIM_CHANNEL_2:
            MODIFY_REG(TIMx->Instance->CCMR1, TIM_CCMR1_OC2M, OCMode << 8U);
            break;
        case TIM_CHANNEL_3:
            MODIFY_REG(TIMx->Instance->CCMR2, TIM_CCMR2_OC3M, OCMode);
            break;
        case TIM_CHANNEL_4:
            MODIFY_REG(TIMx->Instance->CCMR2, TIM_CCMR2_OC4M, OCMode << 8U);
            break;
        default:
            break;
    }
}

FORCE_INLINE void TIM_SET_Polarity(TIM_HandleTypeDef *const TIMx, const uint32_t channel, const uint32_t Polarity) {
    switch (channel) {
        case TIM_CHANNEL_1:
            MODIFY_REG(TIMx->Instance->CCER, TIM_CCER_CC1P, Polarity);
            break;
        case TIM_CHANNEL_2:
            MODIFY_REG(TIMx->Instance->CCER, TIM_CCER_CC2P, Polarity << 4U);
            break;
        case TIM_CHANNEL_3:
            MODIFY_REG(TIMx->Instance->CCER, TIM_CCER_CC3P, Polarity << 8U);
            break;
        case TIM_CHANNEL_4:
            MODIFY_REG(TIMx->Instance->CCER, TIM_CCER_CC4P, Polarity << 12U);
            break;
        default:
            break;
    }
}

FORCE_INLINE uint32_t get_TIM_CHANNEL_x(const uint32_t channel) {
    const static uint32_t CHANNEL_mapping[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    return CHANNEL_mapping[channel - 1];
}

FORCE_INLINE uint32_t get_TIM_CCxIF(const uint32_t channel) {
    const static uint32_t CCxIF_mapping[] = {TIM_SR_CC1IF, TIM_SR_CC2IF, TIM_SR_CC3IF, TIM_SR_CC4IF};
    return CCxIF_mapping[channel - 1];
}

FORCE_INLINE uint32_t get_TIM_CCxOF(const uint32_t channel) {
    const static uint32_t CCxOF_mapping[] = {TIM_FLAG_CC1OF, TIM_FLAG_CC2OF, TIM_FLAG_CC3OF, TIM_FLAG_CC4OF};
    return CCxOF_mapping[channel - 1];
}

FORCE_INLINE uint32_t get_TIM_CCxG(const uint32_t channel) {
    const static uint32_t CCxG_mapping[] = {TIM_EGR_CC1G, TIM_EGR_CC2G, TIM_EGR_CC3G, TIM_EGR_CC4G};
    return CCxG_mapping[channel - 1];
}

void init_input_capture_gpio(const InputCaptureConfig *);

void init_slave_tim(TIM_HandleTypeDef *, uint32_t, int);

void init_tim_input_capture(const InputCaptureConfig *);

void init_tim_output_trigger(const OutputTriggerConfig *);

void init_output_trigger_gpio(const OutputTriggerConfig *);

HAL_StatusTypeDef TIM_IC_Setup(TIM_HandleTypeDef *, uint32_t);

HAL_StatusTypeDef TIM_OC_Setup(TIM_HandleTypeDef *, uint32_t);

HAL_StatusTypeDef TIM_IC_Setup_IT(TIM_HandleTypeDef *, uint32_t);

HAL_StatusTypeDef TIM_PWM_Setup_IT(TIM_HandleTypeDef *, uint32_t);

#endif //DIGITAL_PLL_TRIGGER_H
