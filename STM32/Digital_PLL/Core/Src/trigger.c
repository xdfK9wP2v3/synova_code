#include "trigger.h"
#include "error.h"

void init_input_capture_gpio(const InputCaptureConfig *const capture) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = capture->interface.gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = capture->interface.alternate;
    HAL_GPIO_Init(capture->interface.gpio_group, &GPIO_InitStruct);
}

// TIM_TS_ITR0
void init_slave_tim(TIM_HandleTypeDef *const htim, const uint32_t ITRx, const int trig_out) {
    // set up the main clock
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 65535;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim->Lock = HAL_UNLOCKED;
    TIM_Base_SetConfig(htim->Instance, &htim->Init);
    htim->DMABurstState = HAL_DMA_BURST_STATE_READY;
    TIM_CHANNEL_STATE_SET_ALL(htim, HAL_TIM_CHANNEL_STATE_READY);
    TIM_CHANNEL_N_STATE_SET_ALL(htim, HAL_TIM_CHANNEL_STATE_READY);
    htim->State = HAL_TIM_STATE_READY;

    // set up the slave mode as internal input reset mode
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = ITRx;
    if (HAL_TIM_SlaveConfigSynchro(htim, &sSlaveConfig) != HAL_OK)
        error_handler(ERROR_FLAG_HAL_ERROR);

    // set up master output trigger
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = trig_out ? TIM_TRGO_UPDATE : TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = trig_out ? TIM_MASTERSLAVEMODE_ENABLE : TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
        error_handler(ERROR_FLAG_HAL_ERROR);

    // disable break if this timer has one
    if (IS_TIM_BREAK_INSTANCE(htim->Instance)) {
        TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
        sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
        sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
        sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
        sBreakDeadTimeConfig.DeadTime = 0;
        sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
        sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        sBreakDeadTimeConfig.BreakFilter = 0;
        sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
        sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
        sBreakDeadTimeConfig.Break2Filter = 0;
        sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
        if (HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig) != HAL_OK)
            error_handler(ERROR_FLAG_HAL_ERROR);
    }
}

void init_tim_input_capture(const InputCaptureConfig *const capture) {
    TIM_IC_InitTypeDef sConfigIC = {0};
    if(capture->capture_edge == CAP_RISE)
        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    else if(capture->capture_edge == CAP_FALL)
        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    else if(capture->capture_edge == CAP_BOTH)
        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    else
        assert_param(0);
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;           // direct capture
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;                     // no pre-scaler
    sConfigIC.ICFilter = 0;                                     // no filter
    if (HAL_TIM_IC_ConfigChannel(capture->interface.htim, &sConfigIC, get_TIM_CHANNEL_x(capture->interface.channel)) != HAL_OK)
        error_handler(ERROR_FLAG_HAL_ERROR);
}

void init_tim_output_trigger(const OutputTriggerConfig *const trigger) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_TIMING;               // MODE = Frozen
    sConfigOC.Pulse = 0;                                // compare value = 0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;         // set active as high (polarity control by trigger handler)
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;          // not pwm, disable fast
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;      // no break, disable
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;    // no break, disable
    if (HAL_TIM_OC_ConfigChannel(trigger->interface.htim, &sConfigOC, get_TIM_CHANNEL_x(trigger->interface.channel)) != HAL_OK)
        error_handler(ERROR_FLAG_HAL_ERROR);
}

void init_output_trigger_gpio(const OutputTriggerConfig *const trigger) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = trigger->interface.gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = trigger->interface.alternate;
    HAL_GPIO_Init(trigger->interface.gpio_group, &GPIO_InitStruct);
}

HAL_StatusTypeDef TIM_IC_Setup(TIM_HandleTypeDef *htim, uint32_t Channel) {
    HAL_TIM_ChannelStateTypeDef channel_state = TIM_CHANNEL_STATE_GET(htim, Channel);
    HAL_TIM_ChannelStateTypeDef complementary_channel_state = TIM_CHANNEL_N_STATE_GET(htim, Channel);

    /* Check the parameters */
    assert_param(IS_TIM_CCX_CHANNEL(htim->Instance, Channel));

    /* Check the TIM channel state */
    if ((channel_state != HAL_TIM_CHANNEL_STATE_READY) || (complementary_channel_state != HAL_TIM_CHANNEL_STATE_READY))
        return HAL_ERROR;

    /* Set the TIM channel state */
    TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);
    TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

    /* Enable the Input Capture channel */
    TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);

    /* Return function status */
    return HAL_OK;
}

HAL_StatusTypeDef TIM_OC_Setup(TIM_HandleTypeDef *htim, uint32_t Channel) {
    /* Check the parameters */
    assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

    /* Check the TIM channel state */
    if (TIM_CHANNEL_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
        return HAL_ERROR;

    /* Set the TIM channel state */
    TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

    /* Enable the Output compare channel */
    TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);
    if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
        __HAL_TIM_MOE_ENABLE(htim);

    /* Return function status */
    return HAL_OK;
}

HAL_StatusTypeDef TIM_IC_Setup_IT(TIM_HandleTypeDef *htim, uint32_t Channel) {
    HAL_TIM_ChannelStateTypeDef channel_state = TIM_CHANNEL_STATE_GET(htim, Channel);
    HAL_TIM_ChannelStateTypeDef complementary_channel_state = TIM_CHANNEL_N_STATE_GET(htim, Channel);

    /* Check the parameters */
    assert_param(IS_TIM_CCX_CHANNEL(htim->Instance, Channel));

    /* Check the TIM channel state */
    if ((channel_state != HAL_TIM_CHANNEL_STATE_READY) || (complementary_channel_state != HAL_TIM_CHANNEL_STATE_READY))
        return HAL_ERROR;

    /* Set the TIM channel state */
    TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);
    TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

    switch (Channel) {
        case TIM_CHANNEL_1:
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
            break;
        case TIM_CHANNEL_2:
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC2);
            break;
        case TIM_CHANNEL_3:
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC3);
            break;
        case TIM_CHANNEL_4:
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC4);
            break;
        default:
            return HAL_ERROR;
    }

    /* Enable the Input Capture channel */
    TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);

    /* Return function status */
    return HAL_OK;
}

HAL_StatusTypeDef TIM_PWM_Setup_IT(TIM_HandleTypeDef *htim, uint32_t Channel) {

    /* Check the parameters */
    assert_param(IS_TIM_CCX_CHANNEL(htim->Instance, Channel));

    /* Check the TIM channel state */
    if (TIM_CHANNEL_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
        return HAL_ERROR;

    /* Set the TIM channel state */
    TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

    switch (Channel) {
        case TIM_CHANNEL_1:
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
            break;
        case TIM_CHANNEL_2:
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC2);
            break;
        case TIM_CHANNEL_3:
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC3);
            break;
        case TIM_CHANNEL_4:
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC4);
            break;
        default:
            return HAL_ERROR;
    }

    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);
    if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
        __HAL_TIM_MOE_ENABLE(htim);

    /* Return function status */
    return HAL_OK;
}
