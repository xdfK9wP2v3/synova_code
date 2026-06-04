/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "frac.h"
#include "flash.h"

#define CALIB_PERIOD (1ul << 20)
#define IDEAL_RTC_CLK_PER_SECOND (1ul << 15)

extern TIM_HandleTypeDef MAIN_TIM;
extern volatile time_t gps_sys_start_timestamp;
extern volatile uint32_t rtc_wdg;
/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void) {

    /* USER CODE BEGIN RTC_Init 0 */

    /* USER CODE END RTC_Init 0 */

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    /* USER CODE BEGIN RTC_Init 1 */

    /* USER CODE END RTC_Init 1 */

    /** Initialize RTC Only
    */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 8 - 1;
    hrtc.Init.SynchPrediv = 4096 - 1;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE BEGIN Check_RTC_BKUP */

    /* USER CODE END Check_RTC_BKUP */

    /** Initialize RTC and set the Time and Date
    */
    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
        Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 0x1;
    sDate.Year = 0x0;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
        Error_Handler();
    }

    /** Enable the WakeUp
    */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 64 - 1, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN RTC_Init 2 */

    /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef *rtcHandle) {

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (rtcHandle->Instance == RTC) {
        /* USER CODE BEGIN RTC_MspInit 0 */

        /* USER CODE END RTC_MspInit 0 */

        /** Initializes the peripherals clock
        */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
        PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /* RTC clock enable */
        __HAL_RCC_RTC_ENABLE();

        /* RTC interrupt Init */
        HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
        /* USER CODE BEGIN RTC_MspInit 1 */

        /* USER CODE END RTC_MspInit 1 */
    }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef *rtcHandle) {

    if (rtcHandle->Instance == RTC) {
        /* USER CODE BEGIN RTC_MspDeInit 0 */

        /* USER CODE END RTC_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_RTC_DISABLE();

        /* RTC interrupt Deinit */
        HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
        /* USER CODE BEGIN RTC_MspDeInit 1 */

        /* USER CODE END RTC_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
void RTC_Init(void) {
    /** Initialize RTC (RCC clock) **/
    hrtc.Instance = RTC;
    HAL_RTC_MspInit(&hrtc);

    if (HAL_RTCEx_BKUPRead(&hrtc, HOT_START_ADDR) == INITIALIZED_MAGIC_NUMBER)
        ATOMIC_FLAG_SET(sys_state, SYS_STATE_HOT_START_FLAG);
    else {
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_HOT_START_FLAG);
        HAL_RTCEx_BKUPWrite(&hrtc, HOT_START_ADDR, INITIALIZED_MAGIC_NUMBER);
    }

    ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG);
    if (!FLAG_CHECK(sys_state, SYS_STATE_HOT_START_FLAG) || HAL_RTCEx_BKUPRead(&hrtc, UTC_WORK_ADDR) != INITIALIZED_MAGIC_NUMBER) {
        /** Initialize RTC Only **/
        hrtc.Instance = RTC;
        hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
        hrtc.Init.AsynchPrediv = 8 - 1;
        hrtc.Init.SynchPrediv = 4096 - 1;
        hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
        hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
        hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
        hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
        if (HAL_RTC_Init(&hrtc) != HAL_OK)
            Error_Handler();

        /** If flash is available, load calibration data from flash **/
        if (FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE))
            load_rtc_from_flash();

        /** Initialize RTC and set the Time and Date **/
        RTC_TimeTypeDef sTime = {.Hours = 0x0, .Minutes = 0x0, .Seconds = 0x0, .DayLightSaving = RTC_DAYLIGHTSAVING_NONE, .StoreOperation=RTC_STOREOPERATION_RESET};
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
            Error_Handler();
        RTC_DateTypeDef sDate = {.Year = 0x0, .Date = 0x1, .Month = RTC_MONTH_JANUARY, .WeekDay = RTC_WEEKDAY_MONDAY};
        if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
            Error_Handler();

    }
    /** Enable the WakeUp */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 64 - 1, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
        Error_Handler();
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
}

// The timestamp of system start, get by the RTC
static time_t rtc_sys_start_timestamp;

static inline void read_rtc_and_sys_time(RTC_HandleTypeDef *const rtc,
                                         TimePoint *const system_timepoint,
                                         time_t *const rtc_timestamp, uint32_frac *const rtc_subsecond) {
    // read RTC time and system time point at the same time
    uint32_t subsecond_reg, time_reg, date_reg;
    get_time_point_with_rtc_low_priority(system_timepoint, rtc, &date_reg, &time_reg, &subsecond_reg);

    // get the timestamp of RTC
    const uint32_t rtc_prediv_s = REG_MASK_GET(rtc->Instance->PRER, RTC_PRER_PREDIV_S, RTC_PRER_PREDIV_S_Pos);
    rtc_subsecond->numer = rtc_prediv_s - subsecond_reg % (rtc_prediv_s + 1);
    rtc_subsecond->denom = rtc_prediv_s + 1;
    *rtc_timestamp = rtc_reg_get_timestamp(date_reg, time_reg) - subsecond_reg / (rtc_prediv_s + 1);
}

static inline int32_t setup_sys_start_timestamp(RTC_HandleTypeDef *const rtc,
                                                const TimePoint *const system_timepoint,
                                                const time_t rtc_timestamp, const uint32_frac *const rtc_subsecond) {
    // initial the rtc_sys_start_timestamp based on current RTC time
    static int rtc_sys_start_initialized = 0;
    if (UNLIKELY(!rtc_sys_start_initialized)) {
        const uint32_t sys_ss = system_timepoint->tick * rtc_subsecond->denom / timing_scale.ticks_per_second;
        rtc_sys_start_timestamp = rtc_timestamp - system_timepoint->second + FRAC_DIFF_ROUND((int32_t) rtc_subsecond->numer, (int32_t) sys_ss, (int32_t) rtc_subsecond->denom);
        // if is a hot start, and RTC had synced with UTC, the system UTC timestamp is available
        if (FLAG_CHECK(sys_state, SYS_STATE_HOT_START_FLAG) && HAL_RTCEx_BKUPRead(rtc, UTC_WORK_ADDR) == INITIALIZED_MAGIC_NUMBER) {
            utc_sys_start_timestamp = rtc_sys_start_timestamp;
            ATOMIC_FLAG_SET(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG);
        }
        rtc_sys_start_initialized = 1;
    }

    // if GNSS timestamp isn't stable, but the RTC timestamp doesn't match GNSS timestamp, disable UTC timestamp
    if (UNLIKELY(FLAG_CHECK(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG) && FLAG_CHECK(sys_state, SYS_STATE_UTC_INPUT_STABLE_FLAG) && rtc_sys_start_timestamp != gps_sys_start_timestamp)) {
        HAL_RTCEx_BKUPWrite(rtc, UTC_WORK_ADDR, 0);
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG);
    }

    // if the GNSS timestamp is stable, let all timestamps align with GNSS timestamp
    int32_t utc_time_shift = 0;
    if (FLAG_CHECK(sys_state, SYS_STATE_UTC_INPUT_TIMESTAMP_STABLE_FLAG)) {
        if (utc_sys_start_timestamp != gps_sys_start_timestamp)
            utc_time_shift = (int32_t) ((int64_t) gps_sys_start_timestamp - (int64_t) utc_sys_start_timestamp);
        rtc_sys_start_timestamp = gps_sys_start_timestamp, utc_sys_start_timestamp = gps_sys_start_timestamp;
        ATOMIC_FLAG_SET(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG);
    }

    return utc_time_shift;
}

static int coarse_freq_calib_sample_initialized = 0, fine_freq_calib_sample_initialized = 0;
static uint32_t coarse_freq_calib_stable_cnt = 0, fine_freq_calib_stable_cnt = 0, phase_calib_stable_cnt = 0;

static inline void reset_all_calibration_samples() {
    coarse_freq_calib_sample_initialized = 0;
    fine_freq_calib_sample_initialized = 0;
}

static inline int frequency_coarse_calibration(RTC_HandleTypeDef *const rtc,
                                               TimePoint *const system_timepoint,
                                               time_t *const rtc_timestamp, uint32_frac *const rtc_subsecond,
                                               const uint32_t ideal_wake_up_per_second) {
    static uint32_t coarse_freq_calib_sample_cnt = 0;
    static TimePoint prev_system_timepoint;
    static time_t prev_rtc_timestamp;
    static uint32_frac prev_rtc_subsecond;

    // if the prediv_s changed, the sample should be reset
    if (UNLIKELY(rtc_subsecond->denom != prev_rtc_subsecond.denom))
        coarse_freq_calib_sample_initialized = 0;

    // if sampling is started, wait until the sample time finished; or start a sample immediately
    if (LIKELY(coarse_freq_calib_sample_initialized && ++coarse_freq_calib_sample_cnt < ideal_wake_up_per_second * RTC_SYNC_CHECK_PERIOD))
        return 0;

    int rtc_config_updated = 0;
    if (LIKELY(coarse_freq_calib_sample_initialized)) {
        /** calculate the prediv_s based on this round of sample **/

        // get the system tick & RTC sub-second difference since last sync check
        const uint32_t rtc_diff_in_subsecond = (*rtc_timestamp - prev_rtc_timestamp) * rtc_subsecond->denom + rtc_subsecond->numer - prev_rtc_subsecond.numer;
        const uint32_t sys_diff_in_tick = (system_timepoint->second - prev_system_timepoint.second) * timing_scale.ticks_per_second + system_timepoint->tick - prev_system_timepoint.tick;
        // sys_diff_in_tick * pulse_per_second * clks_per_pulse / ticks_per_second
        const uint32_t sys_diff_in_clks = (uint64_t) sys_diff_in_tick * timing_scale.pulse_per_second * timing_scale.clks_per_pulse.integer / timing_scale.ticks_per_second
                                          + (uint64_t) sys_diff_in_tick * timing_scale.pulse_per_second * timing_scale.clks_per_pulse.frac.numer /
                                            timing_scale.clks_per_pulse.frac.denom / timing_scale.ticks_per_second
                                          + system_timepoint->subtick - prev_system_timepoint.subtick;

        // calculate new prediv_s = rtc_diff_in_subsecond / (sys_diff_in_clks / clks_per_pulse)
        const uint32_t new_prediv_s = (uint64_t) timing_scale.clks_per_pulse.integer * rtc_diff_in_subsecond / sys_diff_in_clks
                                      + FRAC_ROUND((uint64_t) timing_scale.clks_per_pulse.integer * rtc_diff_in_subsecond % sys_diff_in_clks, sys_diff_in_clks) - 1;

        // get current prediv_s and max difference can be handled by fine digital calibration
        const uint32_t curr_prediv_s = REG_MASK_GET(rtc->Instance->PRER, RTC_PRER_PREDIV_S, RTC_PRER_PREDIV_S_Pos);
        const uint32_t prediv_a = REG_MASK_GET(rtc->Instance->PRER, RTC_PRER_PREDIV_A, RTC_PRER_PREDIV_A_Pos);
        const uint32_t calib_tolerance = IDEAL_RTC_CLK_PER_SECOND * 512 / CALIB_PERIOD / (prediv_a + 1);  // 512 is the max shift by RTC Precision Digital Calibration

        // check if new prediv_s is close to previous prediv_s (if it outside the range of calibration, do coarse calibration)
        if (curr_prediv_s - calib_tolerance < new_prediv_s && new_prediv_s < curr_prediv_s + calib_tolerance) {
            // coarse frequency calibration looks good, adding evaluation
            if (coarse_freq_calib_stable_cnt > RTC_FREQ_COARSE_NUM_THRESHOLD)
                ATOMIC_FLAG_SET(sys_state, SYS_STATE_RTC_FREQ_COARSE_SYNC_FLAG);
            else
                ++coarse_freq_calib_stable_cnt;
        }
        else {
            /** STOP the RTC and set prediv_s **/

            // coarse frequency calibration has significant error, reset the flag and counting
            ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_RTC_FREQ_COARSE_SYNC_FLAG);
            ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_RTC_FREQ_FINE_SYNC_FLAG);
            reset_all_calibration_samples();

            // reset prediv_s register
            __HAL_RTC_WRITEPROTECTION_DISABLE(rtc);
            if (RTC_EnterInitMode(rtc) == HAL_OK) {
                REG_MASK_SET(rtc->Instance->PRER, new_prediv_s, RTC_PRER_PREDIV_S_Msk, RTC_PRER_PREDIV_S_Pos);
                RTC_ExitInitMode(rtc);
            }
            else
                error_handler(ERROR_FLAG_RTC);
            __HAL_RTC_WRITEPROTECTION_ENABLE(rtc);
            rtc_config_updated = 1;

            // re-read time, as initial mode would stop the clock
            read_rtc_and_sys_time(rtc, system_timepoint, rtc_timestamp, rtc_subsecond);
        }
    }
    else
        coarse_freq_calib_sample_initialized = 1;

    // finished this sample, and initial for the next sample period
    coarse_freq_calib_sample_cnt = 0;
    prev_rtc_timestamp = *rtc_timestamp;
    prev_rtc_subsecond = *rtc_subsecond;
    prev_system_timepoint = *system_timepoint;
    return rtc_config_updated;
}

static inline int rtc_timestamp_check(RTC_HandleTypeDef *const rtc,
                                      TimePoint *const system_timepoint,
                                      time_t *const rtc_timestamp, uint32_frac *const rtc_subsecond) {
    time_t sys_timestamp = rtc_sys_start_timestamp + system_timepoint->second; // the current timestamp of the system clock, based on RTC
    int rtc_config_update = 0;
    if (UNLIKELY(*rtc_timestamp < sys_timestamp - RTC_PHASE_FORCE_SYNC_THRESHOLD || sys_timestamp + RTC_PHASE_FORCE_SYNC_THRESHOLD < *rtc_timestamp)) {
        // phase error is too large (over second level), force sync by edit date and time register
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_RTC_PHASE_SYNC_FLAG);
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_RTC_FREQ_FINE_SYNC_FLAG);
        phase_calib_stable_cnt = 0, fine_freq_calib_stable_cnt = 0;
        reset_all_calibration_samples();
        rtc_config_update = 1;

        __HAL_RTC_WRITEPROTECTION_DISABLE(rtc);
        if (RTC_EnterInitMode(rtc) == HAL_OK) {
            struct tm *time_info = gmtime(&sys_timestamp);
            rtc->Instance->TR = (((uint32_t) RTC_ByteToBcd2(time_info->tm_hour) << RTC_TR_HU_Pos) |
                                 ((uint32_t) RTC_ByteToBcd2(time_info->tm_min) << RTC_TR_MNU_Pos) |
                                 ((uint32_t) RTC_ByteToBcd2(time_info->tm_sec) << RTC_TR_SU_Pos)) & RTC_TR_RESERVED_MASK;
            rtc->Instance->DR = (((uint32_t) RTC_ByteToBcd2((time_info->tm_year + 1900 - 2000) % 100) << RTC_DR_YU_Pos) |
                                 ((uint32_t) RTC_ByteToBcd2(time_info->tm_mon + 1) << RTC_DR_MU_Pos) |
                                 ((uint32_t) RTC_ByteToBcd2(time_info->tm_mday) << RTC_DR_DU_Pos)) & RTC_DR_RESERVED_MASK;
            RTC_ExitInitMode(rtc);
        }
        else
            error_handler(ERROR_FLAG_RTC);
        __HAL_RTC_WRITEPROTECTION_ENABLE(rtc);

        // re-read time, as initial mode would stop the clock
        read_rtc_and_sys_time(rtc, system_timepoint, rtc_timestamp, rtc_subsecond);
    }

    /** if the system utc timestamp is available, after the time sync, the RTC time is also available **/
    if (LIKELY(FLAG_CHECK(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG)))
        HAL_RTCEx_BKUPWrite(rtc, UTC_WORK_ADDR, INITIALIZED_MAGIC_NUMBER);
    return rtc_config_update;
}

static inline int32_t frequency_phase_fine_calibration(RTC_HandleTypeDef *const rtc,
                                                       const TimePoint *const system_timepoint,
                                                       const time_t *const rtc_timestamp, const uint32_frac *const rtc_subsecond,
                                                       const uint32_t ideal_wake_up_per_second) {
    /** Calculate the phase error between system clock and RTC **/
    // get system time
    const uint32_t system_subsecond = system_timepoint->tick * rtc_subsecond->denom / timing_scale.ticks_per_second;
    const time_t sys_timestamp = rtc_sys_start_timestamp + system_timepoint->second; // the current timestamp of the system clock, based on RTC

    // get the phase error
    const time_t year_2020 = 1577808000;
    const int32_t second_error = (int32_t) ((int64_t) (*rtc_timestamp - year_2020) - (int64_t) (sys_timestamp - year_2020)); // subtract year_2020 to avoid overflow
    const int32_t phase_error = second_error * (int32_t) rtc_subsecond->denom + (int32_t) rtc_subsecond->numer - (int32_t) system_subsecond;

    const int32_t max_phase_error = (int32_t) rtc_subsecond->denom / RTC_MAX_PHASE_ERROR_RATE + 1;
    /** Check the phase error: if the error is small enough, ignore it **/
    if (LIKELY(-max_phase_error <= phase_error && phase_error <= max_phase_error)) {
        if (phase_calib_stable_cnt < RTC_PHASE_SECOND_THRESHOLD * ideal_wake_up_per_second)
            ++phase_calib_stable_cnt;
        else
            ATOMIC_FLAG_SET(sys_state, SYS_STATE_RTC_PHASE_SYNC_FLAG);

        if (fine_freq_calib_stable_cnt < RTC_FREQ_FINE_SECOND_THRESHOLD * ideal_wake_up_per_second)
            ++fine_freq_calib_stable_cnt;
        else
            ATOMIC_FLAG_SET(sys_state, SYS_STATE_RTC_FREQ_FINE_SYNC_FLAG);

        return 0;
    }

    /** phase error is large enough to take action **/

    // evaluate phase and frequency (if the time between two phase errors is long enough, ignore it)
    if (phase_calib_stable_cnt < RTC_PHASE_SECOND_THRESHOLD * ideal_wake_up_per_second)
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_RTC_PHASE_SYNC_FLAG);
    if (fine_freq_calib_stable_cnt < RTC_FREQ_FINE_SECOND_THRESHOLD * ideal_wake_up_per_second)
        ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_RTC_FREQ_FINE_SYNC_FLAG);
    phase_calib_stable_cnt = 0, fine_freq_calib_stable_cnt = 0;

    // if SHIFT may cause overflow, ignore once
    if (UNLIKELY(rtc_subsecond->numer & (1ul << 15)))
        return 0;
    // if freq isn't sync coarse, ignore phase error (or possible live-lock: freq_error -> phase_sync -> freq_sample_invalid -> freq_error_keeping)
    if (UNLIKELY(!FLAG_CHECK(sys_state, SYS_STATE_RTC_FREQ_COARSE_SYNC_FLAG)))
        return 0;

    // clip the phase error, avoid RTC shift overflow
    int32_t phase_shift = phase_error;
    if (UNLIKELY(phase_error < -(int32_t) rtc_subsecond->denom))
        phase_shift = -(int32_t) rtc_subsecond->denom;
    if (UNLIKELY(phase_error > (int32_t) ((1ul << 14) - 1)))
        phase_shift = (int32_t) ((1ul << 14) - 1);

    __HAL_RTC_WRITEPROTECTION_DISABLE(rtc);
    // apply the phase shift using RTC shift register
    if (phase_shift > 0)
        rtc->Instance->SHIFTR = RTC_SHIFTADD1S_RESET | REG_MASK_SET_CHECK(phase_shift, RTC_SHIFTR_SUBFS_Msk, RTC_SHIFTR_SUBFS_Pos);
    else
        rtc->Instance->SHIFTR = RTC_SHIFTADD1S_SET | REG_MASK_SET_CHECK((uint32_t) (rtc_subsecond->denom + phase_shift), RTC_SHIFTR_SUBFS_Msk, RTC_SHIFTR_SUBFS_Pos);

    /** frequency fine calibration (RTC Precision Digital Calibration) **/
    if (FLAG_CHECK(sys_state, SYS_STATE_RTC_FREQ_COARSE_SYNC_FLAG)) {
        static uint32_t prev_fine_second, prev_fine_tick;
        static int32_t accumulated_phase_shift = 0;

        // accumulate phase shift applies to the clock
        accumulated_phase_shift += phase_error;

        // if has accumulate enough time (for overcome quantization error)
        if (fine_freq_calib_sample_initialized && system_timepoint->second - prev_fine_second > RTC_FINE_CALIBRATION_PERIOD && phase_shift == phase_error) {
            // get current RTC Precision Digital Calibration
            const uint32_t prev_prediv_s = REG_MASK_GET(rtc->Instance->PRER, RTC_PRER_PREDIV_S, RTC_PRER_PREDIV_S_Pos);
            const uint32_t prev_calm = REG_MASK_GET(rtc->Instance->CALR, RTC_CALR_CALM_Msk, RTC_CALR_CALM_Pos);
            const uint32_t prev_calp = REG_MASK_GET(rtc->Instance->CALR, RTC_CALR_CALP_Msk, RTC_CALR_CALP_Pos);
            const int32_t prev_cal = (int32_t) prev_calm - 512 * (int32_t) prev_calp;

            // get the target CALIB_PERIOD
            const uint32_t tick_diff = (system_timepoint->second - prev_fine_second) * timing_scale.ticks_per_second + system_timepoint->tick - prev_fine_tick;
            const uint32_t old_calib_period = CALIB_PERIOD + prev_cal;
            const uint32_t new_calib_period = old_calib_period + (int64_t) old_calib_period * accumulated_phase_shift * timing_scale.ticks_per_second / tick_diff / prev_prediv_s;

            // check if the calibration request is outside calibration ability
            if (CALIB_PERIOD - 512 <= new_calib_period && new_calib_period <= CALIB_PERIOD + 511) {
                // change calibration register only
                if (new_calib_period < CALIB_PERIOD)
                    rtc->Instance->CALR = RTC_CALR_CALP | REG_MASK_SET_CHECK(new_calib_period + 512 - CALIB_PERIOD, RTC_CALR_CALM_Msk, RTC_CALR_CALM_Pos);
                else
                    rtc->Instance->CALR = REG_MASK_SET_CHECK(new_calib_period - CALIB_PERIOD, RTC_CALR_CALM_Msk, RTC_CALR_CALM_Pos);
            }
            else {
                // disable the digital calibration completely, forcing coarse frequency calibration in the next sync check
                rtc->Instance->CALR = 0;
                ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_RTC_FREQ_COARSE_SYNC_FLAG);
                ATOMIC_FLAG_CLEAR(sys_state, SYS_STATE_RTC_FREQ_FINE_SYNC_FLAG);
                fine_freq_calib_stable_cnt = 0, coarse_freq_calib_stable_cnt = 0;
            }
        }

        // reset the sample after one calibration
        if (!fine_freq_calib_sample_initialized || system_timepoint->second - prev_fine_second > RTC_FINE_CALIBRATION_PERIOD)
            fine_freq_calib_sample_initialized = 1, prev_fine_second = system_timepoint->second, prev_fine_tick = system_timepoint->tick, accumulated_phase_shift = 0;
    }

    __HAL_RTC_WRITEPROTECTION_ENABLE(rtc);

    coarse_freq_calib_sample_initialized = 0;
    return phase_error;
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *rtc) {
    // reset the watch dog
    rtc_wdg = SOFTWARE_WDG_RATE;

    // waiting for the main timer start
    if (UNLIKELY(!FLAG_CHECK(sys_state, SYS_STATE_MAIN_TIM_ENABLE_FLAG)))
        return;

    // if there is pulse input while the main clock didn't follow pulse good, disable fine frequency calibration
    if (FLAG_CHECK(sys_state, SYS_STATE_PULSE_AVAILABLE_FLAG) && (!FLAG_CHECK(sys_state, SYS_STATE_PULSE_FREQ_FAIR_FLAG) || !FLAG_CHECK(sys_state, SYS_STATE_PULSE_PHASE_FAIR_FLAG)))
        reset_all_calibration_samples();

    if (UNLIKELY(MASK_CHECK_ANY(rtc->Instance->ISR, RTC_ISR_RECALPF | RTC_ISR_INIT | RTC_ISR_INITF | RTC_ISR_SHPF)))
        return;  // ensure the RTC not in re-calibrating, initialing mode or state, or doing shifting
    if (UNLIKELY(!MASK_CHECK_ALL(rtc->Instance->ISR, RTC_ISR_RSF)))
        return;  // ensure shadow registers had updated

    /** READ out the current time (system time and RTC time) **/
    TimePoint sys_tp;
    time_t rtc_timestamp;
    uint32_frac rtc_subsecond;
    read_rtc_and_sys_time(rtc, &sys_tp, &rtc_timestamp, &rtc_subsecond);

    // get the frequency of the RTC callback
    const uint32_t clks_per_wake_up = (1 << (4 - (rtc->Instance->CR & 0x3))) * ((rtc->Instance->WUTR & 0xFFFF) + 1);
    const uint32_t ideal_wake_up_per_second = IDEAL_RTC_CLK_PER_SECOND / clks_per_wake_up;

    // for event report
    int rtc_config_update = 0;
    int32_t utc_shift, ss_shift = 0;

    /** Check if the RTC timestamp match GNSS timestamp **/
    utc_shift = setup_sys_start_timestamp(rtc, &sys_tp, rtc_timestamp, &rtc_subsecond);
    rtc_config_update = (utc_shift != 0);

    /** doing frequency coarse calibration by setting prediv_s **/
    frequency_coarse_calibration(rtc, &sys_tp, &rtc_timestamp, &rtc_subsecond, ideal_wake_up_per_second);

    /** check if the RTC timestamp is matched to UTC timestamp, and perform a hard shift if the error is too large **/
    rtc_config_update = rtc_config_update || rtc_timestamp_check(rtc, &sys_tp, &rtc_timestamp, &rtc_subsecond);

    /** making a fine phase sync, and calculate the RTC Precision Digital Calibration **/
    ss_shift = frequency_phase_fine_calibration(rtc, &sys_tp, &rtc_timestamp, &rtc_subsecond, ideal_wake_up_per_second);
    rtc_config_update = rtc_config_update || (ss_shift != 0);

    // record the RTC config update event
    static uint32_t rtc_idle_cnt = 0;
    if (rtc_config_update || rtc_idle_cnt++ > RTC_IDLE_REPORT_INTERVAL * ideal_wake_up_per_second) {
        record_rtc_update_event(sys_state, &sys_tp,
                                utc_shift, REG_MASK_GET(rtc->Instance->PRER, RTC_PRER_PREDIV_S, RTC_PRER_PREDIV_S_Pos), ss_shift,
                                (int32_t) REG_MASK_GET(rtc->Instance->CALR, RTC_CALR_CALM_Msk, RTC_CALR_CALM_Pos) - (FLAG_CHECK(rtc->Instance->CALR, RTC_CALR_CALP) ? (int32_t) 512 : 0)
        );
        rtc_idle_cnt = 0;
    }
}
/* USER CODE END 1 */
