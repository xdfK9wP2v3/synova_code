#include "system_time.h"
#include "sys_var.h"
#include "peripheral_mapping.h"
#include "tim.h"

void get_time_point_low_priority(TimePoint *const time_point) {
    while (1) {
        time_point->second = sys_tim.second;
        MEMORY_BARRIER();
        time_point->tick = sys_tim.tick;
        MEMORY_BARRIER();

        time_point->subtick = __HAL_TIM_GET_COUNTER(&MAIN_TIM);
        time_point->tick_fraction = __HAL_TIM_GET_AUTORELOAD(&MAIN_TIM) + 1;
        time_point->clocks_since_prev_second = sys_tim.clocks_since_prev_second + time_point->subtick;

        MEMORY_BARRIER();
        const uint32_t post_tick = sys_tim.tick;
        MEMORY_BARRIER();
        const uint32_t post_second = sys_tim.second;
        if (LIKELY(time_point->second == post_second && time_point->tick == post_tick)) return;
    }
}

void get_time_point_with_rtc_low_priority(TimePoint *const time_point, RTC_HandleTypeDef *const hrtc, uint32_t *const date_reg, uint32_t *const time_reg, uint32_t *const subsecond) {
    while (1) {
        time_point->second = sys_tim.second;
        MEMORY_BARRIER();
        time_point->tick = sys_tim.tick;
        MEMORY_BARRIER();

        time_point->subtick = __HAL_TIM_GET_COUNTER(&MAIN_TIM);
        rtc_read_reg(hrtc, date_reg, time_reg, subsecond, NULL);
        time_point->tick_fraction = __HAL_TIM_GET_AUTORELOAD(&MAIN_TIM) + 1;
        time_point->clocks_since_prev_second = sys_tim.clocks_since_prev_second + time_point->subtick;

        MEMORY_BARRIER();
        const uint32_t post_tick = sys_tim.tick;
        MEMORY_BARRIER();
        const uint32_t post_second = sys_tim.second;
        if (LIKELY(time_point->second == post_second && time_point->tick == post_tick)) return;
    }
}

time_t get_timestamp(const uint32_t year, const uint8_t month, const uint8_t day, const uint8_t hour, const uint8_t minute,
                     const uint8_t second) {
    struct tm time_info;
    time_info.tm_year = (int) (year - 1900);
    time_info.tm_mon = month - 1;
    time_info.tm_mday = day;
    time_info.tm_hour = hour;
    time_info.tm_min = minute;
    time_info.tm_sec = second;
    time_info.tm_isdst = 0;
    return mktime(&time_info);
}

int sub_second_sync(const uint32_t ms_a, const uint32_t ms_b, const uint32_t advan, const uint32_t delay,
                    const uint32_t period, int *const second_shift) {
    if (period + ms_a - advan <= period + ms_b && ms_b <= ms_a + delay) { // a & b in same second
        *second_shift = 0;
        return 1;
    }
    if (period - delay <= ms_a && ms_b <= (ms_a + delay) % period) { // an is delay than b (b in next second)
        *second_shift = 1;
        return 1;
    }
    if (period - advan <= ms_b && ms_a <= (ms_b + advan) % period) { // an is advancer than b (a in next second)
        *second_shift = -1;
        return 1;
    }
    return 0;
}

void rtc_reg_get_time(const uint32_t time_reg, uint8_t *const hour, uint8_t *const minute, uint8_t *const second) {
    if (hour != NULL)
        *hour = RTC_Bcd2ToByte((uint8_t) REG_MASK_GET(time_reg, RTC_TR_HT | RTC_TR_HU, RTC_TR_HU_Pos));
    if (minute != NULL)
        *minute = RTC_Bcd2ToByte((uint8_t) REG_MASK_GET(time_reg, RTC_TR_MNT | RTC_TR_MNU, RTC_TR_MNU_Pos));
    if (second != NULL)
        *second = RTC_Bcd2ToByte((uint8_t) REG_MASK_GET(time_reg, RTC_TR_ST | RTC_TR_SU, RTC_TR_SU_Pos));
}

void rtc_reg_get_date(const uint32_t date_reg, uint32_t *const year, uint8_t *const month, uint8_t *const day) {
    if (year != NULL)
        *year = 2000 + RTC_Bcd2ToByte((uint8_t) REG_MASK_GET(date_reg, RTC_DR_YT | RTC_DR_YU, RTC_DR_YU_Pos));
    if (month != NULL)
        *month = RTC_Bcd2ToByte((uint8_t) REG_MASK_GET(date_reg, RTC_DR_MT | RTC_DR_MU, RTC_DR_MU_Pos));
    if (day != NULL)
        *day = RTC_Bcd2ToByte((uint8_t) REG_MASK_GET(date_reg, RTC_DR_DT | RTC_DR_DU, RTC_DR_DU_Pos));
}

time_t rtc_reg_get_timestamp(const uint32_t date_reg, const uint32_t time_reg) {
    uint32_t year;
    uint8_t month, day, hour, minute, second;
    rtc_reg_get_date(date_reg, &year, &month, &day);
    rtc_reg_get_time(time_reg, &hour, &minute, &second);
    return get_timestamp(year, month, day, hour, minute, second);
}

void rtc_read_reg(RTC_HandleTypeDef *const hrtc, uint32_t *const date_reg, uint32_t *const time_reg, uint32_t *const subsecond, uint32_t *const subsecond_frac) {
    FLAG_CLEAR(hrtc->Instance->ISR, RTC_ISR_RSF);
    while (!FLAG_CHECK(hrtc->Instance->ISR, RTC_ISR_RSF));
    MEMORY_BARRIER();
    *subsecond = hrtc->Instance->SSR;
    MEMORY_BARRIER();
    if (subsecond_frac != NULL)
        *subsecond_frac = REG_MASK_GET(hrtc->Instance->PRER, RTC_PRER_PREDIV_S, RTC_PRER_PREDIV_S_Pos) + 1;
    *time_reg = hrtc->Instance->TR & RTC_TR_RESERVED_MASK;
    MEMORY_BARRIER();
    *date_reg = hrtc->Instance->DR & RTC_DR_RESERVED_MASK;
    MEMORY_BARRIER();
}