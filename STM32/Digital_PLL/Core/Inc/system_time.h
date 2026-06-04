#ifndef DIGITAL_PLL_SYSTEM_TIME_H
#define DIGITAL_PLL_SYSTEM_TIME_H

#include "stdint.h"
#include "time.h"
#include "help_macro.h"

typedef struct {
    uint32_t second;                    // index of second from power on
    uint32_t tick;                      // index of tick (timer overflow) from a second start (0 at initial of a second)
    uint32_t clocks_since_prev_second;  // clock from second start to the tick
} SystemMainTime;

typedef struct {
    uint32_t second, tick, subtick, tick_fraction, clocks_since_prev_second;
} TimePoint;

void get_time_point_low_priority(TimePoint *);

void get_time_point_with_rtc_low_priority(TimePoint *time_point, RTC_HandleTypeDef *hrtc, uint32_t *, uint32_t *, uint32_t *);

FORCE_INLINE void set_time_point(TimePoint *const time_point, const uint32_t second, const uint32_t tick,
                                 const uint32_t subtick, const uint32_t tick_fraction, const uint32_t clocks_since_prev_second) {
    time_point->second = second;
    time_point->tick = tick;
    time_point->subtick = subtick;
    time_point->tick_fraction = tick_fraction;
    time_point->clocks_since_prev_second = clocks_since_prev_second;
}

FORCE_INLINE void get_time_point_high_priority(const volatile SystemMainTime *const main_time, TimePoint *const time_point, const uint32_t subtick, const uint32_t tick_fraction) {
    time_point->second = main_time->second;
    time_point->tick = main_time->tick;
    time_point->subtick = subtick;
    time_point->tick_fraction = tick_fraction;
    time_point->clocks_since_prev_second = main_time->clocks_since_prev_second + subtick;
}

time_t get_timestamp(uint32_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);

int sub_second_sync(uint32_t ms_a, uint32_t ms_b, uint32_t advan, uint32_t delay, uint32_t period, int *second_shift);

void rtc_reg_get_time(uint32_t time_reg, uint8_t *hour, uint8_t *minute, uint8_t *second);

void rtc_reg_get_date(uint32_t date_reg, uint32_t *year, uint8_t *month, uint8_t *day);

time_t rtc_reg_get_timestamp(uint32_t date_reg, uint32_t time_reg);

void rtc_read_reg(RTC_HandleTypeDef *hrtc, uint32_t *date_reg, uint32_t *time_reg, uint32_t *subsecond, uint32_t *subsecond_frac);

FORCE_INLINE void wait_RTC_second_sync(RTC_HandleTypeDef *hrtc) {
    uint32_t date_reg, time_reg, subsecond, subsecond_frac;
    do {
        rtc_read_reg(hrtc, &date_reg, &time_reg, &subsecond, &subsecond_frac);
    } while (subsecond != 0);
}

#endif //DIGITAL_PLL_SYSTEM_TIME_H
