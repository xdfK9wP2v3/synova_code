#ifndef DIGITAL_PLL_EVENT_MSG_H
#define DIGITAL_PLL_EVENT_MSG_H

#include "stdint.h"
#include "help_macro.h"
#include "system_time.h"
#include "time.h"

typedef enum {
    RAISE_EDGE = 0,
    FALL_EDGE = 1,
    RAISE_EDGE_AND_UNKNOWN_EDGES = 2,
    FALL_EDGE_AND_UNKNOWN_EDGES = 3,
} EdgeType;

typedef enum {
    UNKNOWN_HOST_COMMAND = 0,
    CONFIG_CAPIN_COMMAND = 1,
    CONFIG_TIMER_COMMAND = 2,
    CONFIG_TRIGGER_COMMAND = 3,
    CONFIG_IP_MAC_ADDRESS_COMMAND = 4,
    CLEAR_ERROR_COMMAND = 5,
} HostCommandType;

typedef enum {
    CMD_FAIL = 0,
    CMD_OK = 1
} HostCommandState;

typedef enum {
    RETURN_IPMAC_CONFIG = 0,
    RETURN_SYSTEM_TIMING_SCALING = 1,
    RETURN_TIMER_CONFIG = 2,
    RETURN_TRIGGER_CONFIG = 3,
    RETURN_CAPIN_CONFIG = 4,
} ReturnConfigType;

typedef struct {
    enum {
        SECOND_EVENT = 0,           // new second of system main clock
        PULSE_EVENT = 1,            // get a valid pulse input
        UTC_INPUT_EVENT = 2,        // get a UTC time input (may not cause system utc update)
        RTC_UPDATE_EVENT = 3,       // RTC config change
        INPUT_CAPTURE_EVENT = 4,    // capture an input event
        TRIGGER_OUTPUT_EVENT = 5,   // trigger an output event
        COMMAND_RESPONSE_EVENT = 6, // response of host command
        PENDING_ERROR_EVENT = 7,    // error cause last reset
        RETURN_CONFIG_EVENT = 8     // return the current config to the host
    } type;

    uint32_t state;

    // time point
    TimePoint time_point;

    union {
        struct {
            uint32_t second_end;
        } second;
        struct {
            uint32_t prev_clk_per_pulse;
            int32_t phase_error;
            uint32_t curr_clks_per_pulse_integer;
            uint32_t curr_clks_per_pulse_numer;
            uint32_t curr_clks_per_pulse_denom;
        } pulse;
        struct {
            uint32_t year;
            uint8_t month, day, hour, minute, second;
            uint32_t millisecond;
        } utc_input;
        struct {
            uint32_t prediv_s;
            int32_t ss_shift, calibration, utc_shift;
        } rtc_update;
        struct {
            uint32_t channel;
            EdgeType edge;
        } input_capture;
        struct {
            uint32_t channel;
            EdgeType edge;
            int32_t rf_compensation;
        } trigger_output;
        struct {
            HostCommandState command_state;
            HostCommandType command_type;
        } command_response;
        struct {

        } pending_error;
        struct {
            ReturnConfigType return_type;
            uint32_t channel;
        } return_config;
    } data;
} EventMsg;

#define EVENT_MSG_BUF_SIZE 128
#define EVENT_MSG_STR_MAX_LEN 256
extern volatile EventMsg event_msg_buffer[EVENT_MSG_BUF_SIZE];
extern volatile uint32_t next_event_msg_idx;

FORCE_INLINE volatile EventMsg *get_new_event_msg(const uint32_t state, const TimePoint *const time_point) {
    uint32_t curr_event_msg_idx;
    ATOMIC_CYCLE_INCREASE(next_event_msg_idx, curr_event_msg_idx, EVENT_MSG_BUF_SIZE);

    volatile EventMsg *const msg = &event_msg_buffer[curr_event_msg_idx];
    msg->state = state;
    msg->time_point = *time_point;
    return msg;
}

FORCE_INLINE void record_second_event(const uint32_t state, const TimePoint *const time_point, const uint32_t second_end) {
    volatile EventMsg *const msg = get_new_event_msg(state, time_point);
    msg->type = SECOND_EVENT;
    msg->data.second.second_end = second_end;
}

FORCE_INLINE void record_pulse_event(
        const uint32_t state, const TimePoint *const time_point,
        const uint32_t prev_clk_per_pulse, const int32_t phase_error,
        const uint32_t curr_clks_per_pulse_integer, const uint32_t curr_clks_per_pulse_numer, const uint32_t curr_clks_per_pulse_denom) {
    volatile EventMsg *const msg = get_new_event_msg(state, time_point);
    msg->type = PULSE_EVENT;
    msg->data.pulse.prev_clk_per_pulse = prev_clk_per_pulse;
    msg->data.pulse.phase_error = phase_error;
    msg->data.pulse.curr_clks_per_pulse_integer = curr_clks_per_pulse_integer;
    msg->data.pulse.curr_clks_per_pulse_numer = curr_clks_per_pulse_numer;
    msg->data.pulse.curr_clks_per_pulse_denom = curr_clks_per_pulse_denom;
}

FORCE_INLINE void record_utc_input_event(
        const uint32_t state, const TimePoint *const time_point,
        const uint32_t year, const uint8_t month, const uint8_t day,
        const uint8_t hour, const uint8_t minute, const uint8_t sec, const uint32_t millisecond) {
    volatile EventMsg *const msg = get_new_event_msg(state, time_point);
    msg->type = UTC_INPUT_EVENT;
    msg->data.utc_input.year = year;
    msg->data.utc_input.month = month;
    msg->data.utc_input.day = day;
    msg->data.utc_input.hour = hour;
    msg->data.utc_input.minute = minute;
    msg->data.utc_input.second = sec;
    msg->data.utc_input.millisecond = millisecond;
}

FORCE_INLINE void record_rtc_update_event(
        const uint32_t state, const TimePoint *const time_point,
        const int32_t utc_shift, const uint32_t prediv_s, const int32_t ss_shift, const int32_t calibration) {
    volatile EventMsg *const msg = get_new_event_msg(state, time_point);
    msg->type = RTC_UPDATE_EVENT;
    msg->data.rtc_update.utc_shift = utc_shift;
    msg->data.rtc_update.prediv_s = prediv_s;
    msg->data.rtc_update.ss_shift = ss_shift;
    msg->data.rtc_update.calibration = calibration;
}

FORCE_INLINE void record_input_capture_event(
        const uint32_t state, const TimePoint *const time_point,
        const uint32_t channel, const EdgeType edge) {
    volatile EventMsg *const msg = get_new_event_msg(state, time_point);
    msg->type = INPUT_CAPTURE_EVENT;
    msg->data.input_capture.channel = channel;
    msg->data.input_capture.edge = edge;
}

FORCE_INLINE void record_trigger_output_event(
        const uint32_t state, const TimePoint *const time_point,
        const uint32_t channel, const EdgeType edge, const int32_t rf_compensation) {
    volatile EventMsg *const msg = get_new_event_msg(state, time_point);
    msg->type = TRIGGER_OUTPUT_EVENT;
    msg->data.trigger_output.channel = channel;
    msg->data.trigger_output.edge = edge;
    msg->data.trigger_output.rf_compensation = rf_compensation;
}

FORCE_INLINE void record_command_event(
        const uint32_t state, const TimePoint *const time_point,
        const HostCommandState command_state, const HostCommandType command_type
) {
    volatile EventMsg *const msg = get_new_event_msg(state, time_point);
    msg->type = COMMAND_RESPONSE_EVENT;
    msg->data.command_response.command_state = command_state;
    msg->data.command_response.command_type = command_type;
}

FORCE_INLINE void record_pending_error_event(
        const uint32_t state, const TimePoint *const time_point
) {
    volatile EventMsg *const msg = get_new_event_msg(state, time_point);
    msg->type = PENDING_ERROR_EVENT;
}

FORCE_INLINE void record_return_config_event(
        const uint32_t state, const TimePoint *const time_point,
        const ReturnConfigType return_type, const uint32_t channel
) {
    volatile EventMsg *const msg = get_new_event_msg(state, time_point);
    msg->type = RETURN_CONFIG_EVENT;
    msg->data.return_config.return_type = return_type;
    msg->data.return_config.channel = channel;
}

uint32_t get_event_msg_str(char *, const volatile EventMsg *, uint32_t);

#endif //DIGITAL_PLL_EVENT_MSG_H
