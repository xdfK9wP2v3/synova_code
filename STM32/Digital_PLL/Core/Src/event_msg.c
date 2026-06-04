#include <stdio.h>
#include <time.h>
#include <memory.h>

#include "event_msg.h"
#include "sys_states.h"
#include "sys_var.h"
#include "mem_helper.h"

volatile __DTCM EventMsg event_msg_buffer[EVENT_MSG_BUF_SIZE];
volatile __DTCM uint32_t next_event_msg_idx = 0;

extern uint32_t stm32_uid_hash;

const static char *edge_mapping[] = {
        "R",    // RAISE_EDGE
        "F",    // FALL_EDGE
        "RX",   // RAISE_EDGE_AND_UNKNOWN_EDGES
        "FX",   // FALL_EDGE_AND_UNKNOWN_EDGES
};

static int second_event_callback(char *const payload_buf, const volatile EventMsg *const msg) {
    return sprintf(payload_buf, "%lu", msg->data.second.second_end);
}

static int pulse_event_callback(char *const payload_buf, const volatile EventMsg *const msg) {
    return sprintf(payload_buf, "%lu,%+ld,%lu+%06lu/%lu",
                   msg->data.pulse.prev_clk_per_pulse,
                   msg->data.pulse.phase_error,
                   msg->data.pulse.curr_clks_per_pulse_integer,
                   msg->data.pulse.curr_clks_per_pulse_numer,
                   msg->data.pulse.curr_clks_per_pulse_denom);
}

static int utc_input_event_callback(char *const payload_buf, const volatile EventMsg *const msg) {
    return sprintf(payload_buf, "%04lu/%02u/%02u,%02u:%02u:%02u.%03lu",
                   msg->data.utc_input.year, msg->data.utc_input.month, msg->data.utc_input.day,
                   msg->data.utc_input.hour, msg->data.utc_input.minute, msg->data.utc_input.second, msg->data.utc_input.millisecond);
}

static int rtc_update_event_callback(char *const payload_buf, const volatile EventMsg *const msg) {
    return sprintf(payload_buf, "%ld,%lu,%+02ld,%04ld",
                   msg->data.rtc_update.utc_shift,
                   msg->data.rtc_update.prediv_s, msg->data.rtc_update.ss_shift, msg->data.rtc_update.calibration);
}

static int input_capture_event_callback(char *const payload_buf, const volatile EventMsg *const msg) {
    return sprintf(payload_buf, "%lu,%s", msg->data.input_capture.channel, edge_mapping[msg->data.input_capture.edge]);
}

static int output_trigger_event_callback(char *const payload_buf, const volatile EventMsg *const msg) {
    return sprintf(payload_buf, "%lu,%s,%ld", msg->data.trigger_output.channel, edge_mapping[msg->data.trigger_output.edge], msg->data.trigger_output.rf_compensation);
}

static int command_response_event_callback(char *const payload_buf, const volatile EventMsg *const msg) {
    const char *command_state_mapping[] = {
            "FAIL", // CMD_FAIL
            "OK"    // CMD_OK
    };
    const char *command_type_mapping[] = {
            "UNKNOWN_CMD",          // UNKNOWN_HOST_COMMAND
            "CAPTURE_CONFIG",       // CONFIG_CAPIN_COMMAND
            "TIMER_CONFIG",         // CONFIG_TIMER_COMMAND
            "TRIGGER_CONFIG",       // CONFIG_TRIGGER_COMMAND
            "IP_MAC_ADDR_CONFIG",   // CONFIG_IP_MAC_ADDRESS_COMMAND
            "CLEAR_ERROR"           // CLEAR_ERROR_COMMAND
    };

    return sprintf(payload_buf, "%s,%s", command_state_mapping[msg->data.command_response.command_state], command_type_mapping[msg->data.command_response.command_type]);
}

static int pending_error_event_callback(char *const payload_buf, const volatile EventMsg *const msg) {
    return sprintf(payload_buf, "%08lX", sys_error);
}

static int return_config_event_callback(char *const payload_buf, const volatile EventMsg *const msg) {
    uint8_t ip_addr[4], ip_mask[4], ip_gate[4], mac_addr[6];
    TimerCaptureConfig *timer;
    OutputTriggerPulseConfig *trigger;
    char mask_buffer[TRIGGER_SIGNAL_SIZE * 20];
    uint32_t pos = 0;

    const uint32_t channel = msg->data.return_config.channel;
    switch (msg->data.return_config.return_type) {
        case RETURN_IPMAC_CONFIG:
            memcpy(ip_addr, &eth_addr_config.ip_addr, 4);
            memcpy(ip_mask, &eth_addr_config.ip_mask, 4);
            memcpy(ip_gate, &eth_addr_config.ip_gate, 4);
            memcpy(mac_addr, &eth_addr_config.mac_prefix, 4);
            mac_addr[4] = (stm32_uid_hash >> 8) & 0xFF;
            mac_addr[5] = (stm32_uid_hash >> 0) & 0xFF;
            return sprintf(payload_buf, "ip_addr,%hhu:%hhu:%hhu:%hhu,ip_mask,%hhu:%hhu:%hhu:%hhu,ip_gate,%hhu:%hhu:%hhu:%hhu,mac_addr,%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                           ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3],
                           ip_mask[0], ip_mask[1], ip_mask[2], ip_mask[3],
                           ip_gate[0], ip_gate[1], ip_gate[2], ip_gate[3],
                           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        case RETURN_SYSTEM_TIMING_SCALING:
            return sprintf(payload_buf, "tick/second,%ld,pulse/second,%ld,clocks/pulse,%ld+%ld/%ld",
                           timing_scale.ticks_per_second, timing_scale.pulse_per_second,
                           timing_scale.clks_per_pulse.integer, timing_scale.clks_per_pulse.frac.numer, timing_scale.clks_per_pulse.frac.denom);
        case RETURN_TIMER_CONFIG:
            timer = &timer_capture_configs[channel - 1];
            return sprintf(payload_buf, "channel,%lu,pre_divider,%lu,shift,%ld,flag,%c",
                           channel, timer->pre_divider, timer->shift,
                           FLAG_CHECK(timer->flags, TC_SECOND_ALIGN) ? (FLAG_CHECK(timer->flags, TC_DROP_LAST) ? 'D' : 'A') : 'N');
        case RETURN_TRIGGER_CONFIG:
            trigger = &output_trigger_configs[channel - 1].pulse;
            for (uint32_t i = 0; i < TRIGGER_SIGNAL_SIZE; ++i)
                pos += sprintf(mask_buffer + pos, "%llx,", trigger->signal_masks[i]);
            return sprintf(payload_buf, "channel,%lu,pulse_width,%u,dead_zone,%u,mask,%s,rf,%d,flags,%c,%c%c",
                           channel, trigger->pulse_width, trigger->dead_zone, mask_buffer, trigger->rf_compensation,
                           FLAG_CHECK(trigger->flags, OT_POLARITY_LOW) ? 'L' : 'H',
                           FLAG_CHECK(trigger->flags, OT_ACTIVATE_EXPEND_FLAG) ?
                           (FLAG_CHECK(trigger->flags, OT_ACTIVATE_FALL_PRIORITY_FLAG) ? 'F' : 'E') :
                           (FLAG_CHECK(trigger->flags, OT_ACTIVATE_FALL_PRIORITY_FLAG) ? 'S' : 'D'),
                           FLAG_CHECK(trigger->flags, OT_DEADZ_PEND_FALL_PRIORITY_FLAG) ?
                           (FLAG_CHECK(trigger->flags, OT_DEADZ_PEND_FIRST_PRIORITY_FLAG) ? 'F' : 'L') :
                           (FLAG_CHECK(trigger->flags, OT_DEADZ_PEND_FIRST_PRIORITY_FLAG) ? 'P' : 'D')
            );
        case RETURN_CAPIN_CONFIG:
            for (uint32_t i = 0; i < INPUT_CAPTURE_NUM; ++i) {
                const InputCaptureConfig *const incap = &input_capture_configs[i];
                payload_buf[i] = incap->capture_edge == CAP_BOTH ? 'B' : (incap->capture_edge == CAP_RISE ? 'R' : 'F');
            }
            payload_buf[INPUT_CAPTURE_NUM] = '\0';
            return INPUT_CAPTURE_NUM;
    }
}

uint32_t get_event_msg_str(char *const buf, const volatile EventMsg *const msg, const uint32_t curr_event_msg_idx) {
    const struct {
        const char *prefix;

        int (*const payload)(char *, const volatile EventMsg *);
    } payload_handlers[] = {
            {.prefix = "SECON", .payload = second_event_callback},              // SECOND_EVENT
            {.prefix = "PULSE", .payload = pulse_event_callback},               // PULSE_EVENT
            {.prefix = "UTCIN", .payload = utc_input_event_callback},           // UTC_INPUT_EVENT
            {.prefix = "RTCUP", .payload = rtc_update_event_callback},          // RTC_UPDATE_EVENT
            {.prefix = "CAPIN", .payload = input_capture_event_callback},       // INPUT_CAPTURE_EVENT
            {.prefix = "TROUT", .payload = output_trigger_event_callback},      // TRIGGER_OUTPUT_EVENT
            {.prefix = "RSPON", .payload = command_response_event_callback},    // COMMAND_RESPONSE_EVENT
            {.prefix = "ERROR", .payload = pending_error_event_callback},       // PENDING_ERROR_EVENT
            {.prefix = "RTCFG", .payload = return_config_event_callback}        // RETURN_CONFIG_EVENT

    };

    /** message prefix: Type, Index, SystemState **/
    uint32_t len = sprintf(buf, "$%s,%04ld,%08lX,", payload_handlers[msg->type].prefix, curr_event_msg_idx, msg->state);

    /** UTC time part: yyyy/mm/dd, hh:mm:ss, UNIX_timestamp **/
    if (FLAG_CHECK(sys_state, SYS_STATE_UTC_TIMESTAMP_AVAILABLE_FLAG)) {
        time_t utc = utc_sys_start_timestamp + msg->time_point.second;
        struct tm *time_info = gmtime(&utc);

        len += sprintf(buf + len, "%04d/%02d/%02d,%02d:%02d:%02d,%lu,",
                       time_info->tm_year + 1900, time_info->tm_mon + 1, time_info->tm_mday,
                       time_info->tm_hour, time_info->tm_min, time_info->tm_sec, (uint32_t) utc);
    }
    else
        len += sprintf(buf + len, ",,,");

    /** System Time part: second, tick, subtick_numer/subtick_demon, clock_cycles_since_second **/
    len += sprintf(buf + len, "%lu,%05lu,%05lu/%lu,%09lu,",
                   msg->time_point.second, msg->time_point.tick, msg->time_point.subtick, msg->time_point.tick_fraction, msg->time_point.clocks_since_prev_second);

    /** Event Message Payload part **/
    len += payload_handlers[msg->type].payload(buf + len, msg);

    /** Suffix part (checksum) **/
    uint8_t checksum = 0;
    for (uint32_t i = 1; i < len; ++i)
        checksum ^= buf[i];
    len += sprintf(buf + len, "*%02X\r\n", checksum);

    return len;
}

