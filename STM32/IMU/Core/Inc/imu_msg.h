
#ifndef IMU_IMU_MSG_H
#define IMU_IMU_MSG_H

#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "help_macro.h"
#include "string.h"
#include "tcp_server.h"
#include "tim.h"

typedef enum {
    UNKNOWN_HOST_COMMAND = 0,
    CONFIG_RTK_COMMAND = 1,
    CONFIG_FREQ_COMMAND = 2,
    CONFIG_IP_MAC_ADDRESS_COMMAND = 3,
    CLEAR_ERROR_COMMAND = 4,
} HostCommandType;

typedef enum {
    CMD_FAIL = 0,
    CMD_OK = 1
} HostCommandState;

typedef enum {
    RETURN_IPMAC_CONFIG = 0,
    RETURN_RTK_CONFIG = 1
} ReturnConfigType;


typedef struct {
    enum {
        SYNC = 0,
        RTK_NMEA = 1,
        ADIS_RAW = 2,
        MAG_RAW = 3,
        LPS28_RAW = 4,
        LPS22_RAW = 5,
        ERR = 6,
        COMMAND_RESPONSE_EVENT = 7,
        RETURN_CONFIG_EVENT = 8
    } type;

    uint16_t msg_sn;
    OnChipTime timestamp;

    union {
        struct {
            char nmea[256];
        } rtk_nmea;

        struct {
            uint16_t cntr;
            int16_t temp;
            int16_t gyro_x;
            int16_t gyro_y;
            int16_t gyro_z;
            int16_t accl_x;
            int16_t accl_y;
            int16_t accl_z;
        } adis_raw;

        struct {
            uint8_t status;
            int8_t temp;
            int16_t mag_x;
            int16_t mag_y;
            int16_t mag_z;
        } mag_raw;

        struct {
            uint8_t status;
            double_t pressure;
            double_t temperature;
        } lps28_raw;

        struct {
            uint8_t status;

            double_t pressure;
            double_t temperature;
        } lps22_raw;

        struct {
            HostCommandState command_state;
            HostCommandType command_type;
        } command_response;

        struct {
            ReturnConfigType return_type;
        } return_config;

        uint32_t err_code;
    } data;
} IMUMsg;

#define IMU_MSG_BUF_SIZE 256
__attribute__((aligned(32))) extern IMUMsg imu_msg_buffer[IMU_MSG_BUF_SIZE];
extern volatile uint32_t next_imu_msg_idx;
extern volatile uint16_t msg_sn[9];

extern volatile uint8_t msg_type;

FORCE_INLINE volatile IMUMsg *get_new_imu_msg() {
  uint32_t curr_event_msg_idx;
  ATOMIC_CYCLE_INCREASE(next_imu_msg_idx, curr_event_msg_idx, IMU_MSG_BUF_SIZE);

  volatile IMUMsg *msg = &imu_msg_buffer[curr_event_msg_idx];
  return msg;
}


FORCE_INLINE void record_sync_raw(const OnChipTime timestamp) {
  volatile IMUMsg *msg = get_new_imu_msg();
  msg->type = SYNC;
  msg->msg_sn = msg_sn[msg->type];
  msg->timestamp = timestamp;
  msg_sn[msg->type] = (msg_sn[msg->type] + 1) % 10000ul;
}


FORCE_INLINE void record_error_raw(const OnChipTime timestamp) {
  volatile IMUMsg *msg = get_new_imu_msg();
  msg->type = ERR;
  msg->msg_sn = msg_sn[msg->type];
  msg->timestamp = timestamp;
  msg->data.err_code = sys_error;
  msg_sn[msg->type] = (msg_sn[msg->type] + 1) % 10000ul;
}

FORCE_INLINE void record_command_event(const OnChipTime timestamp, const HostCommandState command_state, const HostCommandType command_type) {
  volatile IMUMsg * msg = get_new_imu_msg();
  msg->type = COMMAND_RESPONSE_EVENT;
  msg->timestamp = timestamp;
  msg->data.command_response.command_state = command_state;
  msg->data.command_response.command_type = command_type;
}

FORCE_INLINE void record_return_config_event(const OnChipTime timestamp, const ReturnConfigType return_type) {
  volatile IMUMsg *const msg = get_new_imu_msg();
  msg->type = RETURN_CONFIG_EVENT;
  msg->timestamp = timestamp;
  msg->data.return_config.return_type = return_type;
}

FORCE_INLINE void record_rtk_nmea(const char *nmea) {
  volatile IMUMsg *msg = get_new_imu_msg();
  msg->type = RTK_NMEA;
  memcpy((void *) msg->data.rtk_nmea.nmea, nmea, strlen(nmea) + 1);
}


FORCE_INLINE void record_adis_raw(const OnChipTime timestamp,
                                  const uint16_t cntr,
                                  const int16_t temp,
                                  const int16_t gyro_x,
                                  const int16_t gyro_y,
                                  const int16_t gyro_z,
                                  const int16_t accl_x,
                                  const int16_t accl_y,
                                  const int16_t accl_z) {
  volatile IMUMsg *msg = get_new_imu_msg();
  msg->type = ADIS_RAW;
  msg->msg_sn = msg_sn[msg->type];
  msg->timestamp = timestamp;
  msg->data.adis_raw.cntr = cntr;
  msg->data.adis_raw.temp = temp;
  msg->data.adis_raw.gyro_x = gyro_x;
  msg->data.adis_raw.gyro_y = gyro_y;
  msg->data.adis_raw.gyro_z = gyro_z;
  msg->data.adis_raw.accl_x = accl_x;
  msg->data.adis_raw.accl_y = accl_y;
  msg->data.adis_raw.accl_z = accl_z;
  msg_sn[msg->type] = (msg_sn[msg->type] + 1) % 10000ul;
}


FORCE_INLINE void record_mag_raw(const OnChipTime timestamp,
                                 const uint8_t status,
                                 const int8_t temp,
                                 const int16_t mag_x,
                                 const int16_t mag_y,
                                 const int16_t mag_z) {
  volatile IMUMsg *msg = get_new_imu_msg();
  msg->type = MAG_RAW;
  msg->msg_sn = msg_sn[msg->type];
  msg->timestamp = timestamp;
  msg->data.mag_raw.status = status;
  msg->data.mag_raw.temp = temp;
  msg->data.mag_raw.mag_x = mag_x;
  msg->data.mag_raw.mag_y = mag_y;
  msg->data.mag_raw.mag_z = mag_z;
  msg_sn[msg->type] = (msg_sn[msg->type] + 1) % 10000ul;
}


FORCE_INLINE void record_lps28_raw(const OnChipTime timestamp,
                                   const uint8_t status,
                                   const double_t pressure,
                                   const double_t temperature) {
  volatile IMUMsg *msg = get_new_imu_msg();
  msg->type = LPS28_RAW;
  msg->msg_sn = msg_sn[msg->type];
  msg->timestamp = timestamp;
  msg->data.lps28_raw.status = status;
  msg->data.lps28_raw.pressure = pressure;
  msg->data.lps28_raw.temperature = temperature;
  msg_sn[msg->type] = (msg_sn[msg->type] + 1) % 10000ul;
}


FORCE_INLINE void record_lps22_raw(const OnChipTime timestamp,
                                   const uint8_t status,
                                   const double_t pressure,
                                   const double_t temperature) {
  volatile IMUMsg *msg = get_new_imu_msg();
  msg->type = LPS22_RAW;
  msg->msg_sn = msg_sn[msg->type];
  msg->timestamp = timestamp;
  msg->data.lps22_raw.status = status;
  msg->data.lps22_raw.pressure = pressure;
  msg->data.lps22_raw.temperature = temperature;
  msg_sn[msg->type] = (msg_sn[msg->type] + 1) % 10000ul;
}


void dbg_msg_transmit(void *msg, uint32_t size);
HAL_StatusTypeDef tcp_transmit(char *, uint8_t, uint32_t);
uint32_t get_imu_msg(char *buf,  IMUMsg *, uint32_t *);

#endif //IMU_IMU_MSG_H
