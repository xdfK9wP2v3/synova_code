

#include "imu_msg.h"
#include "stdio.h"
#include "tcp_server.h"
#include "lwip/tcp.h"
#include "usart.h"
#include "rtk.h"
//#include "usbd_cdc_if.h"

__attribute__((aligned(32))) IMUMsg imu_msg_buffer[IMU_MSG_BUF_SIZE];
uint32_t volatile next_imu_msg_idx = 0;
uint16_t volatile msg_sn[9] = {0};
extern uint32_t stm32_uid_hash;
uint8_t ip_addr[4], ip_mask[4], ip_gate[4], mac_addr[6];

const int16_t msg_pcb_mapping[] = {-1, 0, 1, 2, 3, 4};

void dbg_msg_transmit(void *msg, uint32_t size) {
    HAL_UART_Transmit(UART_DBG_HDL, (uint8_t *) msg, size, UART_TIMEOUT);
//    while (CDC_Transmit_FS((uint8_t *) msg, size) == USBD_BUSY);
}


HAL_StatusTypeDef tcp_transmit(char *msg_buff, uint8_t tcp_msg_type, uint32_t msg_len) {
  uint16_t pcb_idx_start, pcb_idx_end;
  uint8_t snd_buf_enough = 1;
  if (tcp_msg_type == SYNC || tcp_msg_type == ERR || tcp_msg_type == COMMAND_RESPONSE_EVENT || tcp_msg_type == RETURN_CONFIG_EVENT) {
    pcb_idx_start = msg_pcb_mapping[ADIS_RAW];
    pcb_idx_end = msg_pcb_mapping[LPS22_RAW];
  } else {
    pcb_idx_start = msg_pcb_mapping[tcp_msg_type];
    pcb_idx_end = pcb_idx_start;
  }

  for (uint16_t i = pcb_idx_start; i < pcb_idx_end + 1; i++) {
    if (eth_pcb[i] && eth_pcb[i]->local_port == (i + TCP_PORT_START) && eth_pcb[i]->state == ESTABLISHED)
      snd_buf_enough &= (eth_pcb[i]->snd_buf >= msg_len);
  }
  if (!snd_buf_enough) return HAL_BUSY;

  for (uint16_t i = pcb_idx_start; i < pcb_idx_end + 1; i++) {
    if (eth_pcb[i] && eth_pcb[i]->local_port == (i + TCP_PORT_START) && eth_pcb[i]->state == ESTABLISHED) {
      err_t write_status = tcp_write(eth_pcb[i],
                                     msg_buff,
                                     msg_len,
                                     TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE);
      if (write_status != ERR_OK && write_status != ERR_CONN) {
        dbg_msg_len = sprintf(dbg_msg_buff, "[Port %u] TCP write errno: %2d, link status: %u\r\n",
                              eth_pcb[i]->local_port,
                              write_status,
                              eth_pcb[i]->state);
        dbg_msg_transmit(dbg_msg_buff, dbg_msg_len);
        return HAL_ERROR;
      }

//      if (i == msg_pcb_mapping[ADIS_RAW] || tcp_msg_type == SYNC || tcp_msg_type == ERR)
        tcp_output(eth_pcb[i]);
    }
  }
  return HAL_OK;
}


uint32_t get_imu_msg(char *const buf, IMUMsg *msg, uint32_t *const type) {
    // set prefix
    const char *event_name[] = {
            "TMS",
            "RTK",
            "ADIS",
            "MAG",
            "LPS28",
            "LPS22",
            "ERROR",
            "RSPON",
            "RSCFG",
    };

    const char *command_state_mapping[] = {
            "FAIL", // CMD_FAIL
            "OK"    // CMD_OK
    };
    const char *command_type_mapping[] = {
            "UNKNOWN_CMD",          // UNKNOWN_HOST_COMMAND
            "RTK_CONFIG",         // CONFIG_TIMER_COMMAND
            "FREQ_CONFIG",       // CONFIG_TRIGGER_COMMAND
            "IP_MAC_ADDR_CONFIG",   // CONFIG_IP_MAC_ADDRESS_COMMAND
            "CLEAR_ERROR"           // CLEAR_ERROR_COMMAND
    };

    *type = msg->type;
    uint32_t len;

    if (msg->type == RTK_NMEA) {
        len = sprintf(buf, "%s", msg->data.rtk_nmea.nmea);
    } else {
        len = sprintf(buf, "$%s,", event_name[msg->type]);

        // set data message
        char *const data_buf = buf + len;
        switch (msg->type) {
            case SYNC:
                len += sprintf(data_buf, "%04u,%06lu.%06lu", msg->msg_sn, msg->timestamp.second, msg->timestamp.microsecond);
              break;
            case ADIS_RAW:
                len += sprintf(data_buf, "%04u,%06lu.%06lu,%05u,%d/10,C,%d,gX,%d,gY,%d,gZ,%d,aX,%d,aY,%d,aZ",
                               msg->msg_sn,
                               msg->timestamp.second, msg->timestamp.microsecond,
                               msg->data.adis_raw.cntr,
                               msg->data.adis_raw.temp,
                               msg->data.adis_raw.gyro_x,
                               msg->data.adis_raw.gyro_y,
                               msg->data.adis_raw.gyro_z,
                               msg->data.adis_raw.accl_x,
                               msg->data.adis_raw.accl_y,
                               msg->data.adis_raw.accl_z);
                break;
            case MAG_RAW:
                len += sprintf(data_buf, "%04u,%06lu.%06lu,%02X,%d,C,%d,X,%d,Y,%d,Z",
                               msg->msg_sn,
                               msg->timestamp.second, msg->timestamp.microsecond,
                               msg->data.mag_raw.status,
                               msg->data.mag_raw.temp,
                               msg->data.mag_raw.mag_x,
                               msg->data.mag_raw.mag_y,
                               msg->data.mag_raw.mag_z);
                break;
            case LPS28_RAW:
                len += sprintf(data_buf, "%04u,%06lu.%06lu,%02X,%.2f,hPa,%.2f,C",
                               msg->msg_sn,
                               msg->timestamp.second, msg->timestamp.microsecond,
                               msg->data.lps28_raw.status,
                               msg->data.lps28_raw.pressure,
                               msg->data.lps28_raw.temperature);
                break;
            case LPS22_RAW:
                len += sprintf(data_buf, "%04u,%06lu.%06lu,%02X,%.2f,hPa,%.2f,C",
                               msg->msg_sn,
                               msg->timestamp.second, msg->timestamp.microsecond,
                               msg->data.lps22_raw.status,
                               msg->data.lps22_raw.pressure,
                               msg->data.lps22_raw.temperature);
                break;
            case ERR:
                len += sprintf(data_buf, "%04u,%06lu.%06lu,%08lX", msg->msg_sn, msg->timestamp.second, msg->timestamp.microsecond, msg->data.err_code);
                break;
            case COMMAND_RESPONSE_EVENT:
                len += sprintf(data_buf, "%06lu.%06lu,%s,%s",
                               msg->timestamp.second, msg->timestamp.microsecond,
                               command_state_mapping[msg->data.command_response.command_state],
                               command_type_mapping[msg->data.command_response.command_type]);
                break;
            case RETURN_CONFIG_EVENT:
                switch (msg->data.return_config.return_type) {
                    case RETURN_IPMAC_CONFIG:
                        memcpy(ip_addr, &eth_addr_config.ip_addr, 4);
                        memcpy(ip_mask, &eth_addr_config.ip_mask, 4);
                        memcpy(ip_gate, &eth_addr_config.ip_gate, 4);
                        memcpy(mac_addr, &eth_addr_config.mac_prefix, 4);
                        mac_addr[4] = (stm32_uid_hash >> 8) & 0xFF;
                        mac_addr[5] = (stm32_uid_hash >> 0) & 0xFF;
                        len += sprintf(data_buf,
                                       "%06lu.%06lu,ip_addr,%hhu:%hhu:%hhu:%hhu,ip_mask,%hhu:%hhu:%hhu:%hhu,ip_gate,%hhu:%hhu:%hhu:%hhu,mac_addr,%02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX",
                                       msg->timestamp.second, msg->timestamp.microsecond,
                                       ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3],
                                       ip_mask[0], ip_mask[1], ip_mask[2], ip_mask[3],
                                       ip_gate[0], ip_gate[1], ip_gate[2], ip_gate[3],
                                       mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
                        break;
                    case RETURN_RTK_CONFIG:
                        len += sprintf(data_buf, "%06lu.%06lu,%s", msg->timestamp.second, msg->timestamp.microsecond, rtk_config.rtk_type == RTK_BS ? "BS" : "MOB");
                        break;
                }
                break;
            default:
              error_handler(ERROR_FLAG_UNKNOWN_MSG);
        }

        // set suffix
        uint8_t checksum = 0;
        for (uint32_t i = 1; i < len; ++i)
            checksum ^= buf[i];
        len += sprintf(buf + len, "*%02X\r\n", checksum);
    }

    return len;
}