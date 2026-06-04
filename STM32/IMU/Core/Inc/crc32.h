

#ifndef IMU_CRC32_H
#define IMU_CRC32_H

#include <stdint.h>

uint32_t crc32(const uint8_t *data, const uint32_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (uint32_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

#endif //IMU_CRC32_H
