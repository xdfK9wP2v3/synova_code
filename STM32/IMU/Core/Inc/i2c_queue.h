
#ifndef IMU_I2C_QUEUE_H
#define IMU_I2C_QUEUE_H

#include "stdint.h"
#include "help_macro.h"
#include "imu_msg.h"

#define I2C_TASK_BUF_SIZE 256

typedef struct {
  uint32_t task;
  OnChipTime timestamp;
} I2CTask;

extern I2CTask i2c_task_buffer[I2C_TASK_BUF_SIZE];
extern volatile uint32_t next_i2c_task_idx;

FORCE_INLINE volatile I2CTask *get_new_i2c_task() {
  uint32_t curr_i2c_task_idx = next_i2c_task_idx;
  next_i2c_task_idx = (next_i2c_task_idx + 1) % I2C_TASK_BUF_SIZE;

  volatile I2CTask *task = &i2c_task_buffer[curr_i2c_task_idx];
  return task;
}

FORCE_INLINE void record_mag_task(const OnChipTime timestamp) {
  volatile I2CTask *task = get_new_i2c_task();
  task->task = MAG_OCC;
  task->timestamp = timestamp;
}

FORCE_INLINE void record_lps28_task(const OnChipTime timestamp) {
  volatile I2CTask *task = get_new_i2c_task();
  task->task = LPS28_OCC;
  task->timestamp = timestamp;
}

FORCE_INLINE void record_lps22_task(const OnChipTime timestamp) {
  volatile I2CTask *task = get_new_i2c_task();
  task->task = LPS22_OCC;
  task->timestamp = timestamp;
}

#endif //IMU_I2C_QUEUE_H
