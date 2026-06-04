
#include "i2c_queue.h"

I2CTask i2c_task_buffer[I2C_TASK_BUF_SIZE];
uint32_t volatile next_i2c_task_idx = 0;
