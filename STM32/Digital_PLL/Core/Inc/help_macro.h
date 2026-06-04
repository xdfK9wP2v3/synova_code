/*
 * macro that helping programming
 */

#ifndef INC_HELP_MACRO_H_
#define INC_HELP_MACRO_H_

#include "stm32h7xx.h"

// disable reorder
#define MEMORY_BARRIER()  __asm__ __volatile__("": : :"memory")
// hint for branch
#define LIKELY(x)         __builtin_expect(!!(x), 1)
#define UNLIKELY(x)        __builtin_expect(!!(x), 0)

#define FORCE_INLINE __attribute__((always_inline)) static inline

// flag set & check
#define FLAG_CHECK(var, flag) (!!((var) & (flag)))
#define FLAG_CLEAR(var, flag) CLEAR_BIT(var, flag)
#define FLAG_SET(var, flag) SET_BIT(var, flag)
#define FLAG_EDIT(var, flag, val) ((var) = (val) ? ((var) | (flag)) : ((var) & (~(flag))))
#define ATOMIC_FLAG_CLEAR(var, flag) ATOMIC_CLEAR_BIT(var, flag)
#define ATOMIC_FLAG_SET(var, flag) ATOMIC_SET_BIT(var, flag)
#define ATOMIC_FLAG_EDIT(var, flag, val)                                                                \
    do{                                                                                                 \
        uint32_t tmp;                                                                                   \
        do {                                                                                            \
          tmp = __LDREXW((__IO uint32_t *)&(var));                                                      \
        } while (__STREXW(tmp ? ((var) | (flag)) : ((var) & (flag)), (__IO uint32_t *)&(var)) != 0U);   \
    } while(0)
#define ATOMIC_FLAG_SET_CLEAR(var, set_flags, clear_flags)                                          \
    do{                                                                                             \
        uint32_t tmp;                                                                               \
        do {                                                                                        \
          tmp = __LDREXW((__IO uint32_t *)&(var));                                                  \
        } while (__STREXW((tmp & ~(clear_flags)) | (set_flags), (__IO uint32_t *)&(var)) != 0U);    \
    } while(0)

#define MASK_CHECK_ANY(var, mask) (!!((var) & (mask)))
#define MASK_CHECK_ALL(var, mask) (((var) & (mask)) == (mask))
#define MASK_CHECK_NONE(var, mask) MASK_CHECK_ALL(~(var), mask)

#define REG_MASK_SET_CHECK(val, mask, pos) (((val) & ((mask) >> (pos))) << (pos))
#define REG_MASK_SET(var, val, mask, pos) ((var) = ((var) & (~(mask))) | REG_MASK_SET_CHECK(val, mask, pos))
#define REG_MASK_GET(var, mask, pos) (((var) & (mask)) >> (pos))

#define ATOMIC_CYCLE_INCREASE(REG, VAR, PERIOD)                          \
    do {                                                                 \
      (VAR) = __LDREXW((__IO uint32_t *)&(REG));                         \
    } while (__STREXW(((VAR) + 1) % (PERIOD), (__IO uint32_t *)&(REG)) != 0U)

#endif /* INC_HELP_MACRO_H_ */
