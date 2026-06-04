#ifndef DIGITAL_PLL_FRAC_H
#define DIGITAL_PLL_FRAC_H

#include <stdint.h>
#include "help_macro.h"

#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef ABS
#define ABS(x)  ((x) >= 0 ? (x) : (-(x)))
#endif /* ABS */

#define FRAC_ROUND(numer, demon) (((numer) <= ((demon) / 2)) ? 0 : 1)  // round(numer / demon)
#define FRAC_DIFF_ROUND(numer_x, numer_y, demon) ((((numer_x) + (demon) / 2)) < (numer_y) ? -1 : ((((numer_y) + (demon) / 2) < (numer_x)) ? 1 : 0))  // round(numer_x / demon - numer_y / demon)
#define DIV_ROUND(dividend, divisor) ((dividend) / (divisor) + FRAC_ROUND((dividend) % (divisor), (divisor)))

typedef struct {
    uint32_t numer, denom;
} uint32_frac;

typedef struct {
    int32_t numer;
    uint32_t denom;
} int32_frac;

typedef struct {
    uint32_t integer;
    uint32_frac frac;
} uint32_mixed_frac;

FORCE_INLINE uint32_t mixed_frac_round(const uint32_mixed_frac *const value) {
    return value->integer + FRAC_ROUND(value->frac.numer, value->frac.denom);
}

FORCE_INLINE uint32_mixed_frac frac_abs(const volatile int32_frac *const value) {
    const uint32_t numer = ABS(value->numer);
    return (uint32_mixed_frac) {.integer = numer / value->denom, .frac={.numer=numer % value->denom, .denom=value->denom}};
}

FORCE_INLINE uint32_t round_frac_mul(const uint32_frac *const frac, const uint32_t value) {
    return (uint32_t) ((uint64_t) value * (uint64_t) frac->numer / (uint64_t) frac->denom);
}

FORCE_INLINE uint32_mixed_frac mixed_frac_mul(const volatile uint32_mixed_frac *const x, const uint32_t y) {
    const uint32_t new_numer = x->frac.numer * y;
    return (uint32_mixed_frac) {.integer = x->integer * y + new_numer / x->frac.denom, .frac = {.numer = new_numer % x->frac.denom, .denom=x->frac.denom}};
}

FORCE_INLINE uint32_mixed_frac mul_frac(const uint32_frac *const x, const uint32_t y) {
    const uint64_t new_numer = (uint64_t) x->numer * (uint64_t) y;
    return (uint32_mixed_frac) {.integer = new_numer / x->denom, .frac = {.numer = new_numer % x->denom, .denom=x->denom}};
}

FORCE_INLINE uint32_mixed_frac div_mixed_frac(const volatile uint32_mixed_frac *const x, const uint32_t y) {
    // x / y == ((x->integer / y) * y + x->integer % y + x->frac.numer / x->frac.denom) / y
    //       == (x->integer / y) + x->integer % y / y + x->frac.numer / (x->frac.denom * y)
    return (uint32_mixed_frac) {.integer = x->integer / y, .frac = {.numer = x->integer % y * x->frac.denom + x->frac.numer, .denom=x->frac.denom * y}};
}

FORCE_INLINE void ExponentialMovingAverage(volatile uint32_mixed_frac *const ema_value, const uint32_t new_value, const uint32_frac *const update_rate) {
    /** ema_value = (1 - update_rate) * ema_value + update_rate * new_value **/

    // ema == ema_full_numer / ema_value->frac.denom
    const uint64_t ema_full_numer = (uint64_t) ema_value->integer * (uint64_t) ema_value->frac.denom + (uint64_t) ema_value->frac.numer;
    // updated == updated_numer / (ema_value->frac.denom * update_rate->denom)
    const uint64_t updated_numer = (uint64_t) (update_rate->denom - update_rate->numer) * (uint64_t) ema_full_numer
                                   + (uint64_t) update_rate->numer * ((uint64_t) new_value * (uint64_t) ema_value->frac.denom);
    // updated == updated_numer / ema_value->frac.denom
    const uint64_t new_full_numer = DIV_ROUND(updated_numer, (uint64_t) update_rate->denom);

    ema_value->integer = (uint32_t) (new_full_numer / ema_value->frac.denom);
    ema_value->frac.numer = (uint32_t) (new_full_numer % ema_value->frac.denom);
}

#endif //DIGITAL_PLL_FRAC_H
