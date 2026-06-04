
#ifndef CACHE_TEST_MEM_HELPER_H
#define CACHE_TEST_MEM_HELPER_H

#include "stdint.h"

// for fast function
#define __ITCM __attribute__((section(".itcm"))) // NOLINT(*-reserved-identifier)
// for fast variable (with initial value)
#define __DTCM __attribute__((section(".dtcm"))) // NOLINT(*-reserved-identifier)

// buffer memory, WITHOUT initial value
#define __BUFFER __attribute__((section(".buffer"))) // NOLINT(*-reserved-identifier)

static inline void load_memory() {
    const uint32_t *p;
    uint32_t *q;

    // load functions from flash to ITCM
    extern uint32_t _siitcm, _sitcm, _eitcm; // NOLINT(*-reserved-identifier)
    p = &_siitcm, q = &_sitcm;
    while (q != &_eitcm)
        *(q++) = *(p++);

    // load data from flash to DTCM
    extern uint32_t _sidtcm, _sdtcm, _edtcm; // NOLINT(*-reserved-identifier)
    p = &_sidtcm, q = &_sdtcm;
    while (q != &_edtcm)
        *(q++) = *(p++);
}

#endif //CACHE_TEST_MEM_HELPER_H
