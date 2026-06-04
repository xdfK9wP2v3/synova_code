

#ifndef DIGITAL_PLL_MEM_HELPER_H
#define DIGITAL_PLL_MEM_HELPER_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

// for fast function
#define __ITCM __attribute__((section(".itcm"))) // NOLINT(*-reserved-identifier)

// for fast variable (with initial value)
#define __DTCM __attribute__((section(".dtcm"))) // NOLINT(*-reserved-identifier)

#define __CACHE_ALIGN __attribute__((aligned(32))) // NOLINT(*-reserved-identifier)

// buffer memory, WITHOUT initial value
#define __DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32))) // NOLINT(*-reserved-identifier)

// buffer memory, WITHOUT initial value
#define __ETH_BUFFER __attribute__((section(".eth_buffer"), aligned(32))) // NOLINT(*-reserved-identifier)

static inline void init_memory() {
    const uint32_t *p;
    uint32_t *q;

    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();

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

    // clear DMA buffer to zero
    extern uint32_t _s_dma_buf, _e_dma_buf; // NOLINT(*-reserved-identifier)
    q = &_s_dma_buf;
    while(q != &_e_dma_buf)
        *(q++) = 0;

    // clear ETH buffer to zero
    extern uint32_t _s_eth_buf, _e_eth_buf; // NOLINT(*-reserved-identifier)
    q = &_s_eth_buf;
    while(q != &_e_eth_buf)
        *(q++) = 0;
}

#endif //DIGITAL_PLL_MEM_HELPER_H
