#include "config_constants.h"

const uint32_frac CLKS_PER_PULSE_UPDATE_RATE = {.numer=17ul, .denom=1009ul};
const uint32_frac CLKS_PER_PULSE_FAST_UPDATE_RATE = {.numer=541ul, .denom=1009ul};

const uint32_frac PHASE_CORRECT_SECONDS = {.numer = 23, .denom = 3};
const uint32_frac PHASE_CORRECT_FAST_SECONDS = {.numer = 3, .denom = 2};

const uint32_frac MAX_CLKS_SHIFT_PER_TICK_RATE = {.numer=71, .denom=1009};