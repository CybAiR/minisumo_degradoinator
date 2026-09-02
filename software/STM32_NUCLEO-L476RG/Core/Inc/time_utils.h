#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include "main.h"

static inline uint32_t timeElapsedMs(uint32_t start_tick)
{
    return HAL_GetTick() - start_tick;
}

#endif
