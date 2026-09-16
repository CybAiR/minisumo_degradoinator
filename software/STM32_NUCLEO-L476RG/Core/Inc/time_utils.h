#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include "main.h"

static inline uint32_t timeElapsedMs(uint32_t startTick)
{
    return HAL_GetTick() - startTick;
}

#endif
