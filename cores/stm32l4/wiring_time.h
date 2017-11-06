#ifndef __wiring_time_H_
#define __wiring_time_H_


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "armv7m.h"

static inline uint32_t millis(void) 
{
    return armv7m_systick_millis();
}

static inline uint32_t micros(void) 
{
    return armv7m_systick_micros();
}

static inline void delay(uint32_t msec) 
{
    if (msec == 0)
	return;

    armv7m_systick_delay(msec);
}

static inline void delayMicroseconds(uint32_t usec) 
{
    if (usec == 0)
	return;

    armv7m_core_udelay(usec);
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif
