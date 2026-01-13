#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

void systick_enable(uint32_t ticks);
void systick_wait_ms(uint32_t ms);
void systick_handler(void);


#endif

