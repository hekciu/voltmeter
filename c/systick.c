#include <stdint.h>

#include "cmsis.h"
#include "systick.h"


static volatile uint32_t ticks_ms = 0;

void systick_handler(void) {
    ticks_ms++;
}

void systick_enable(uint32_t ticks) {
    SysTick_Config(ticks);
}


void systick_wait_ms(uint32_t ms) {
    ticks_ms = 0;

    while (ticks_ms < ms) (void)0;
}
