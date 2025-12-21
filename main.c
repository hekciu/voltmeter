#include <stdbool.h>

#define STM32WB55xx

#include "stm32wbxx.h"


#define BIT(i) (1UL << (i))


static inline void spin(uint32_t ticks) { while(ticks > 0) ticks--; }



__attribute__((naked, noreturn)) void _reset(void) {

    while(true) {
    };
}



extern void _estack(void);  // Defined in link.ld

// 16 standard and 63 STM32WB55-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 63])(void) = {
  _estack, _reset, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};
