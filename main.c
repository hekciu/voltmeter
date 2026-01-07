#include <stdbool.h>

#include "cmsis.h"
#include "usb.h"

static inline void spin(uint32_t ticks) { while(ticks > 0) ticks--; }



__attribute__((naked, noreturn)) void _reset(void) {
    initialize_usb();

    while(true) {
    };
}



extern void _estack(void);  // Defined in link.ld

// 16 standard and 68 STM32F10xxx-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 68])(void) = {
  _estack, _reset, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};
