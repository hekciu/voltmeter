#include <stdbool.h>

#include "cmsis.h"
#include "usb.h"
#include "led.h"
#include "systick.h"


static inline void spin(uint32_t ticks) { while(ticks > 0) ticks--; }


__attribute__((naked, noreturn)) void _reset(void) {
    // initialize_usb();
    // TODO: calcualte proper value for these ticks
    // TODO: Can I set interrupt handlers via some CMSIS macros/functions???
    systick_enable(4000);
    led_enable();

    while(true) {
        led_toggle();

        systick_wait_ms(1000);

        // spin(99999);
    };
}


extern void _estack(void);  // Defined in link.ld

// 16 standard and 68 STM32F10xxx-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 68])(void) = {
  _estack, _reset, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, systick_handler
};
