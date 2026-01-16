#include <stdbool.h>

#include "cmsis.h"
#include "usb.h"
#include "led.h"
#include "systick.h"


static inline void spin(uint32_t ticks) { while(ticks > 0) ticks--; }

/* Defined inside system.c */
extern uint32_t SystemCoreClock;
extern void SystemCoreClockUpdate(void);


/* Overriding interrupt handlers */
void SysTick_Handler(void) { systick_handler(); };


void main(void) {
    // initialize_usb();
    SystemCoreClockUpdate();
    systick_enable(SystemCoreClock/ 1000);
    led_enable();

    while(true) {
        led_toggle();

        systick_wait_ms(1000);

        // spin(99999);
    };
}
