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
void USB_HP_CAN1_TX_IRQHandler(void) { usb_hp_handler(); };
void USB_LP_CAN1_RX0_IRQHandler(void) { usb_lp_handler(); };
void USBWakeUp_IRQHandler(void) { usb_wakeup_handler(); };


void main(void) {
    /* SysTick initialization, other stuff depend on this */
    SystemCoreClockUpdate();
    systick_enable(SystemCoreClock / 1000);

    /* Peripherals initilization */
    led_enable();
    usb_initialize();

    while(true) {
        //led_toggle();

        //systick_wait_ms(500);

        // spin(99999);
    };
}
