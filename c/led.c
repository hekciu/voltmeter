#include "led.h"
#include "cmsis.h"

#include <stdbool.h>


/* GPIO -> PC13 */


volatile static bool on = false;

void led_enable() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    /* Output mode, max speed 10 MHz */
    GPIOC->CRH &= ~GPIO_CRH_MODE13_Msk;
    GPIOC->CRH |= GPIO_CRH_MODE13_0;

    /* General purpose output push-pull */
    GPIOC->CRH &= ~GPIO_CRH_CNF13_Msk;
}

void led_toggle() {
    if (on) {
        on = false;

        GPIOC->BSRR &= ~GPIO_BSRR_BS13;
        GPIOC->BSRR |= GPIO_BSRR_BR13;
    } else {
        on = true;

        GPIOC->BSRR &= ~GPIO_BSRR_BR13;
        GPIOC->BSRR |= GPIO_BSRR_BS13;
    }
}
