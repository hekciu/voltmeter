#include <stdint.h>

#include "cmsis.h"
#include "usb.h"
#include "led.h"
#include "systick.h"


#define ENDPOINT_0_ADDRESS 0

static void on_usb_reset();


void usb_hp_handler(void) {
    led_toggle();
}


void usb_lp_handler(void) {
    led_toggle();
}


void usb_wakeup_handler(void) {
    led_toggle();
}


void usb_initialize() {
    /*
    NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0);
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0);
    */

    NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_EnableIRQ(USBWakeUp_IRQn);

    /*
        usb macrocell clock configuration,
        we need 48MHz from PLL output
    */
    // set HSI (8MHz) / 2 = 4MHz as PLL input
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;

    // select PLL multiply factor to 12 (12 * 4 = 48)
    RCC->CFGR &= ~RCC_CFGR_PLLMULL_Msk;
    RCC->CFGR |= (10 << RCC_CFGR_PLLMULL_Pos);

    // set usb prescaler to 1x (we need 48MHz)
    RCC->CFGR |= RCC_CFGR_USBPRE;

    // enable PLL
    RCC->CR |= RCC_CR_PLLON;

    // de-assert macrocell specific reset signal
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST;
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    // switch on interval voltage for port transceiver
    USB->CNTR &= ~USB_CNTR_PDWN;

    // wait for t_startup (1 us)
    systick_wait_ms(1); /* We actually wait 1000x longer but I guess it shouldn't be a problem */

    // remove reset condition on USB
    USB->CNTR &= ~USB_CNTR_FRES;

    // remove any spurious pending interrupt
    USB->ISTR = 0UL;

    // enable all interrupts and configure registers
    USB->CNTR |= USB_CNTR_CTRM;
    USB->CNTR |= USB_CNTR_PMAOVRM;
    USB->CNTR |= USB_CNTR_ERRM;
    USB->CNTR |= USB_CNTR_WKUPM;
    USB->CNTR |= USB_CNTR_SUSPM;
    USB->CNTR |= USB_CNTR_RESETM;
    USB->CNTR |= USB_CNTR_SOFM;
    USB->CNTR |= USB_CNTR_ESOFM;

    // initialize packet buffer description table

    // this part is the same as when usb reset interrupt is triggered
    on_usb_reset();
}


static void configure_0_endpoint() {
    // 0 endpoint must always have CONTROL type
    USB->EP0R &= ~USB_EP0R_EP_TYPE_Msk;

    // set 0 endpoint address
    USB->EP0R &= ~USB_EP0R_EA_Msk;
    USB->EP0R |= (ENDPOINT_0_ADDRESS << USB_EP0R_EA_Pos);

    // enable 0 endpoint
}


static void on_usb_reset() {
    // enable USB device
    USB->DADDR |= USB_DADDR_EF;

    configure_0_endpoint();
}
