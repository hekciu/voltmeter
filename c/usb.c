#include "cmsis.h"
#include "usb.h"


void initialize_usb() {
    // activate register macrocell clock
    SET_BIT(RCC->CFGR, BIT(22));

    // de-assert macrocell specific reset signal
    SET_BIT(RCC->ABP1RSTR, !BIT(23));
    SET_BIT(RCC->ABP1ENR, BIT(23));

    // switch on interval voltage for port transceiver
    SET_BIT(USB->CNTR, !BIT(1)); // !PDWN

    // wait for t_startup

    // remove reset condition on USB
    SET_BIT(USB->CNTR, !BIT(0)); // !FRES

    // remove any spurious pending interrupt
    CLEAR_REG(USB->ISTR);

    // this part is the same as when usb reset interrupt is triggered
    on_usb_reset();
}


static void on_usb_reset() {
    // enable USB device
    SET_BIT(USB->DADDR, BIT(7)); // EF

}


static void configure_0_endpoint() {
    // 0 endpoint must always have CONTROL type
    USB->EP0R &= ~(3UL << 9);

    // set USB KIND?
}
