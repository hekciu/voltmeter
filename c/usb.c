#include "cmsis.h"
#include "usb.h"


void initialize_usb() {
    // activate register macrocell clock

    // de-assert macrocell specific reset signal

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
