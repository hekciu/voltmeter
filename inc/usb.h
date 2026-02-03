#ifndef USB_H
#define USB_H

#include <stdint.h>

void usb_initialize(void);
void usb_hp_handler(void);
void usb_lp_handler(void);
void usb_wakeup_handler(void);
void usb_send_string(const char*);

#endif
