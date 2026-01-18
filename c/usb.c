#include <stdint.h>
#include <assert.h>

#include "cmsis.h"
#include "usb.h"
#include "led.h"
#include "systick.h"


typedef struct {
    uint32_t tx_addrs;
    uint32_t tx_count;
    uint32_t rx_addrs;
    uint32_t rx_count; 
} pma_endpoint_desc_t;


#define ENDPOINT_DESC(n) ((pma_endpoint_desc_t*)(USB_PMAADDR + n*sizeof(pma_endpoint_desc_t)))


#define ENDPOINT_0_ADDRESS 0
#define BTABLE_ADDRESS 0x0

#define PMA_ENDPT_DESC_BYTES sizeof(pma_endpoint_desc_t)
#define ENDPT_NUM 8

#define ENDPOINT0_RX_BUFFER_SIZE 1024

#define PACKET_RX_BUFFER_ADDR_0 (USB_PMAADDR + PMA_ENDPT_DESC_BYTES * ENDPT_NUM)
#define PACKET_TX_BUFFER_ADDR_0 (PACKET_RX_BUFFER_ADDR_0 + ENDPOINT0_RX_BUFFER_SIZE)

#define PMA_ADDR_FROM_APP(n) (n - USB_PMAADDR)

static void zero_btable();
static void on_usb_reset();
static void configure_0_endpoint();
static void configure_1_endpoint();
static void set_endpoint_0_rx_status(uint32_t status);
static void set_endpoint_0_tx_status(uint32_t status);


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
    // enable usb clock
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    uint32_t usb_priority_group = NVIC_GetPriorityGrouping();

    NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, NVIC_EncodePriority(usb_priority_group, 0, 0));
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(usb_priority_group, 0, 0));
    NVIC_SetPriority(USBWakeUp_IRQn, NVIC_EncodePriority(usb_priority_group, 0, 0));

    NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_EnableIRQ(USBWakeUp_IRQn);


    /*
        usb macrocell clock configuration,
        we need 48MHz from PLL output
    */

    // de-assert macrocell specific reset signal
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST;
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    // reset and clear reset
    USB->CNTR |= USB_CNTR_FRES;
    USB->CNTR = 0UL;

    // BDT address is 0x0 relative to how USB peripheral see dedicated memory
    USB->BTABLE = BTABLE_ADDRESS;

    // switch on interval voltage for port transceiver
    USB->CNTR &= ~USB_CNTR_PDWN;
    // wait for t_startup (1 us)
    systick_wait_ms(1); /* We actually wait 1000x longer but I guess it shouldn't be a problem */

    // remove any spurious pending interrupt
    USB->ISTR = 0UL;

    // enable reset interrupt
    USB->CNTR |= USB_CNTR_RESETM;

    // initialize packet buffer description table

    // this part is the same as when usb reset interrupt is triggered
    on_usb_reset();
}


static void on_usb_reset() {
    zero_btable();

    // configure endpoints
    configure_0_endpoint();
    configure_1_endpoint();

    // enable USB device
    USB->DADDR |= USB_DADDR_EF;

    // enable other interrupts (besides  RESETM)
    USB->CNTR |= USB_CNTR_CTRM;
    USB->CNTR |= USB_CNTR_PMAOVRM;
    USB->CNTR |= USB_CNTR_ERRM;
    USB->CNTR |= USB_CNTR_WKUPM;
    //USB->CNTR |= USB_CNTR_SUSPM;
    //USB->CNTR |= USB_CNTR_SOFM;
    //USB->CNTR |= USB_CNTR_ESOFM;
}


static void configure_0_endpoint() {
    // 0 endpoint must always have CONTROL type
    USB->EP0R &= ~USB_EP0R_EP_TYPE_Msk;
    // we always write CTR_TX and CTR_RX so to not clear bit accidentally
    USB->EP0R |= USB_EP0R_EP_TYPE_0 | USB_EP0R_CTR_TX | USB_EP0R_CTR_RX;

    // set 0 endpoint address
    USB->EP0R &= ~USB_EP0R_EA_Msk;
    USB->EP0R |= (ENDPOINT_0_ADDRESS << USB_EP0R_EA_Pos) | USB_EP0R_CTR_TX | USB_EP0R_CTR_RX;

    static_assert(ENDPOINT0_RX_BUFFER_SIZE > 62);

    // set rx endpoint
    ENDPOINT_DESC(0)->rx_addrs = PMA_ADDR_FROM_APP(PACKET_RX_BUFFER_ADDR_0);
    uint32_t num_blocks = ENDPOINT0_RX_BUFFER_SIZE / 64; // because we check BLSIZE bit
    ENDPOINT_DESC(0)->rx_count = USB_COUNT0_RX_BLSIZE | (num_blocks << 10);

    // clear DTOG_RX bit (by toggling)
    if ((USB->EP0R & USB_EP0R_DTOG_RX) != 0) {
        USB->EP0R |= USB_EP0R_DTOG_RX | USB_EP0R_CTR_TX | USB_EP0R_CTR_RX;
    }

    // set rx endpoint status as valid
    set_endpoint_0_tx_status(USB_EP_RX_VALID);


    // set tx endpoint
    ENDPOINT_DESC(0)->tx_addrs = PMA_ADDR_FROM_APP(PACKET_TX_BUFFER_ADDR_0);
    ENDPOINT_DESC(0)->tx_count = 0UL;

    // clear DTOG_TX bit (also by toggling)
    if ((USB->EP0R & USB_EP0R_DTOG_TX) != 0) {
        USB->EP0R |= USB_EP0R_DTOG_TX | USB_EP0R_CTR_TX | USB_EP0R_CTR_RX;
    }

    // set tx endpoint status as valid
    set_endpoint_0_tx_status(USB_EP_TX_VALID);
}


static void configure_1_endpoint() {
}


static void zero_btable() {
    ENDPOINT_DESC(0)->rx_addrs = 0UL;
    ENDPOINT_DESC(0)->rx_count = 0UL;
    ENDPOINT_DESC(0)->tx_addrs = 0UL;
    ENDPOINT_DESC(0)->tx_count = 0UL;

    ENDPOINT_DESC(1)->rx_addrs = 0UL;
    ENDPOINT_DESC(1)->rx_count = 0UL;
    ENDPOINT_DESC(1)->tx_addrs = 0UL;
    ENDPOINT_DESC(1)->tx_count = 0UL;
}


static void set_endpoint_0_rx_status(uint32_t status) {
    uint32_t reg_write_value = 0UL;

    if (((USB->EP0R & USB_EPRX_DTOG1) ^ status) != 0) {
        reg_write_value |= USB_EPRX_DTOG1;
    }

    if (((USB->EP0R & USB_EPRX_DTOG2) ^ status) != 0) {
        reg_write_value |= USB_EPRX_DTOG2;
    }

    USB->EP0R |= reg_write_value | USB_EP0R_CTR_TX | USB_EP0R_CTR_RX;
}


static void set_endpoint_0_tx_status(uint32_t status) {
    uint32_t reg_write_value = 0UL;

    if (((USB->EP0R & USB_EPTX_DTOG1) ^ status) != 0) {
        reg_write_value |= USB_EPTX_DTOG1;
    }

    if (((USB->EP0R & USB_EPTX_DTOG2) ^ status) != 0) {
        reg_write_value |= USB_EPTX_DTOG2;
    }

    USB->EP0R |= reg_write_value;
}
