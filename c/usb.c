#include <stdint.h>
#include <assert.h>
#include <stddef.h>
#include <stdbool.h>

#include "cmsis.h"
#include "usb.h"
#include "led.h"
#include "systick.h"

static uint32_t debug_count = 0;

typedef struct {
    uint32_t tx_addrs;
    uint32_t tx_count;
    uint32_t rx_addrs;
    uint32_t rx_count; 
} pma_endpoint_desc_t;


typedef struct {
    uint8_t request_type;
    uint8_t request;
    uint16_t value;
    uint16_t index;
    uint16_t length;
} usb_ctrl_req_t;


#define ENDPOINT_DESC(n) ((pma_endpoint_desc_t*)(USB_PMAADDR + (n)*sizeof(pma_endpoint_desc_t)))


#define ENDPOINT_0_ADDRESS 0
#define BTABLE_ADDRESS 0x0

#define PMA_ENDPT_DESC_BYTES sizeof(pma_endpoint_desc_t)
#define ENDPT_NUM 8

#define ENDPOINT0_RX_BUFFER_SIZE 256

#define PACKET_RX_BUFFER_ADDR_0 (USB_PMAADDR + PMA_ENDPT_DESC_BYTES * ENDPT_NUM)
#define PACKET_TX_BUFFER_ADDR_0 (PACKET_RX_BUFFER_ADDR_0 + ENDPOINT0_RX_BUFFER_SIZE)

#define PMA_ADDR_FROM_APP(n) (((n) - USB_PMAADDR) >> 1)
#define APP_ADDR_FROM_PMA(n) ((1 << n) + USB_PMAADDR)
#define USB_EP_REG_PTR(n) (&(USB->EP0R) + ((n) * 2U))

/*  */
#define  GET_TWO_BYTES(addr) (((uint16_t)(*((uint8_t *)(addr)))) + (((uint16_t)(*(((uint8_t *)(addr)) + 1U))) << 8U))
#define  LOBYTE(x) ((uint8_t)((x) & 0x00FFU))
#define  HIBYTE(x) ((uint8_t)(((x) & 0xFF00U) >> 8U))

/* Static variables */
static uint8_t USB_Address = 0;

/* Descriptors definitions */



/* Static function declarations */
static void zero_btable(void);
static void on_usb_reset(void);
static void configure_0_endpoint(void);
static void configure_1_endpoint(void);
static void process_descriptor_request(void);
static inline uint32_t get_rx_count(uint8_t endpoint);
static void service_correct_transfer_intr(void);
static void control_endpoint_CTR_handler(void);
static inline void set_ep_tx_status(uint32_t ep, uint32_t status);
static inline void set_ep_rx_status(uint32_t ep, uint32_t status);
static inline void clear_tx_ep_ctr(uint32_t ep);
static inline void clear_rx_ep_ctr(uint32_t ep);
static void process_setup_messages();
static void read_data_from_pma(uint16_t rx_addrs, uint8_t* out, uint16_t rx_count);
static void parse_ctrl_message(uint8_t* data, usb_ctrl_req_t* request);


volatile static uint16_t ISTR_reg_data = 0;

void usb_hp_handler(void) {
    //ISTR_reg_data = USB->ISTR;
    //led_toggle();
}


void usb_lp_handler(void) {
    ISTR_reg_data = USB->ISTR;

    if (ISTR_reg_data & USB_ISTR_RESET) {
        on_usb_reset();
        USB->ISTR &= ~USB_ISTR_RESET;
    }

    if (ISTR_reg_data & USB_ISTR_SOF) {
        USB->ISTR &= ~USB_ISTR_SOF;
    }

    if (ISTR_reg_data & USB_ISTR_ESOF) {
        USB->ISTR &= ~USB_ISTR_ESOF;
    }

    if (ISTR_reg_data & USB_ISTR_SUSP) {
        USB->ISTR &= ~USB_ISTR_SUSP;
    }

    if (ISTR_reg_data & USB_ISTR_WKUP) {
        USB->ISTR &= ~USB_ISTR_WKUP;
    }

    if (ISTR_reg_data & USB_ISTR_ERR) {
        USB->ISTR &= ~USB_ISTR_ERR;
    }

    if (ISTR_reg_data & USB_ISTR_PMAOVR) {
        USB->ISTR &= ~USB_ISTR_PMAOVR;
    }

    if(USB->ISTR & USB_ISTR_CTR) {
        debug_count++;
        service_correct_transfer_intr();
    }
}


void usb_wakeup_handler(void) {
    //led_toggle();
    //ISTR_reg_data = USB->ISTR;

    //on_usb_reset();
}


void usb_initialize(void) {
    // enable usb clock
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    uint32_t usb_priority_group = NVIC_GetPriorityGrouping();

    //NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, NVIC_EncodePriority(usb_priority_group, 0, 0));
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(usb_priority_group, 0, 0));
    //NVIC_SetPriority(USBWakeUp_IRQn, NVIC_EncodePriority(usb_priority_group, 0, 0));

    //NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    //NVIC_EnableIRQ(USBWakeUp_IRQn);

    // de-assert macrocell specific reset signal
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST;
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    // reset and clear reset
    USB->CNTR |= USB_CNTR_FRES;
    USB->CNTR = 0UL;

    // BDT address is 0x0 relative to how USB peripheral see dedicated memory
    USB->BTABLE = BTABLE_ADDRESS;

    // switch on interval voltage for port transceiver
    // USB->CNTR &= ~USB_CNTR_PDWN;
    // wait for t_startup (1 us)
    // systick_wait_ms(1); /* We actually wait 1000x longer but I guess it shouldn't be a problem */

    // remove any spurious pending interrupt
    USB->ISTR = 0UL;

    // enable reset interrupt
    USB->CNTR |= USB_CNTR_RESETM | USB_CNTR_CTRM;

    // now we wait for RESET interrupt triggered on host connection
}


static void on_usb_reset(void) {
    zero_btable();

    // configure endpoints
    configure_0_endpoint();
    //configure_1_endpoint();

    // enable USB device
    USB->DADDR |= USB_DADDR_EF;

    // enable other interrupts (besides  RESETM and CTRM)
    USB->CNTR |= USB_CNTR_PMAOVRM;
    USB->CNTR |= USB_CNTR_ERRM;
    USB->CNTR |= USB_CNTR_WKUPM;
    USB->CNTR |= USB_CNTR_SUSPM;
    USB->CNTR |= USB_CNTR_SOFM;
    USB->CNTR |= USB_CNTR_ESOFM;
}


static void configure_0_endpoint(void) {
    // 0 endpoint must always have CONTROL type
    USB->EP0R &= ~USB_EP0R_EP_TYPE_Msk;
    // we always write CTR_TX and CTR_RX so to not clear bit accidentally
    USB->EP0R |= USB_EP_CONTROL | USB_EP0R_CTR_TX | USB_EP0R_CTR_RX;

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
    set_ep_rx_status(0, USB_EP_RX_VALID);


    // set tx endpoint
    ENDPOINT_DESC(0)->tx_addrs = PMA_ADDR_FROM_APP(PACKET_TX_BUFFER_ADDR_0);
    ENDPOINT_DESC(0)->tx_count = 0UL;

    // clear DTOG_TX bit (also by toggling)
    if ((USB->EP0R & USB_EP0R_DTOG_TX) != 0) {
        USB->EP0R |= USB_EP0R_DTOG_TX | USB_EP0R_CTR_TX | USB_EP0R_CTR_RX;
    }

    // set tx endpoint status as nak 
    set_ep_tx_status(0, USB_EP_TX_NAK);
}


static void configure_1_endpoint(void) {
}


static void zero_btable(void) {
    ENDPOINT_DESC(0)->rx_addrs = 0UL;
    ENDPOINT_DESC(0)->rx_count = 0UL;
    ENDPOINT_DESC(0)->tx_addrs = 0UL;
    ENDPOINT_DESC(0)->tx_count = 0UL;

    ENDPOINT_DESC(1)->rx_addrs = 0UL;
    ENDPOINT_DESC(1)->rx_count = 0UL;
    ENDPOINT_DESC(1)->tx_addrs = 0UL;
    ENDPOINT_DESC(1)->tx_count = 0UL;
}


static inline uint32_t get_rx_count(uint8_t endpoint) {
    return ENDPOINT_DESC(endpoint)->rx_count & 0x3FF; // first 10 bits
}


static void service_correct_transfer_intr(void) {
    uint8_t endpoint;

    while (USB->ISTR & USB_ISTR_CTR) {
        endpoint = USB->ISTR & USB_ISTR_EP_ID;

        if (endpoint == 0) {
            control_endpoint_CTR_handler();
        }
    }
}


static void control_endpoint_CTR_handler(void) {
    const uint8_t endpoint = 0;
    const uint32_t ep_reg_val = USB->EP0R;

    if ((USB->ISTR & USB_ISTR_DIR) == 0) {
        // DIR == 0 means IN transaction and CTR_TX definitely is 1
        clear_tx_ep_ctr(endpoint);

        if (USB_Address != 0) {
            USB->DADDR = USB_Address | USB_DADDR_EF;
            USB_Address = 0;
            set_ep_tx_status(0, USB_EP_TX_STALL);
        } else {
            set_ep_tx_status(0, USB_EP_TX_STALL);
            set_ep_rx_status(0, USB_EP_RX_VALID);
        }
    } else {
        if (ep_reg_val & USB_EP_SETUP) {
            process_setup_messages();
        } else if ((ep_reg_val & USB_EP_CTR_RX) != 0) {
            clear_rx_ep_ctr(endpoint);
            ENDPOINT_DESC(0)->rx_count = ((1 << 10) | USB_COUNT0_RX_BLSIZE);
            set_ep_rx_status(0, USB_EP_RX_VALID);
        }
    }
}


static inline void set_ep_tx_status(uint32_t ep, uint32_t status) {
    uint16_t reg_write_value = *USB_EP_REG_PTR(ep) & USB_EPTX_DTOGMASK;

    if ((USB_EPTX_DTOG1 & status) != 0U) {
        reg_write_value ^= USB_EPTX_DTOG1;
    }

    if ((USB_EPTX_DTOG2 & status) != 0) {
        reg_write_value ^= USB_EPTX_DTOG2;
    }

    *USB_EP_REG_PTR(ep) = reg_write_value | USB_EP_CTR_TX | USB_EP_CTR_RX;
}


static inline void set_ep_rx_status(uint32_t ep, uint32_t status) {
    uint16_t reg_write_value = *USB_EP_REG_PTR(ep);

    if ((USB_EPRX_DTOG1 & status) != 0U) {
        reg_write_value ^= USB_EPRX_DTOG1;
    }

    if ((USB_EPRX_DTOG2 & status) != 0) {
        reg_write_value ^= USB_EPRX_DTOG2;
    }

    *USB_EP_REG_PTR(ep) = reg_write_value | USB_EP_CTR_TX | USB_EP_CTR_RX;
}


static inline void clear_tx_ep_ctr(uint32_t ep) {
    *USB_EP_REG_PTR(ep) &= ~(~USB_EPREG_MASK | USB_EP_CTR_TX);
    //*USB_EP_REG_PTR(ep) &= USB_EP_CTR_TX;
}


static inline void clear_rx_ep_ctr(uint32_t ep) {
    *USB_EP_REG_PTR(ep) &= ~(~USB_EPREG_MASK | USB_EP_CTR_RX);
    //*USB_EP_REG_PTR(ep) &= USB_EP_CTR_RX;
}


static void process_setup_messages() {
    usb_ctrl_req_t request;
    const uint8_t endpoint = 0;
    uint8_t *buff = NULL;
    uint8_t xfer_data[16] = {0};
    uint16_t len;
    uint16_t xfer_count;

    uint16_t vfer_count = get_rx_count(0);
    read_data_from_pma(ENDPOINT_DESC(0)->rx_addrs, xfer_data, xfer_count);

    clear_rx_ep_ctr(0);
    parse_ctrl_message(xfer_data, &request);
}


static void read_data_from_pma(uint16_t rx_addrs, uint8_t* out, uint16_t rx_count) {
    led_toggle();
    // we read two bytes every time
    uint16_t count = rx_count >> 1;
    uint16_t value;

    uint16_t* pma_addr = (uint16_t*)APP_ADDR_FROM_PMA(rx_addrs);

    for (; count != 0; count--) {
        value = *pma_addr;

        pma_addr += 2;
        *out = (uint8_t)(value & 0xFF);
        out++;
        *out = (uint8_t)((value >> 8) & 0xFF);
        out++;
    }
}


static void parse_ctrl_message(uint8_t* data, usb_ctrl_req_t* req) {
    req->request_type = *(uint8_t *)(data);
    req->request = *(uint8_t *)(data + 1U);
    req->value = GET_TWO_BYTES(data + 2U);
    req->index = GET_TWO_BYTES(data + 4U);
    req->length = GET_TWO_BYTES(data + 6U);
}
