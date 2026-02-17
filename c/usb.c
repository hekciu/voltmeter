#include <stdint.h>
#include <assert.h>
#include <stddef.h>
#include <stdbool.h>

#include "cmsis.h"
#include "usb.h"
#include "led.h"
#include "systick.h"

static volatile uint32_t debug_count = 0;

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


#define ENDPOINT_0_ADDRESS 0
#define ENDPOINT_1_ADDRESS 1
#define BTABLE_ADDRESS 0x0

#define PMA_ENDPT_DESC_BYTES sizeof(pma_endpoint_desc_t)
#define ENDPT_NUM 8

#define ENDPOINT0_RX_BUFFER_SIZE 64
#define ENDPOINT0_TX_BUFFER_SIZE 64

#define ENDPOINT1_RX_BUFFER_SIZE 64
#define ENDPOINT1_TX_BUFFER_SIZE 64

#define PACKET_RX_BUFFER_ADDR_0 (USB_PMAADDR + PMA_ENDPT_DESC_BYTES * ENDPT_NUM)
#define PACKET_TX_BUFFER_ADDR_0 (PACKET_RX_BUFFER_ADDR_0 + ENDPOINT0_RX_BUFFER_SIZE)

#define PACKET_RX_BUFFER_ADDR_1 (PACKET_TX_BUFFER_ADDR_0 + ENDPOINT0_TX_BUFFER_SIZE)
#define PACKET_TX_BUFFER_ADDR_1 (PACKET_RX_BUFFER_ADDR_1 + ENDPOINT1_RX_BUFFER_SIZE)

#define PMA_ADDR_FROM_APP(n) (((n) - USB_PMAADDR) >> 1)
#define APP_ADDR_FROM_PMA(n) ((n << 1) + USB_PMAADDR)
#define USB_EP_REG_PTR(n) (&(USB->EP0R) + ((n) * 2U))


#define ENDPOINT_DESC(n) ((pma_endpoint_desc_t*)(USB_PMAADDR + (n)*sizeof(pma_endpoint_desc_t)))
//#define ENDPOINT_DESC(n) ((pma_endpoint_desc_t*)(0x40006000 + (n)*sizeof(pma_endpoint_desc_t)))

#define  GET_TWO_BYTES(addr) (((uint16_t)(*((uint8_t *)(addr)))) + (((uint16_t)(*(((uint8_t *)(addr)) + 1U))) << 8U))
#define  LOBYTE(x) ((uint8_t)((x) & 0x00FFU))
#define  HIBYTE(x) ((uint8_t)(((x) & 0xFF00U) >> 8U))

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a > b ? a : b)

/* Static variables */
static volatile uint8_t USB_Address = 0;


/* USB-specific definitions */
#define USBD_PRODUCT_STRING_FS                          "STM32 Learning Interface"
#define USBD_MANUFACTURER_STRING                        "STMicroelectronics"
#define USB_SIZ_STRING_SERIAL                           0x1A
#define UID_BASE                                        0x1FFFF7E8UL    /*!< Unique device ID register base address */
#define DEVICE_ID1                                      (UID_BASE)
#define DEVICE_ID2                                      (UID_BASE + 0x4)
#define DEVICE_ID3                                      (UID_BASE + 0x8)

#define  CUSTOM_HID_REPORT_DESC                         0x22U

#define  USB_REQ_RECIPIENT_DEVICE                       0x00U
#define  USB_REQ_RECIPIENT_INTERFACE                    0x01U

#define  USB_REQ_TYPE_STANDARD                          0x00U
#define  USB_REQ_TYPE_CLASS                             0x20U
#define  USB_REQ_TYPE_MASK                              0x60U

#define  USB_REQ_SET_ADDRESS                            0x05U
#define  USB_REQ_GET_DESCRIPTOR                         0x06U
#define  USB_REQ_SET_CONFIGURATION                      0x09U

#define  USB_DESC_TYPE_DEVICE                           0x01U
#define  USB_DESC_TYPE_CONFIGURATION                    0x02U
#define  USB_DESC_TYPE_STRING                           0x03U
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                 0x06U

#define  USBD_LANGID_STR                                0x00U

// Macros related to language descriptors
#define  USB_LEN_LANGID_STR_DESC                        0x04U
#define  USB_DESC_TYPE_STRING                           0x03U
#define  USBD_LANGID_STRING                             1033

// Macros related to FS descriptors
#define  USB_DESC_TYPE_CONFIGURATION                    0x02U
#define  USB_CUSTOM_HID_CONFIG_DESC_SIZ                 0x29U
#define  USB_DESC_TYPE_INTERFACE                        0x04U
#define  CUSTOM_HID_DESCRIPTOR_TYPE                     0x21U
#define  USBD_CUSTOM_HID_REPORT_DESC_SIZE               0x03U
#define  USB_DESC_TYPE_ENDPOINT                         0x05U
#define  CUSTOM_HID_EPIN_ADDR                           0x81U
#define  CUSTOM_HID_EPIN_SIZE                           0x02U
#define  CUSTOM_HID_FS_BINTERVAL                        0x05U
#define  USB_DESC_TYPE_ENDPOINT                         0x05U
#define  CUSTOM_HID_EPOUT_ADDR                          0x01U
#define  CUSTOM_HID_EPOUT_SIZE                          0x02U
#define  CUSTOM_HID_FS_BINTERVAL                        0x05U

// Device descriptor related macros
#define  USB_DESC_TYPE_DEVICE                           0x01U
#define  USB_MAX_EP0_SIZE                               64U
#define  USBD_VID                                       1155
#define  USBD_PID_FS                                    22362
#define  USBD_IDX_MFC_STR                               0x01U
#define  USBD_IDX_PRODUCT_STR                           0x02U
#define  USBD_IDX_SERIAL_STR                            0x03U
#define  USBD_MAX_NUM_CONFIGURATION                     1
#define  USBD_STRING_DESC_SIZE                          0x100

#define  QUAL_DESC_SIZE                                 0x0A
#define  USB_DESC_TYPE_DEV_QUALIFIER                    0x06


/* Device descriptors and stuff, mostly copied from mshafeeqkn's examples */

uint8_t dev_desc[0x12]  __attribute__ ((aligned (4))) =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB */
  0x02,
  0x00,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0x00,                       /*bcdDevice rel. 2.00*/
  0x02,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

uint8_t fs_config[41] __attribute__ ((aligned (4))) = {
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_CUSTOM_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xC0,         /*bmAttributes: bus powered */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x02,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: CUSTOM_HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 18 */
  0x09,         /*bLength: CUSTOM_HID Descriptor size*/
  CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
  0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
  0x00,
  CUSTOM_HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
  /* 34 */

  0x07,          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
  0x03, /* bmAttributes: Interrupt endpoint */
  CUSTOM_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  CUSTOM_HID_FS_BINTERVAL,  /* bInterval: Polling Interval */
  /* 41 */
};

static uint8_t qual_desc[QUAL_DESC_SIZE]  __attribute__ ((aligned (4))) = {
    QUAL_DESC_SIZE,
    USB_DESC_TYPE_DEV_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    ENDPOINT0_TX_BUFFER_SIZE,
    USBD_MAX_NUM_CONFIGURATION,
    0x00
};

uint8_t lang_desc[USB_LEN_LANGID_STR_DESC]  __attribute__ ((aligned (4))) = {
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

uint8_t serial_str[USB_SIZ_STRING_SERIAL] __attribute__ ((aligned (4))) = {
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};

static uint8_t report_desc[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __attribute__ ((aligned (4))) = {
  0xA1, 0x01,
  0xC0
};

uint8_t str_desc[USBD_STRING_DESC_SIZE]  __attribute__ ((aligned (4)));


/* Static function declarations */
static void zero_btable(void);
static void on_usb_reset(void);
static void configure_0_endpoint(void);
static void configure_1_endpoint(void);
static inline uint32_t get_rx_count(uint8_t endpoint);
static void service_correct_transfer_intr(void);
static void control_endpoint_CTR_handler(void);
static inline void set_ep_tx_status(uint8_t ep, uint32_t status);
static inline void set_ep_rx_status(uint8_t ep, uint32_t status);
static inline void clear_tx_ep_ctr(uint8_t ep);
static inline void clear_rx_ep_ctr(uint8_t ep);
static void process_setup_messages(void);
static void process_descriptor_request(usb_ctrl_req_t* request);
static void process_standard_request(usb_ctrl_req_t* request);
static void process_string_request(usb_ctrl_req_t* request);
static void usb_send_data(uint8_t ep, const uint8_t* buffer, uint16_t len);
static void read_data_from_pma(uint16_t rx_addrs, uint8_t* out, uint16_t rx_count);
static void write_data_to_pma(uint16_t tx_addrs, const uint8_t* src, uint16_t tx_count);
static void parse_ctrl_message(uint8_t* data, usb_ctrl_req_t* request);
static void convert_str_to_desc(uint8_t *desc, uint8_t *unicode, uint16_t *len);
static uint8_t get_len(const uint8_t *buf);
static void get_serial_num(void);
static void int_to_unicode(uint32_t value, uint8_t * pbuf, uint8_t len);


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
    // 1 endpoint have interrupt type
    USB->EP1R &= ~USB_EP0R_EP_TYPE_Msk;
    // we always write CTR_TX and CTR_RX so to not clear bit accidentally
    USB->EP1R |= USB_EP_INTERRUPT | USB_EP0R_CTR_TX | USB_EP0R_CTR_RX;

    // set 1 endpoint address
    USB->EP1R &= ~USB_EP1R_EA_Msk;
    USB->EP1R |= (ENDPOINT_1_ADDRESS << USB_EP1R_EA_Pos) | USB_EP1R_CTR_TX | USB_EP1R_CTR_RX;

    static_assert(ENDPOINT1_RX_BUFFER_SIZE > 62);

    // set rx endpoint
    ENDPOINT_DESC(1)->rx_addrs = PMA_ADDR_FROM_APP(PACKET_RX_BUFFER_ADDR_1);
    uint32_t num_blocks = ENDPOINT1_RX_BUFFER_SIZE / 64; // because we check BLSIZE bit
    ENDPOINT_DESC(1)->rx_count = USB_COUNT1_RX_BLSIZE | (num_blocks << 10);

    // clear DTOG_RX bit (by toggling)
    if ((USB->EP1R & USB_EP1R_DTOG_RX) != 0) {
        USB->EP1R |= USB_EP1R_DTOG_RX | USB_EP1R_CTR_TX | USB_EP1R_CTR_RX;
    }

    // set rx endpoint status as valid
    set_ep_rx_status(1, USB_EP_RX_VALID);


    // set tx endpoint
    ENDPOINT_DESC(1)->tx_addrs = PMA_ADDR_FROM_APP(PACKET_TX_BUFFER_ADDR_1);
    ENDPOINT_DESC(1)->tx_count = 0UL;

    // clear DTOG_TX bit (also by toggling)
    if ((USB->EP1R & USB_EP1R_DTOG_TX) != 0) {
        USB->EP1R |= USB_EP1R_DTOG_TX | USB_EP1R_CTR_TX | USB_EP1R_CTR_RX;
    }

    // set tx endpoint status as nak 
    set_ep_tx_status(1, USB_EP_TX_NAK);
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
        } else {
            uint16_t ep_reg_value = *USB_EP_REG_PTR(endpoint);

            if((ep_reg_value & USB_EP_CTR_RX) != 0) {
                clear_rx_ep_ctr(endpoint);

                uint16_t xfer_count = ENDPOINT_DESC(endpoint)->rx_count & 0x3FF;

                // we could read data or something
                set_ep_rx_status(endpoint, USB_EP_RX_VALID);
            } else if((ep_reg_value & USB_EP_CTR_TX) != 0) {
                clear_rx_ep_ctr(endpoint);
            }
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


static inline void set_ep_tx_status(uint8_t ep, uint32_t status) {
    uint16_t reg_write_value = *USB_EP_REG_PTR(ep) & USB_EPTX_DTOGMASK;

    if ((USB_EPTX_DTOG1 & status) != 0U) {
        reg_write_value ^= USB_EPTX_DTOG1;
    }

    if ((USB_EPTX_DTOG2 & status) != 0U) {
        reg_write_value ^= USB_EPTX_DTOG2;
    }

    *USB_EP_REG_PTR(ep) = reg_write_value | USB_EP_CTR_TX | USB_EP_CTR_RX;
}


static inline void set_ep_rx_status(uint8_t ep, uint32_t status) {
    uint16_t reg_write_value = *USB_EP_REG_PTR(ep) & USB_EPRX_DTOGMASK;

    if ((USB_EPRX_DTOG1 & status) != 0U) {
        reg_write_value ^= USB_EPRX_DTOG1;
    }

    if ((USB_EPRX_DTOG2 & status) != 0U) {
        reg_write_value ^= USB_EPRX_DTOG2;
    }

    *USB_EP_REG_PTR(ep) = reg_write_value | USB_EP_CTR_TX | USB_EP_CTR_RX;
}


static inline void clear_tx_ep_ctr(uint8_t ep) {
    uint16_t reg_write_value = *USB_EP_REG_PTR(ep) & (0x7FFFU & USB_EPREG_MASK);
    *USB_EP_REG_PTR(ep) = reg_write_value | USB_EP_CTR_TX;
}


static inline void clear_rx_ep_ctr(uint8_t ep) {
    if (ep == 0) {
        uint16_t reg_write_value = USB->EP0R & (0x7FFFU & USB_EPREG_MASK);
        USB->EP0R = reg_write_value | USB_EP_CTR_RX;
    }

    if (ep == 1) {
        uint16_t reg_write_value = USB->EP1R & (0x7FFFU & USB_EPREG_MASK);
        USB->EP1R = reg_write_value | USB_EP_CTR_RX;
    }
}


static void process_setup_messages() {
    usb_ctrl_req_t request;
    const uint8_t endpoint = 0;
    uint8_t *buff = NULL;
    uint8_t xfer_data[16] = {0};
    uint16_t len;

    uint16_t xfer_count = get_rx_count(0);
    read_data_from_pma(ENDPOINT_DESC(0)->rx_addrs, xfer_data, xfer_count);

    clear_rx_ep_ctr(0);
    parse_ctrl_message(xfer_data, &request);

    switch(request.request_type & 0x1F) {
        case USB_REQ_RECIPIENT_DEVICE:
            // Recepient is a device
            if(USB_REQ_TYPE_STANDARD == (request.request_type & USB_REQ_TYPE_MASK)) {
                // Request type is standard
                process_standard_request(&request);
            }
            break;

        case USB_REQ_RECIPIENT_INTERFACE:
            if((request.request_type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD) {
                if(USB_REQ_GET_DESCRIPTOR == request.request) {
                    if(CUSTOM_HID_REPORT_DESC == (request.value >> 8)) {
                        // 81 06 00 22 00 00 03 00
                        buff = report_desc;
                        len = MIN(request.length, 163);
                        usb_send_data(0, buff, len);
                    }
                }
            }

            if((request.request_type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS) {
                // 21 0A 00 00 00 00 00 00
                usb_send_data(0, NULL, 0);
            }

            break;
    }
}


static void read_data_from_pma(uint16_t rx_addrs, uint8_t* out, uint16_t rx_count) {
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


static void write_data_to_pma(uint16_t tx_addrs, const uint8_t* src, uint16_t tx_count) {
    // we write two bytes every time
    uint16_t count = (tx_count + 1) >> 1;
    uint16_t write_value = 0;

    volatile uint16_t* pma_addr = (volatile uint16_t*)APP_ADDR_FROM_PMA(tx_addrs);

    for (; count != 0; count--) {
        write_value = src[0];
        write_value |= src[1] << 8;
        *pma_addr = write_value;
        pma_addr++;
        pma_addr++;
        src++;
        src++;
    }
}



static void parse_ctrl_message(uint8_t* data, usb_ctrl_req_t* req) {
    req->request_type = *(uint8_t *)(data);
    req->request = *(uint8_t *)(data + 1U);
    req->value = GET_TWO_BYTES(data + 2U);
    req->index = GET_TWO_BYTES(data + 4U);
    req->length = GET_TWO_BYTES(data + 6U);
}


static void process_standard_request(usb_ctrl_req_t* request) {
    switch(request->request) {
        case USB_REQ_SET_ADDRESS:
            // 00 05 02 00 00 00 00 00
            USB_Address = request->value & 0x7F;
            usb_send_data(0, NULL, 0);
            break;

        case USB_REQ_GET_DESCRIPTOR:
            // 80 06 00 06 00 00 0A 00
            process_descriptor_request(request);
            break;

        case USB_REQ_SET_CONFIGURATION:
            // 00 09 01 00 00 00 00 00
            //uint8_t cfg_idx = request->value;
            configure_1_endpoint();
            usb_send_data(0, NULL, 0);
            break;
    }
}


static void process_descriptor_request(usb_ctrl_req_t* request) {
    uint8_t *buff = NULL;
    uint16_t len;

    switch(request->value >> 8) {
        case USB_DESC_TYPE_DEVICE:
            // 80 06 00 01 00 00 40 00
            // FIXME: ten case wywołuje sie w nieskończoność
            debug_count++;
            buff = dev_desc;
            len = sizeof(dev_desc);
            usb_send_data(0, buff, len);
            break;

        case USB_DESC_TYPE_CONFIGURATION:
            // 80 06 00 02 00 00 09 00
            buff = fs_config;
            len = MIN(request->length, sizeof(fs_config));
            usb_send_data(0, buff, len);
            break;

        case USB_DESC_TYPE_STRING:
            process_string_request(request);
            break;

        case USB_DESC_TYPE_DEVICE_QUALIFIER:
            // 80 06 00 06 00 00 0A 00
            buff = qual_desc;
            len = sizeof(qual_desc);
            usb_send_data(0, buff, len);
            break;
    }
}


static void process_string_request(usb_ctrl_req_t* request) {
    uint8_t *buff = NULL;
    uint16_t len;

    switch(request->value & 0xFF) {
        case USBD_LANGID_STR:
            // 80 06 00 03 00 00 FF 00
            buff = lang_desc;
            len = sizeof(lang_desc);
            usb_send_data(0, buff, len);
            break;

        case USBD_IDX_MFC_STR:
            // 80 06 01 03 09 04 FF 00
            convert_str_to_desc((uint8_t*)USBD_MANUFACTURER_STRING, str_desc, &len);
            buff = str_desc;
            usb_send_data(0, buff, len);
            break;

        case USBD_IDX_PRODUCT_STR:
            // 80 06 02 03 09 04 FF 00
            convert_str_to_desc((uint8_t*)USBD_PRODUCT_STRING_FS, str_desc, &len);
            buff = str_desc;
            usb_send_data(0, buff, len);
            break;

        case USBD_IDX_SERIAL_STR:
            // 80 06 03 03 09 04 FF 00
            len = USB_SIZ_STRING_SERIAL;
            get_serial_num();
            buff = serial_str;
            usb_send_data(0, buff, len);
            break;
    }
}


static void usb_send_data(uint8_t ep, const uint8_t* buffer, uint16_t len) {
    write_data_to_pma(ENDPOINT_DESC(ep)->tx_addrs, buffer, len);
    ENDPOINT_DESC(ep)->tx_count = len;
    set_ep_tx_status(ep, USB_EP_TX_VALID);
}


static void convert_str_to_desc(uint8_t *desc, uint8_t *unicode, uint16_t *len) {
    uint8_t idx = 0U;

    if (desc != NULL) {
        *len = get_len(desc) * 2U + 2U;
        unicode[idx++] = *(uint8_t *)(void *)len;
        unicode[idx++] = USB_DESC_TYPE_STRING;

        while (*desc != '\0') {
            unicode[idx++] = *desc++;
            unicode[idx++] =  0U;
        }
    }
}


static uint8_t get_len(const uint8_t *buf) {
    uint8_t output = 0;

    while (*buf != '\0') {
        output++;
        buf++;
    }

    return output;
}


static void get_serial_num(void) {
    uint32_t device_serial_0;
    uint32_t device_serial_1;
    uint32_t device_serial_2;

    device_serial_0 = *(uint32_t *) DEVICE_ID1;
    device_serial_1 = *(uint32_t *) DEVICE_ID2;
    device_serial_2 = *(uint32_t *) DEVICE_ID3;

    device_serial_0 += device_serial_2;

    if (device_serial_0 != 0) {
        int_to_unicode(device_serial_0, &serial_str[2], 8);
        int_to_unicode(device_serial_1, &serial_str[18], 4);
    }
}


static void int_to_unicode(uint32_t value, uint8_t * pbuf, uint8_t len) {
    uint8_t idx = 0;

    for (idx = 0; idx < len; idx++) {
        if (((value >> 28)) < 0xA) {
            pbuf[2 * idx] = (value >> 28) + '0';
        } else {
            pbuf[2 * idx] = (value >> 28) + 'A' - 10;
        }

        value = value << 4;

        pbuf[2 * idx + 1] = 0;
    }
}


void usb_send_string(const char* data) {
    usb_send_data(1, data, get_len(data));
}
