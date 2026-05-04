#include "tusb.h"
#include "pico/stdlib.h"
#include <string.h>

// Dynamic mode: true = boot 04b4:bd29, false = runtime 04b4:0bdc.
extern volatile bool boot_mode;

//--------------------------------------------------------------------
// Device descriptors
//--------------------------------------------------------------------
tusb_desc_device_t const desc_device_boot = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0xFF, // USB_CLASS_VENDOR_SPEC
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x04B4,
    .idProduct          = 0xBD29,
    .bcdDevice          = 0x0001,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

tusb_desc_device_t const desc_device_runtime = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0xFF, // USB_CLASS_VENDOR_SPEC
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x04B4,
    .idProduct          = 0x0BDC,
    .bcdDevice          = 0x0001,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

//--------------------------------------------------------------------
// Configuration descriptor
//
// Critical brcmfmac requirement:
//   endpoint index 0 of the vendor interface must be INTERRUPT IN.
//   Then data endpoints must be BULK.
//
// Real/log-compatible endpoint set:
//   EP81 interrupt IN
//   EP82 bulk IN
//   EP02 bulk OUT
//--------------------------------------------------------------------
#define CONFIG_TOTAL_LEN    (9 + 9 + 7 + 7 + 7)

uint8_t const desc_fs_configuration[] = {
    // Configuration descriptor
    9, TUSB_DESC_CONFIGURATION,
    U16_TO_U8S_LE(CONFIG_TOTAL_LEN),
    1,                      // bNumInterfaces
    1,                      // bConfigurationValue
    0,                      // iConfiguration
    0xA0,                   // bmAttributes: bus-powered + remote wakeup
    0xFA,                   // bMaxPower: 500 mA (CYW4373 needs 350mA+ TX)

    // Interface descriptor: FF/02/FF
    9, TUSB_DESC_INTERFACE,
    0,                      // bInterfaceNumber
    0,                      // bAlternateSetting
    3,                      // bNumEndpoints
    TUSB_CLASS_VENDOR_SPECIFIC,
    0x02,                    // bInterfaceSubClass, brcmfmac requires 2
    0xFF,                    // bInterfaceProtocol, brcmfmac requires 0xFF
    0,

    // EP81 IN Interrupt - first endpoint, required by brcmfmac probe.
    7, TUSB_DESC_ENDPOINT,
    0x81,
    TUSB_XFER_INTERRUPT,
    U16_TO_U8S_LE(16),
    9,                      // bInterval from real-log style; FS interprets as 9 ms

    // EP82 IN Bulk - data RX pipe from host perspective.
    7, TUSB_DESC_ENDPOINT,
    0x82,
    TUSB_XFER_BULK,
    U16_TO_U8S_LE(64),
    0,

    // EP02 OUT Bulk - firmware download / TX pipe from host perspective.
    7, TUSB_DESC_ENDPOINT,
    0x02,
    TUSB_XFER_BULK,
    U16_TO_U8S_LE(64),
    0,
};

//--------------------------------------------------------------------
// Descriptor callbacks
//--------------------------------------------------------------------
uint8_t const * tud_descriptor_device_cb(void) {
    return (uint8_t const *)(boot_mode ? &desc_device_boot : &desc_device_runtime);
}

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_fs_configuration;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    static uint16_t desc_str[64];
    uint8_t chr_count;
    const char *str;

    if (index == 0) {
        desc_str[1] = 0x0409;
        desc_str[0] = (TUSB_DESC_STRING << 8) | (2 + 2);
        return desc_str;
    }

    switch (index) {
        case 1:
            str = "Cypress Semiconductor Corp.";
            break;
        case 2:
            str = boot_mode ? "Remote Download Wireless Adapter"
                            : "Cypress USB 802.11 Wireless Adapter";
            break;
        case 3:
            str = "000000000001";
            break;
        default:
            return NULL;
    }

    chr_count = (uint8_t)strlen(str);
    if (chr_count > 63) chr_count = 63;

    for (uint8_t i = 0; i < chr_count; i++) {
        desc_str[1 + i] = (uint8_t)str[i];
    }

    desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return desc_str;
}
