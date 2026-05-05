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
// Composite device: Interface 0 = WLAN (vendor), Interface 1 = Bluetooth
// This mimics real CYW4373 which has both WiFi and BT in one chip.
// Without composite BT interface, Canon expects USB hub + 2 devices.
//--------------------------------------------------------------------
#define CONFIG_TOTAL_LEN    (9 + 9 + 7 + 7 + 7 + 9 + 7)

uint8_t const desc_fs_configuration[] = {
    // Configuration descriptor
    9, TUSB_DESC_CONFIGURATION,
    U16_TO_U8S_LE(CONFIG_TOTAL_LEN),
    2,                      // bNumInterfaces: 0=WLAN, 1=BT
    1,                      // bConfigurationValue
    0,                      // iConfiguration
    0xA0,                   // bmAttributes: bus-powered + remote wakeup
    0xFA,                   // bMaxPower: 500 mA

    // === Interface 0: WLAN (Vendor Specific) ===
    9, TUSB_DESC_INTERFACE,
    0,                      // bInterfaceNumber
    0,                      // bAlternateSetting
    3,                      // bNumEndpoints
    TUSB_CLASS_VENDOR_SPECIFIC,
    0x02,                    // bInterfaceSubClass, brcmfmac requires 2
    0xFF,                    // bInterfaceProtocol
    0,                       // iInterface

    // EP81 IN Interrupt - event/heartbeat
    7, TUSB_DESC_ENDPOINT,
    0x81,
    TUSB_XFER_INTERRUPT,
    U16_TO_U8S_LE(16),
    9,

    // EP82 IN Bulk - data RX
    7, TUSB_DESC_ENDPOINT,
    0x82,
    TUSB_XFER_BULK,
    U16_TO_U8S_LE(64),
    0,

    // EP02 OUT Bulk - data TX / firmware download
    7, TUSB_DESC_ENDPOINT,
    0x02,
    TUSB_XFER_BULK,
    U16_TO_U8S_LE(64),
    0,

    // === Interface 1: Bluetooth (Wireless Controller) ===
    // Dummy interface so host sees WiFi+BT composite like real CYW4373
    9, TUSB_DESC_INTERFACE,
    1,                      // bInterfaceNumber
    0,                      // bAlternateSetting
    1,                      // bNumEndpoints
    0xE0,                   // bInterfaceClass = Wireless Controller
    0x01,                   // bInterfaceSubClass = Bluetooth
    0x01,                   // bInterfaceProtocol = Bluetooth
    0,                      // iInterface

    // EP83 IN Interrupt - dummy BT event endpoint (never sends data)
    7, TUSB_DESC_ENDPOINT,
    0x83,
    TUSB_XFER_INTERRUPT,
    U16_TO_U8S_LE(16),
    9,
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
