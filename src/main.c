#include "pico/stdlib.h"
#include "pico/stdio_uart.h"
#include "bsp/board.h"
#include "tusb.h"
#include "hardware/sync.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

//--------------------------------------------------------------------
// Pins
//--------------------------------------------------------------------
#define WL_REG_ON_PIN   2   // Canon pin 1 -> GPIO2: Power Control / WL_REG_ON

//--------------------------------------------------------------------
// Status LED
//
// Waveshare RP2040-Zero has onboard WS2812/RGB LED on GPIO16.
// GPIO25 is also toggled as simple external LED output.
//--------------------------------------------------------------------
#define STATUS_WS2812_PIN      16
#define STATUS_GPIO_LED_PIN    25
#define STATUS_LED_ENABLE      1

// 0 = wait forever. For bench test without Canon, tie GPIO2 to 3.3V.
#define WL_REG_ON_WAIT_TIMEOUT_MS 0u
#define POWER_STABLE_DELAY_MS 200u  // Fast attach test: after WL_REG_ON, wait 200 ms before TinyUSB
#define FW_PROGRESS_LOG_STEP 131072u // log firmware progress every 128 KB
#define LOG_EVERY_GETSTATE 0         // do not print every DL_GETSTATE

//--------------------------------------------------------------------
// Broadcom/Cypress download protocol
//--------------------------------------------------------------------
#define DL_GETSTATE     0
#define DL_CHECK_CRC    1
#define DL_GO           2
#define DL_START        3
#define DL_REBOOT       4
#define DL_GETVER       5
#define DL_GO_PROTECTED 6
#define DL_EXEC         7
#define DL_RESETCFG     8

#define DL_WAITING      0
#define DL_READY        1
#define DL_RUNNABLE     4

#define BRCMF_C_UP                  2
#define BRCMF_C_DOWN                3
#define BRCMF_C_GET_REVINFO         98
#define BRCMF_C_SET_SCAN_CHANNEL_TIME   185
#define BRCMF_C_SET_SCAN_UNASSOC_TIME   187
#define BRCMF_C_GET_PM                  85
#define BRCMF_C_SET_PM_COMPAT           86
#define BRCMF_C_IOCTL_244_COMPAT        244
#define BRCMF_C_GET_VAR             262
#define BRCMF_C_SET_VAR             263
#define BRCMF_POSTBOOT_ID           0xA123

//--------------------------------------------------------------------
// Real CYW4373 log values
//--------------------------------------------------------------------
#define CYW4373_CHIP_ID                 0x00004373u
#define CYW4373_CHIP_REV                0u
#define CYW4373_RAM_SIZE                0x000C0000u
#define CYW4373_EXPECTED_FW_SIZE        638976u
#define CYW4373_FW_VERSION_STRING       "wl0: Aug  3 2022 20:28:09 version 13.10.246.286 (4b0a74a CY) FWID 01-149d4809"
#define CYW4373_CLM_VERSION_STRING      "API: 18.1 Data: Murata.Type2AE Compiler: 1.35.0 ClmImport: 1.39.1 Customization: v3 23/02/09 Creation: 2023-02-09 04:01:47"
#define CYW4373_EVENT_MSGS_FILL         0xFFu

//--------------------------------------------------------------------
// Tolerance knobs
//--------------------------------------------------------------------
#define CYW4373_ASSUME_DL_START_ON_BULK          1
#define CYW4373_AUTO_RUNTIME_FALLBACK            0
#define CYW4373_AUTO_RUNTIME_FALLBACK_DELAY_MS   1200u
#define CYW4373_AUTO_RUNTIME_MIN_BYTES           638976u
#define CYW4373_RUNNABLE_FALLBACK_GETSTATE_POLLS 6u
#define CYW4373_DL_GO_RECONNECT_DELAY_MS         500u

//--------------------------------------------------------------------
// BCDC/DCMD header
//--------------------------------------------------------------------
typedef struct {
    uint32_t cmd;
    uint32_t len;
    uint32_t flags;
    uint32_t status;
} bcdc_hdr_t;

typedef struct {
    uint32_t chip;
    uint32_t chiprev;
    uint32_t ramsize;
    uint32_t romsize;
    uint32_t boardtype;
    uint32_t boardrev;
} bootrom_id_t;

typedef struct {
    uint32_t state;
    uint32_t bytes;
} rdl_state_t; // DL_GETSTATE is 8 bytes: state + bytes. Chip ID is DL_GETVER.

//--------------------------------------------------------------------
// Global state
//--------------------------------------------------------------------
volatile bool boot_mode = true;        // true = 04b4:bd29, false = 04b4:0bdc
volatile bool needs_reconnect = false;
volatile bool postboot = false;

static uint32_t fw_bytes_received = 0;
static uint32_t dl_state = DL_WAITING;
static absolute_time_t last_bulk_time;

static uint8_t bcdc_buf[512];
static uint8_t ctrl_buf[512];
static uint32_t last_bcdc_cmd = 0;
static uint32_t last_bcdc_len = 0;
static uint32_t last_bcdc_flags = 0;
static uint32_t current_pm = 0;
static bool runtime_out_pending = false;
static uint16_t runtime_out_len = 0;
static uint32_t runtime_out_seq = 0;
static uint32_t last_heartbeat_ms = 0;
static bool ep1_in_opened = false;
static volatile bool dongle_up = false;
static bool ep81_first_event = true;

// Backup iovar storage -- survives even if bcdc_buf is touched between OUT and IN
static char last_iovar_name[64] = {0};
static bool last_iovar_valid = false;
static uint32_t pending_if_event_ms = 0;


// Stateful country buffer (host may SET then GET)
static uint8_t current_country[12] = {
    'X','A',0,0,
    0,0,0,0,
    'X','A',0,0
};
static uint16_t current_country_len = 12;

// Simple stateful iovar storage for SET_VAR -> GET_VAR round-trips
#define IOVAR_STATE_MAX 16
#define IOVAR_NAME_MAX  32
#define IOVAR_DATA_MAX  128
typedef struct {
    bool used;
    char name[IOVAR_NAME_MAX];
    uint8_t data[IOVAR_DATA_MAX];
    uint16_t len;
} iovar_state_t;
static iovar_state_t iovar_state[IOVAR_STATE_MAX];

static const uint8_t EMU_MAC[6] = {0x6C, 0xF2, 0xD8, 0xD2, 0xA4, 0xA7};

// Realistic Murata Type 2AE / CYW4373 NVRAM identity fallback.
// If duplicate variables exist in NVRAM, Broadcom tools usually use the last one.
// We keep only one active emulator MAC: 6C:F2:D8:D2:A4:A7.
#define CYW4373_MANFID     0x02D0u
#define CYW4373_PRODID     0x4373u
#define CYW4373_SROMREV    11u
#define CYW4373_BOARDREV   0x1301u
#define CYW4373_BOARDNUM   9492u
#define CYW4373_BOARDTYPE  0x083Du
#define CYW4373_DEVID      0x4418u
#define CYW4373_VENDID     0x14E4u
#define CYW4373_NOCRC      1u
#define CYW4373_AA2G       1u
#define CYW4373_AA5G       1u

static const uint8_t CYW4373_MAC_OTP_TUPLE[] = {
    0x80,       // Cypress tuple start
    0x07,       // length = tag + 6 MAC bytes
    0x19,       // tag = macaddr
    0x6C, 0xF2, 0xD8, 0xD2, 0xA4, 0xA7
};

static const char CYW4373_NVRAM_TEXT[] =
    "manfid=0x2d0\n"
    "prodid=0x4373\n"
    "sromrev=11\n"
    "macaddr=6C:F2:D8:D2:A4:A7\n"
    "boardrev=0x1301\n"
    "boardnum=9492\n"
    "boardtype=0x83d\n"
    "customvar1=0x222d0000\n"
    "xtalfreq=37400\n"
    "aa2g=1\n"
    "aa5g=1\n"
    "rxchain=1\n"
    "txchain=1\n"
    "antswitch=0\n"
    "devid=0x4418\n"
    "nocrc=1\n"
    "vendid=0x14e4\n"
    "tssipos2g=1\n"
    "tssipos5g=1\n"
    "extpagain2g=2\n"
    "extpagain5g=2\n"
    "subband5gver=0x4\n"
    "boardflags=0x00000001\n"
    "boardflags2=0x00800000\n"
    "boardflags3=0x48202100\n"
    "pdoffset2g40ma0=15\n"
    "pdoffset2g40ma1=15\n"
    "pdoffset40ma0=15\n"
    "pdoffset40ma1=15\n"
    "rssismf2g=0xf\n"
    "rssismc2g=0x8\n"
    "rssisav2g=0x1\n"
    "pa2gw0a0=0xfe74\n"
    "pa2gw1a0=0x1a14\n"
    "pa2gw2a0=0xfac8\n"
    "maxp2ga0=76\n"
    "maxp5ga0=76\n"
    "swdiv_en=1\n"
    "pa2ga0=-188,5529,-658\n"
    "pa5ga0=-153,5976,-697,-153,5784,-684,-155,5691,-677,-167,5748,-688\n";

static bool dl_started = false;
static bool fw_go_seen = false;
static bool runnable_seen = false;
static uint32_t runnable_seen_ms = 0;
static uint32_t reconnect_at_ms = 0;
static uint32_t dl_go_reconnect_at_ms = 0;
static uint32_t getstate_poll_count = 0;
static uint32_t last_getstate_bytes = 0;
static uint32_t fw_last_log_bytes = 0;


//--------------------------------------------------------------------
// Status LED helpers
//--------------------------------------------------------------------
static void ws2812_delay_cycles(uint32_t cycles) {
    while (cycles--) {
        __asm volatile("nop");
    }
}

static void ws2812_write_pixel(uint8_t r, uint8_t g, uint8_t b) {
#if STATUS_LED_ENABLE
    // WS2812 expects GRB bit order. This simple bit-bang is good enough
    // for status indication on RP2040 at default clock.
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;

    uint32_t irq_state = save_and_disable_interrupts();
    for (int i = 23; i >= 0; --i) {
        bool bit = (grb >> i) & 1u;
        if (bit) {
            gpio_put(STATUS_WS2812_PIN, 1);
            ws2812_delay_cycles(12);
            gpio_put(STATUS_WS2812_PIN, 0);
            ws2812_delay_cycles(5);
        } else {
            gpio_put(STATUS_WS2812_PIN, 1);
            ws2812_delay_cycles(5);
            gpio_put(STATUS_WS2812_PIN, 0);
            ws2812_delay_cycles(12);
        }
    }
    restore_interrupts(irq_state);
    sleep_us(80);
#else
    (void)r; (void)g; (void)b;
#endif
}

static void status_led_rgb(uint8_t r, uint8_t g, uint8_t b) {
#if STATUS_LED_ENABLE
    gpio_put(STATUS_GPIO_LED_PIN, (r || g || b) ? 1 : 0);
    ws2812_write_pixel(r, g, b);
#else
    (void)r; (void)g; (void)b;
#endif
}

static void status_led_off(void)       { status_led_rgb(0, 0, 0); }
static void status_led_wait_power(void){ status_led_rgb(0, 0, 0); }       // waiting WL_REG_ON
static void status_led_power(void)     { status_led_rgb(40, 25, 0); }      // yellow/orange: WL_REG_ON active, stable delay
static void status_led_boot(void)      { status_led_rgb(0, 0, 40); }       // blue: USB boot bd29
static void status_led_dl(void)        { status_led_rgb(30, 0, 40); }      // purple: DL_GETVER/DL_* activity
static void status_led_runtime(void)   { status_led_rgb(0, 40, 0); }       // green: runtime 0bdc
static void status_led_error(void)     { status_led_rgb(40, 0, 0); }       // red: reserved

//--------------------------------------------------------------------
// Helpers
//--------------------------------------------------------------------
static uint16_t min_u16(uint16_t a, uint16_t b) { return a < b ? a : b; }

static void clear_ctrl(void) {
    memset(ctrl_buf, 0, sizeof(ctrl_buf));
}

static void fill_bootrom_id(bootrom_id_t *id, bool postboot_state) {
    memset(id, 0, sizeof(*id));
    id->chip = postboot_state ? BRCMF_POSTBOOT_ID : 0x00004373u;
    id->chiprev = 0;
    id->ramsize = 0x000C0000u;
}

static void fill_rdl_state(rdl_state_t *st, uint32_t state, uint32_t bytes) {
    st->state = state;
    st->bytes = bytes;
}

static const char *dl_name(uint8_t cmd) {
    switch (cmd) {
        case DL_GETSTATE: return "DL_GETSTATE";
        case DL_CHECK_CRC: return "DL_CHECK_CRC";
        case DL_GO: return "DL_GO";
        case DL_START: return "DL_START";
        case DL_REBOOT: return "DL_REBOOT";
        case DL_GO_PROTECTED: return "DL_GO_PROTECTED";
        case DL_EXEC: return "DL_EXEC";
        case DL_GETVER: return "DL_GETVER";
        case DL_RESETCFG: return "DL_RESETCFG";
        default: return "DL_UNKNOWN";
    }
}

static const char *safe_iovar_name(const uint8_t *payload, uint16_t payload_len) {
    if (!payload || payload_len == 0) return NULL;
    for (uint16_t i = 0; i < payload_len; i++) {
        uint8_t c = payload[i];
        if (c == 0) {
            return i > 0 ? (const char *)payload : NULL; // empty string is invalid
        }
        if (c < 0x20 || c > 0x7E) {
            return NULL; // non-printable character -- not an iovar name
        }
    }
    return NULL;
}


static uint16_t copy_u32_payload(uint8_t *payload, uint16_t max_payload, uint32_t value) {
    memset(payload, 0, max_payload);
    if (max_payload >= 4) {
        payload[0] = (uint8_t)(value & 0xFF);
        payload[1] = (uint8_t)((value >> 8) & 0xFF);
        payload[2] = (uint8_t)((value >> 16) & 0xFF);
        payload[3] = (uint8_t)((value >> 24) & 0xFF);
        return 4;
    }
    return max_payload;
}


//--------------------------------------------------------------------
// Common DL control responses
//--------------------------------------------------------------------
static bool handle_dl_control_common(uint8_t rhport, tusb_control_request_t const *request) {
    if ((request->bmRequestType & 0x80) == 0) return false;
    if ((request->bmRequestType & 0x60) != 0x40) return false; // vendor request only

    uint8_t cmd = request->bRequest;

    if (cmd == DL_GETVER) {
        bootrom_id_t *id = (bootrom_id_t *)ctrl_buf;
        memset(id, 0, sizeof(*id));
        id->chip = postboot ? BRCMF_POSTBOOT_ID : CYW4373_CHIP_ID;
        id->chiprev = CYW4373_CHIP_REV;
        id->ramsize = CYW4373_RAM_SIZE;
        return tud_control_xfer(rhport, request, ctrl_buf,
                                min_u16(request->wLength, sizeof(*id)));
    }

    if (cmd == DL_GETSTATE) {
        rdl_state_t *st = (rdl_state_t *)ctrl_buf;
        uint32_t state;

        if (postboot || !boot_mode || fw_go_seen) {
            state = DL_RUNNABLE;
        } else if (!dl_started && fw_bytes_received == 0) {
            state = DL_READY;
        } else if (fw_bytes_received >= CYW4373_EXPECTED_FW_SIZE) {
            state = DL_RUNNABLE;
        } else if (fw_bytes_received > 0) {
            // RUNNABLE if host has been silent for 300 ms (all blocks sent)
            int64_t us_since_bulk = absolute_time_diff_us(last_bulk_time, get_absolute_time());
            if (us_since_bulk > 300000) {
                state = DL_RUNNABLE;
            } else {
                state = DL_READY;
            }
        } else {
            state = DL_READY;
        }

        st->state = state;
        st->bytes = fw_bytes_received;

#if LOG_EVERY_GETSTATE
        printf("%s DL_GETSTATE -> state=%lu bytes=%lu\r\n",
               boot_mode ? "BOOT" : "RUNTIME/POSTBOOT",
               (unsigned long)st->state,
               (unsigned long)st->bytes);
#else
        static uint32_t last_getstate_log_bytes = 0;
        static bool runnable_logged = false;

        if (state == DL_RUNNABLE && !runnable_logged) {
            printf("%s DL_GETSTATE -> RUNNABLE bytes=%lu\r\n",
                   boot_mode ? "BOOT" : "RUNTIME/POSTBOOT",
                   (unsigned long)st->bytes);
            runnable_logged = true;
            last_getstate_log_bytes = st->bytes;
        } else if (boot_mode && st->bytes >= last_getstate_log_bytes + FW_PROGRESS_LOG_STEP) {
            printf("BOOT firmware progress: %lu / %lu bytes\r\n",
                   (unsigned long)st->bytes,
                   (unsigned long)CYW4373_EXPECTED_FW_SIZE);
            last_getstate_log_bytes = st->bytes;
        }
#endif

        return tud_control_xfer(rhport, request, ctrl_buf, sizeof(*st));
    }

    if (cmd == DL_RESETCFG) {
        bootrom_id_t *id = (bootrom_id_t *)ctrl_buf;
        fill_bootrom_id(id, true);
        printf("%s DL_RESETCFG -> bootrom_id 24B ACK\r\n", boot_mode ? "BOOT" : "RUNTIME/POSTBOOT");
        return tud_control_xfer(rhport, request, ctrl_buf, min_u16(request->wLength, sizeof(bootrom_id_t)));
    }

    return false;
}


//--------------------------------------------------------------------
// Stateful iovar helpers
//--------------------------------------------------------------------
static void save_iovar_state(const char *name, const uint8_t *data, uint16_t len) {
    if (!name || !name[0]) return;
    // Try update existing
    for (int i = 0; i < IOVAR_STATE_MAX; i++) {
        if (iovar_state[i].used && strcmp(iovar_state[i].name, name) == 0) {
            uint16_t copy_len = len < IOVAR_DATA_MAX ? len : IOVAR_DATA_MAX;
            memcpy(iovar_state[i].data, data, copy_len);
            iovar_state[i].len = copy_len;
            return;
        }
    }
    // Find free slot
    for (int i = 0; i < IOVAR_STATE_MAX; i++) {
        if (!iovar_state[i].used) {
            strncpy(iovar_state[i].name, name, IOVAR_NAME_MAX - 1);
            iovar_state[i].name[IOVAR_NAME_MAX - 1] = 0;
            uint16_t copy_len = len < IOVAR_DATA_MAX ? len : IOVAR_DATA_MAX;
            memcpy(iovar_state[i].data, data, copy_len);
            iovar_state[i].len = copy_len;
            iovar_state[i].used = true;
            return;
        }
    }
}

static const iovar_state_t *find_iovar_state(const char *name) {
    if (!name || !name[0]) return NULL;
    for (int i = 0; i < IOVAR_STATE_MAX; i++) {
        if (iovar_state[i].used && strcmp(iovar_state[i].name, name) == 0) {
            return &iovar_state[i];
        }
    }
    return NULL;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
// Runtime: extract real command (handles cmd=0 wrapper used by brcmfmac)
//--------------------------------------------------------------------
static uint32_t get_effective_cmd(void) {
    if (last_bcdc_cmd == 0 && last_bcdc_len >= 4) {
        const uint8_t *payload = bcdc_buf + sizeof(bcdc_hdr_t);
        uint32_t embedded = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
        return embedded;
    }
    return last_bcdc_cmd;
}

//--------------------------------------------------------------------
// Open EP1 IN (interrupt) endpoint manually -- not part of TinyUSB
// vendor class descriptors, but must be open for events to reach host.
//--------------------------------------------------------------------
static void open_ep1_in(void) {
    if (ep1_in_opened) return;
    tusb_desc_endpoint_t ep1_desc;
    ep1_desc.bLength          = sizeof(tusb_desc_endpoint_t);
    ep1_desc.bDescriptorType  = TUSB_DESC_ENDPOINT;
    ep1_desc.bEndpointAddress = 0x81;
    ep1_desc.bmAttributes     = 0x03;            // Interrupt endpoint
    ep1_desc.wMaxPacketSize   = 16;             // 16 bytes
    ep1_desc.bInterval        = 9;              // 9 ms polling interval
    bool ok = usbd_edpt_open(0, &ep1_desc);
    if (ok) {
        ep1_in_opened = true;
        printf("EP81 open: OK (via usbd_edpt_open)\r\n");
        return;
    }
    // If usbd_edpt_open failed, endpoint might already be opened by TinyUSB
    // from the configuration descriptor at SET_CONFIGURATION time.
    if (usbd_edpt_busy(0, 0x81)) {
        ep1_in_opened = true;
        printf("EP81 open: OK (already active via descriptor)\r\n");
    } else {
        printf("EP81 open: FAIL (not openable and not active)\r\n");
    }
}

//--------------------------------------------------------------------
// Send heartbeat on EP81 (interrupt IN)
//
// Per brcmfmac/usb.c (Linux kernel):
//   if(wMaxPacketSize == 16) intr_size = 8 (sizeof intr_transfer_buf)
//   else intr_size = 4
//
// struct intr_transfer_buf {
//     u32 notification;   // 4 bytes (often 0)
//     u32 reserved;       // 4 bytes (always 0)
// };
//
// brcmf_usb_intr_complete() does NOT parse content -- it only checks
// urb->status and resubmits. So this is a heartbeat / keep-alive,
// not an event channel.
//--------------------------------------------------------------------
static void send_interrupt_event(void) {
    static uint8_t evt[8];                  // 8 bytes per Linux driver
    static uint32_t mount_time_ms = 0;
    static uint32_t notification_counter = 0;

    if (ep81_first_event) {
        mount_time_ms = to_ms_since_boot(get_absolute_time());
        ep81_first_event = false;
        printf("EP81 first heartbeat queued (will send after 200ms)\r\n");
    }

    // Defer first heartbeat 200 ms after runtime mount
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms < mount_time_ms + 200) {
        return;
    }

    // intr_transfer_buf: notification (u32 LE) + reserved (u32 LE) = 8 bytes
    memset(evt, 0, sizeof(evt));
    notification_counter++;
    evt[0] = (uint8_t)(notification_counter & 0xFF);
    evt[1] = (uint8_t)((notification_counter >> 8) & 0xFF);
    evt[2] = (uint8_t)((notification_counter >> 16) & 0xFF);
    evt[3] = (uint8_t)((notification_counter >> 24) & 0xFF);
    // reserved = 0 (already from memset)

    // Open endpoint if not already
    open_ep1_in();
    if (!ep1_in_opened) {
        printf("EP81 send: endpoint not open\r\n");
        return;
    }

    if (usbd_edpt_busy(0, 0x81)) {
        printf("EP81 send: busy, skipping\r\n");
        return;
    }

    bool xfer_ok = usbd_edpt_xfer(0, 0x81, evt, sizeof(evt));
    printf("EP81 heartbeat #%lu: xfer %s\r\n",
           (unsigned long)notification_counter, xfer_ok ? "OK" : "FAIL");

    // TinyUSB manages endpoint claim/release internally
}

//--------------------------------------------------------------------
// Send full Broadcom event packet on EP82 (bulk IN)
// Includes eth header (14 bytes) + bcm_ethertype (4) + event_msg
//--------------------------------------------------------------------
static void send_bulk_event(uint32_t event_type, uint32_t status, uint32_t reason, uint32_t flags) {
    uint8_t pkt[80];
    memset(pkt, 0, sizeof(pkt));

    // === Ethernet header (14 bytes) ===
    // dst = broadcast
    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = 0xFF; pkt[3] = 0xFF; pkt[4] = 0xFF; pkt[5] = 0xFF;
    // src = EMU_MAC (6 bytes)
    memcpy(pkt + 6, EMU_MAC, 6);
    // ethertype = 0x886C (Broadcom / ETH_P_LINK_CTL)
    pkt[12] = 0x88; pkt[13] = 0x6C;

    // === bcmeth_hdr_t (10 bytes) ===
    // subtype = BCMILCP_SUBTYPE_VENDOR_LONG = 0x8001 (big-endian)
    // required by bcmdhd: short_subtype byte must have bit7 set
    pkt[14] = 0x80; pkt[15] = 0x01;
    // length = 59 = version(1) + oui(3) + usr_subtype(2) + event_msg(48) + payload(5)
    pkt[16] = 0x00; pkt[17] = 0x3B;
    // version = 0 (BCMILCP_BCM_SUBTYPEHDR_VERSION)
    pkt[18] = 0x00;
    // oui = BRCM_OUI = 00:10:18
    pkt[19] = 0x00; pkt[20] = 0x10; pkt[21] = 0x18;
    // usr_subtype = BCMILCP_BCM_SUBTYPE_EVENT = 1
    pkt[22] = 0x00; pkt[23] = 0x01;

    // === wl_event_msg_t / brcmf_event_msg_be (44 bytes, big-endian) ===
    // version = 2
    pkt[24] = 0x00; pkt[25] = 0x02;
    // flags
    pkt[26] = (uint8_t)((flags >> 8) & 0xFF);
    pkt[27] = (uint8_t)(flags & 0xFF);
    // event_type (u32)
    pkt[28] = (uint8_t)((event_type >> 24) & 0xFF);
    pkt[29] = (uint8_t)((event_type >> 16) & 0xFF);
    pkt[30] = (uint8_t)((event_type >>  8) & 0xFF);
    pkt[31] = (uint8_t)(event_type & 0xFF);
    // status (u32)
    pkt[32] = (uint8_t)((status >> 24) & 0xFF);
    pkt[33] = (uint8_t)((status >> 16) & 0xFF);
    pkt[34] = (uint8_t)((status >>  8) & 0xFF);
    pkt[35] = (uint8_t)(status & 0xFF);
    // reason (u32)
    pkt[36] = (uint8_t)((reason >> 24) & 0xFF);
    pkt[37] = (uint8_t)((reason >> 16) & 0xFF);
    pkt[38] = (uint8_t)((reason >>  8) & 0xFF);
    pkt[39] = (uint8_t)(reason & 0xFF);
    // auth_type (u32) = 0
    pkt[40] = 0x00; pkt[41] = 0x00; pkt[42] = 0x00; pkt[43] = 0x00;
    // datalen (u32) = 5 (sizeof wl_event_data_if_t)
    pkt[44] = 0x00; pkt[45] = 0x00; pkt[46] = 0x00; pkt[47] = 0x05;
    // addr[6] = EMU_MAC
    memcpy(pkt + 48, EMU_MAC, 6);
    // ifname[16] = "wl0" + zero padding
    memcpy(pkt + 54, "wl0", 3);
    // ifidx = 0, bsscfgidx = 0 (already zeroed by memset)

    // === wl_event_data_if_t payload (5 bytes) ===
    // Starts at offset 72 = after eth(14) + bcmeth(10) + event_msg(48)
    pkt[72] = 0x00;  // ifidx
    pkt[73] = 0x01;  // opcode / action = BRCMF_E_IF_ADD
    pkt[74] = 0x00;  // reserved / flags
    pkt[75] = 0x00;  // bssidx
    pkt[76] = 0x00;  // role = WLC_E_IF_ROLE_STA

    // Total packet size: 77 bytes (14 + 10 + 48 + 5)
    // Send via bulk IN (EP82). TinyUSB will split 64 + 13 if MPS=64.
    const uint16_t total_pkt = 77;
    uint16_t sent = 0;
    uint16_t retries = 0;
    while (sent < total_pkt && retries < 3) {
        uint16_t n = tud_vendor_write(pkt + sent, total_pkt - sent);
        if (n == 0) {
            tud_vendor_flush();
            retries++;
            continue;
        }
        sent += n;
    }
    tud_vendor_flush();
    printf("EP82 event %lu sent: %u/%u bytes\r\n", event_type, sent, total_pkt);
}

//--------------------------------------------------------------------
// Send BRCMF_E_IF event (interface added) on EP82 bulk IN
// This is the critical async event that host waits for after UP/init
//--------------------------------------------------------------------
static void send_brcmf_if_event(void) {
    send_bulk_event(54, 0, 0, 1);  // WLC_E_IF, status=0, reason=0, flags=1 (UP)
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
// Runtime BCDC/DCMD response
//--------------------------------------------------------------------
static uint16_t prepare_bcdc_response(uint8_t *buf, uint16_t max_len) {
    if (max_len < sizeof(bcdc_hdr_t)) return 0;

    bcdc_hdr_t *hdr = (bcdc_hdr_t *)buf;
    uint32_t effective_cmd = get_effective_cmd();

    hdr->cmd = last_bcdc_cmd;  // preserve original cmd in response header
    hdr->flags = last_bcdc_flags & ~1u; // clear ERROR
    hdr->status = 0;
    hdr->len = 0;

    uint16_t max_payload = max_len - sizeof(bcdc_hdr_t);
    uint8_t *payload = buf + sizeof(bcdc_hdr_t);
    uint16_t payload_len = 0;

    // iovar source: backup (survives ctrl_buf overwrite) or live bcdc_buf
    // FIX: account for cmd=0 wrapper -- skip embedded 4-byte command if present
    uint16_t iovar_off = sizeof(bcdc_hdr_t);
    if (last_bcdc_cmd == 0 &&
        (effective_cmd == BRCMF_C_GET_VAR || effective_cmd == BRCMF_C_SET_VAR)) {
        iovar_off += 4; // skip embedded command
    }
    const char *iovar = last_iovar_valid ? last_iovar_name
                        : (const char *)(bcdc_buf + iovar_off);

    memset(payload, 0, max_payload);

    // ===== IOCTL commands (effective_cmd < 256) =====
    if (effective_cmd < 256) {
        switch (effective_cmd) {
            case BRCMF_C_UP:
                printf("RUNTIME BRCMF_C_UP -> OK\r\n");
                dongle_up = true;
                last_heartbeat_ms = to_ms_since_boot(get_absolute_time());  // reset heartbeat timer
                send_interrupt_event();          // short ping on EP81
                // Defer BRCMF_E_IF event by ~150ms to mimic real firmware timing
                pending_if_event_ms = to_ms_since_boot(get_absolute_time()) + 150;
                break;
            case BRCMF_C_DOWN:
                printf("RUNTIME BRCMF_C_DOWN -> OK\r\n");
                dongle_up = false;
                break;
            case BRCMF_C_GET_REVINFO:
                payload_len = last_bcdc_len ? last_bcdc_len : 68;
                if (payload_len < 68 && max_payload >= 68) payload_len = 68;
                if (payload_len > max_payload) payload_len = max_payload;
                memset(payload, 0, payload_len);
                // brcmf_rev_info layout (at least 68 bytes):
                // chipid at offset 0:  73 43 00 00  (little-endian 0x4373)
                if (payload_len >= 8) {
                    payload[0] = 0x73; payload[1] = 0x43; payload[2] = 0x00; payload[3] = 0x00;
                }
                // vendid at offset 24: E4 14 00 00  (little-endian 0x14E4)
                if (payload_len >= 28) {
                    payload[24] = (uint8_t)(CYW4373_VENDID & 0xFF);
                    payload[25] = (uint8_t)((CYW4373_VENDID >> 8) & 0xFF);
                }
                // devid at offset 28: 18 44 00 00  (little-endian 0x4418)
                if (payload_len >= 32) {
                    payload[28] = (uint8_t)(CYW4373_DEVID & 0xFF);
                    payload[29] = (uint8_t)((CYW4373_DEVID >> 8) & 0xFF);
                }
                // boardtype at offset 44, boardrev at offset 48
                if (payload_len >= 48) {
                    payload[44] = (uint8_t)(CYW4373_BOARDTYPE & 0xFF);
                    payload[45] = (uint8_t)((CYW4373_BOARDTYPE >> 8) & 0xFF);
                }
                if (payload_len >= 52) {
                    payload[48] = (uint8_t)(CYW4373_BOARDREV & 0xFF);
                    payload[49] = (uint8_t)((CYW4373_BOARDREV >> 8) & 0xFF);
                }
                printf("RUNTIME GET_REVINFO -> chip=0x4373 vendid=0x%04X devid=0x%04X boardtype=0x%04X boardrev=0x%04X\r\n",
                       CYW4373_VENDID, CYW4373_DEVID, CYW4373_BOARDTYPE, CYW4373_BOARDREV);
                break;
            case BRCMF_C_GET_PM:
                payload_len = copy_u32_payload(payload, max_payload, current_pm);
                printf("RUNTIME BRCMF_C_GET_PM -> %lu\r\n", (unsigned long)current_pm);
                break;
            case BRCMF_C_SET_PM_COMPAT:
                if (last_bcdc_len >= 4) {
                    const uint8_t *p = bcdc_buf + sizeof(bcdc_hdr_t);
                    current_pm = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
                }
                printf("RUNTIME BRCMF_C_SET_PM -> %lu OK\r\n", (unsigned long)current_pm);
                payload_len = 0;
                break;
            case BRCMF_C_IOCTL_244_COMPAT:
                printf("RUNTIME IOCTL 244 -> OK\r\n");
                payload_len = 0;
                break;
            case BRCMF_C_SET_SCAN_CHANNEL_TIME:
            case BRCMF_C_SET_SCAN_UNASSOC_TIME:
                printf("RUNTIME IOCTL cmd=%lu -> OK\r\n", (unsigned long)effective_cmd);
                break;
            default:
                printf("RUNTIME unknown IOCTL cmd=%lu -> OK/noop\r\n", (unsigned long)effective_cmd);
                break;
        }
        hdr->len = payload_len;
        return sizeof(bcdc_hdr_t) + payload_len;
    }

    // ===== BCDC commands (effective_cmd >= 256) =====
    if (effective_cmd == BRCMF_C_GET_VAR) {
        if (strncmp(iovar, "cur_etheraddr", 20) == 0) {
            payload_len = max_payload >= 8 ? 8 : max_payload;
            memcpy(payload, EMU_MAC, payload_len >= 6 ? 6 : payload_len);

        } else if (strncmp(iovar, "ver", 20) == 0 || strncmp(iovar, "version", 20) == 0) {
            const char *ver = CYW4373_FW_VERSION_STRING "\n";
            payload_len = (uint16_t)(strlen(ver) + 1);
            if (payload_len > max_payload) payload_len = max_payload;
            memcpy(payload, ver, payload_len);

        } else if (strncmp(iovar, "clmver", 20) == 0) {
            const char *clm = CYW4373_CLM_VERSION_STRING;
            payload_len = (uint16_t)(strlen(clm) + 1);
            if (payload_len > max_payload) payload_len = max_payload;
            memcpy(payload, clm, payload_len);

        } else if (strncmp(iovar, "event_msgs", 20) == 0 || strncmp(iovar, "event_msgs_ext", 20) == 0) {
            payload_len = 18;  // BRCMF_EVENTING_MASK_LEN (18 bytes for CYW4373)
            if (payload_len > max_payload) payload_len = max_payload;
            memset(payload, CYW4373_EVENT_MSGS_FILL, payload_len);

        } else if (strncmp(iovar, "country", 20) == 0) {
            payload_len = max_payload >= current_country_len ? current_country_len : max_payload;
            memcpy(payload, current_country, payload_len);

        } else if (strncmp(iovar, "mpc", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 1);

        // ===== Power management / offload =====
        } else if (strncmp(iovar, "pm", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, current_pm);

        } else if (strncmp(iovar, "bcn_timeout", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "assoc_listen", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "dtim_assoc", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "dtim_prog", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "pspretend_threshold", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "lpas", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "arpoe", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "arp_ol", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "ndoe", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "nd_hostip", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "manfid", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_MANFID);

        } else if (strncmp(iovar, "prodid", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_PRODID);

        } else if (strncmp(iovar, "sromrev", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_SROMREV);

        } else if (strncmp(iovar, "boardrev", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_BOARDREV);

        } else if (strncmp(iovar, "boardnum", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_BOARDNUM);

        } else if (strncmp(iovar, "boardtype", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_BOARDTYPE);

        } else if (strncmp(iovar, "boardflags", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0x00000001u);

        } else if (strncmp(iovar, "boardflags2", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0x00800000u);

        } else if (strncmp(iovar, "boardflags3", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0x48202100u);

        } else if (strncmp(iovar, "devid", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_DEVID);

        } else if (strncmp(iovar, "vendid", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_VENDID);

        } else if (strncmp(iovar, "macaddr", 20) == 0) {
            payload_len = max_payload >= 8 ? 8 : max_payload;
            memcpy(payload, EMU_MAC, payload_len >= 6 ? 6 : payload_len);

        } else if (strncmp(iovar, "vars", 20) == 0 ||
                   strncmp(iovar, "nvram", 20) == 0 ||
                   strncmp(iovar, "nvram_dump", 20) == 0 ||
                   strncmp(iovar, "cisdump", 20) == 0 ||
                   strncmp(iovar, "otpdump", 20) == 0) {
            payload_len = (uint16_t)(strlen(CYW4373_NVRAM_TEXT) + 1);
            if (payload_len > max_payload) payload_len = max_payload;
            memcpy(payload, CYW4373_NVRAM_TEXT, payload_len);

        } else if (strncmp(iovar, "nocrc", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_NOCRC);

        } else if (strncmp(iovar, "aa2g", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_AA2G);

        } else if (strncmp(iovar, "aa5g", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_AA5G);

        } else if (strncmp(iovar, "mac_otp_tuple", 20) == 0 ||
                   strncmp(iovar, "otp_mac_tuple", 20) == 0) {
            payload_len = sizeof(CYW4373_MAC_OTP_TUPLE);
            if (payload_len > max_payload) payload_len = max_payload;
            memcpy(payload, CYW4373_MAC_OTP_TUPLE, payload_len);

        } else if (strncmp(iovar, "mimo_bw_cap", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "ampdu_ba_wsize", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 64);

        // ===== WiFi association state (required for "ready but not connected") =====
        } else if (strncmp(iovar, "ssid", 20) == 0) {
            // ssid structure: uint32_t len + uint8_t SSID[32] = 36 bytes
            // Empty = not associated
            payload_len = max_payload >= 36 ? 36 : max_payload;
            memset(payload, 0, payload_len);  // len=0, SSID=""

        } else if (strncmp(iovar, "bssid", 20) == 0) {
            payload_len = max_payload >= 6 ? 6 : max_payload;
            memset(payload, 0, payload_len);  // 00:00:00:00:00:00

        } else if (strncmp(iovar, "status", 20) == 0) {
            // WiFi disabled: report NOT_ASSOCIATED (0)
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "wpa_auth", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "wsec", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "infra", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 1);  // infrastructure

        } else if (strncmp(iovar, "mfp", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "apsta", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);  // STA only, no AP concurrent

        } else if (strncmp(iovar, "wet", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "sup_wpa", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 1);

        } else if (strncmp(iovar, "wme", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 1);

        } else if (strncmp(iovar, "wme_apsd", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "maxassoc", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "obss_coex", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "amsdu", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 1);

        } else if (strncmp(iovar, "ampdu", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 1);

        } else if (strncmp(iovar, "join_pref", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "reassoc", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "actframe", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);

        } else if (strncmp(iovar, "bss_max", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 1);

        } else if (strncmp(iovar, "escanresults", 20) == 0 ||
                   strncmp(iovar, "scanresults", 20) == 0) {
            // Return minimal escan result: 1 network "Canon" with WPA2
            // Structure: escan_result header + bss_info
            #define FAKE_SSID "CanonWiFi"
            #define FAKE_BSSID {0x00, 0x11, 0x22, 0x33, 0x44, 0x55}
            uint16_t ssid_len = (uint16_t)strlen(FAKE_SSID);
            uint16_t fixed_len = 4 + 6 + 6 + 2 + 8 + 2 + 2 + 2 + 2 + 4 + 4 + 4 + 4 + 2 + 2 + 2 + 2 + 2 + 2 + 2 + 2 + 4 + 4 + 4 + 1 + ssid_len;
            payload_len = fixed_len < max_payload ? fixed_len : max_payload;
            memset(payload, 0, payload_len);
            if (payload_len >= 4) {
                payload[0] = 0x01;  // 1 result
            }
            if (payload_len >= 10) {
                uint8_t bssid[] = FAKE_BSSID;
                memcpy(payload + 4, bssid, 6);
            }
            if (payload_len >= 22) {
                payload[20] = 0x07;  // ch 7
                payload[21] = 0x00;
            }
            if (payload_len >= 32) {
                payload[28] = 0x6C;  // RSSI -80
                payload[29] = 0xFF;
            }
            if (payload_len >= 56 && ssid_len > 0) {
                payload[55] = ssid_len;
                memcpy(payload + 56, FAKE_SSID, ssid_len);
            }
            printf("RUNTIME answer %s with 1 fake network '%s'\r\n", iovar, FAKE_SSID);

        } else if (strncmp(iovar, "cap", 20) == 0 ||
                   strncmp(iovar, "chanspecs", 20) == 0) {
            // Known-safe variables: check if host SET them before, else return zeros
            const iovar_state_t *saved = find_iovar_state(iovar);
            if (saved) {
                payload_len = saved->len < max_payload ? saved->len : max_payload;
                memcpy(payload, saved->data, payload_len);
            } else {
                payload_len = last_bcdc_len ? last_bcdc_len : 4;
                if (payload_len > max_payload) payload_len = max_payload;
                memset(payload, 0, payload_len);
            }

        } else {
            // Truly unknown GET_VAR: log for diagnostics, still return zero to not break init
            printf("RUNTIME unknown GET_VAR '%s' -> TODO (zero for compat)\r\n",
                   safe_iovar_name((uint8_t*)iovar, 128) ? iovar : "?");
            payload_len = 0;
        }

    } else if (effective_cmd == BRCMF_C_SET_VAR) {
        const char *set_name = last_iovar_valid ? last_iovar_name : "";
        printf("RUNTIME SET_VAR '%s' -> OK\r\n", set_name);
        // Save stateful value if payload exists after name\0
        if (last_iovar_valid && bcdc_buf && last_bcdc_len > 0) {
            uint16_t name_len = (uint16_t)(strlen(set_name) + 1);
            if (last_bcdc_len > name_len) {
                const uint8_t *val_ptr = bcdc_buf + sizeof(bcdc_hdr_t) + name_len;
                uint16_t val_len = last_bcdc_len - name_len;
                if (val_len > 0) {
                    save_iovar_state(set_name, val_ptr, val_len);
                    // Special case: country
                    if (strncmp(set_name, "country", 20) == 0 && val_len >= 4) {
                        memcpy(current_country, val_ptr, 4);
                        current_country[0] = val_ptr[0];
                        current_country[1] = val_ptr[1];
                        current_country[2] = 0;
                        current_country[3] = 0;
                        if (val_len >= 12) {
                            memcpy(current_country + 8, val_ptr + 8, 4);
                        }
                    }
                }
            }
        }
        payload_len = 0;
        // After scan/escan request, send scan complete event (no networks found)
        if (last_iovar_valid && (strncmp(set_name, "escan", 20) == 0 ||
                                  strncmp(set_name, "scan", 20) == 0)) {
            send_interrupt_event();                 // ping on EP81
            send_bulk_event(69, 3, 0, 0);           // WLC_E_ESCAN_RESULT, status=NO_NETWORKS(3)
            send_bulk_event(52, 3, 0, 0);           // WLC_E_SCAN_COMPLETE, status=NO_NETWORKS(3)
        }

    } else if (effective_cmd == BRCMF_C_UP || effective_cmd == BRCMF_C_DOWN ||
               effective_cmd == BRCMF_C_SET_SCAN_CHANNEL_TIME ||
               effective_cmd == BRCMF_C_SET_SCAN_UNASSOC_TIME) {
        printf("RUNTIME command %lu -> OK\r\n", (unsigned long)effective_cmd);
        payload_len = 0;

    } else {
        printf("RUNTIME unknown cmd=%lu -> OK/noop\r\n", (unsigned long)effective_cmd);
        payload_len = 0;
    }

    hdr->len = payload_len;
    uint16_t resp_total = sizeof(bcdc_hdr_t) + payload_len;
    // Final-stage diagnostic log (seq, cmd, iovar, sizes, flags)
    const char *log_name = last_iovar_valid ? last_iovar_name : "";
    if (effective_cmd >= 256) {
        printf("RUNTIME IN seq=%lu cmd=%lu iovar='%s' payload=%u resp=%u flags=0x%lX status=%u\r\n",
               (unsigned long)runtime_out_seq,
               (unsigned long)effective_cmd,
               log_name,
               payload_len,
               resp_total,
               (unsigned long)(last_bcdc_flags & ~1u),
               0U);
    } else {
        printf("RUNTIME IN seq=%lu ioctl=%lu payload=%u resp=%u flags=0x%lX status=%u\r\n",
               (unsigned long)runtime_out_seq,
               (unsigned long)effective_cmd,
               payload_len,
               resp_total,
               (unsigned long)(last_bcdc_flags & ~1u),
               0U);
    }
    return resp_total;
}

static bool is_supported_bcdc_cmd(uint32_t cmd) {
    return cmd == BRCMF_C_GET_VAR ||
           cmd == BRCMF_C_SET_VAR ||
           cmd == BRCMF_C_GET_REVINFO ||
           cmd == BRCMF_C_UP ||
           cmd == BRCMF_C_DOWN ||
           cmd == BRCMF_C_GET_PM ||
           cmd == BRCMF_C_SET_PM_COMPAT ||
           cmd == BRCMF_C_IOCTL_244_COMPAT ||
           cmd == BRCMF_C_SET_SCAN_CHANNEL_TIME ||
           cmd == BRCMF_C_SET_SCAN_UNASSOC_TIME;
}

static bool bcdc_header_all_zero(const bcdc_hdr_t *hdr) {
    return hdr->cmd == 0 && hdr->len == 0 && hdr->flags == 0 && hdr->status == 0;
}

static bool parse_runtime_out(uint16_t total) {
    if (total > sizeof(ctrl_buf)) total = sizeof(ctrl_buf);

    // Hex dump disabled: 64 bytes at 115200 baud takes ~17ms, stalls EP0
    // Enable only for deep debug: #define PARSE_HEX_DUMP 1
#if 0
    printf("parse_out: total=%u", total);
    uint16_t dump_len = total < 64 ? total : 64;
    for (uint16_t i = 0; i < dump_len; i++) {
        printf(" %02X", ctrl_buf[i]);
    }
    printf("\r\n");
#endif

    if (total < sizeof(bcdc_hdr_t)) {
        return false;
    }

    bcdc_hdr_t *hdr = (bcdc_hdr_t *)ctrl_buf;

    if (bcdc_header_all_zero(hdr)) {
        return false;
    }

    if (hdr->cmd != 0 && !is_supported_bcdc_cmd(hdr->cmd)) {
        // Keep unknown non-zero IOCTL commands instead of falling back to cmd=0.
        // prepare_bcdc_response() will ACK/noop them, which is safer during init.
    }

    last_bcdc_cmd = hdr->cmd;
    last_bcdc_len = hdr->len & 0xFFFFu;
    if (last_bcdc_len > total - sizeof(bcdc_hdr_t)) {
        last_bcdc_len = total - sizeof(bcdc_hdr_t);
    }
    last_bcdc_flags = hdr->flags;

    memset(bcdc_buf, 0, sizeof(bcdc_buf));
    memcpy(bcdc_buf, ctrl_buf, total);

    // ===== FIX: calculate iovar offset for cmd=0 wrapper =====
    uint16_t iovar_off = sizeof(bcdc_hdr_t);
    uint32_t effective_now = last_bcdc_cmd;
    if (last_bcdc_cmd == 0 && last_bcdc_len >= 4) {
        const uint8_t *p = bcdc_buf + sizeof(bcdc_hdr_t);
        effective_now = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
        if (effective_now == BRCMF_C_GET_VAR || effective_now == BRCMF_C_SET_VAR) {
            iovar_off += 4; // skip embedded command bytes
        }
    }

    // Save iovar name from correct offset
    const char *name = NULL;
    if (total > iovar_off) {
        name = safe_iovar_name(bcdc_buf + iovar_off, total - iovar_off);
    }
    if (name) {
        strncpy(last_iovar_name, name, sizeof(last_iovar_name) - 1);
        last_iovar_name[sizeof(last_iovar_name) - 1] = 0;
        last_iovar_valid = true;
    } else {
        last_iovar_valid = false;
    }

    if ((effective_now == BRCMF_C_GET_VAR || effective_now == BRCMF_C_SET_VAR) && !name) {
        return false;
    }

    if (effective_now == BRCMF_C_GET_VAR) {
        printf("RUNTIME BCDC GET_VAR %s len=%lu flags=0x%08lX\r\n",
               name ? name : "?",
               (unsigned long)last_bcdc_len,
               (unsigned long)last_bcdc_flags);
    } else if (effective_now == BRCMF_C_SET_VAR) {
        printf("RUNTIME BCDC SET_VAR %s len=%lu flags=0x%08lX\r\n",
               name ? name : "?",
               (unsigned long)last_bcdc_len,
               (unsigned long)last_bcdc_flags);
    } else {
        printf("RUNTIME BCDC cmd=%lu effective=%lu len=%lu flags=0x%08lX\r\n",
               (unsigned long)last_bcdc_cmd,
               (unsigned long)effective_now,
               (unsigned long)last_bcdc_len,
               (unsigned long)last_bcdc_flags);
    }

    return true;
}
//--------------------------------------------------------------------
// Control callback
//--------------------------------------------------------------------
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
    // Runtime: process DATA/ACK after OUT transfer (must be before early return).
    // Do not clear pending unless the BCDC header/iovar really parsed; ACK can be
    // the second chance if DATA is still empty/early on a given TinyUSB build.
    if (!boot_mode &&
        (stage == CONTROL_STAGE_DATA || stage == CONTROL_STAGE_ACK) &&
        runtime_out_pending &&
        request->bmRequestType == 0x21 &&
        request->bRequest == 0) {

        bool ok = parse_runtime_out(runtime_out_len);
        if (ok) {
            runtime_out_pending = false;
        }
        return true;
    }

    if (stage != CONTROL_STAGE_SETUP) return true;

    // Canon/brcmfmac can ask DL_GETVER/DL_GETSTATE again after runtime reconnect.
    // Common handler only for runtime/postboot; boot mode must use the tolerant
    // boot-specific block below (fallback RUNNABLE, DL_RESETCFG reconnect, etc.)
    if (!boot_mode && handle_dl_control_common(rhport, request)) {
        return true;
    }

    if (boot_mode) {
        if (stage == CONTROL_STAGE_SETUP) {
            if (request->bmRequestType == 0xC1) {
                uint8_t cmd = request->bRequest;
                uint16_t len = request->wLength;
                clear_ctrl();

                // Minimal boot logging: only key commands to avoid UART stall on EP0
                // (brcmfmac times out if control transfer delayed by slow 115200 UART)
                if (cmd == DL_START || cmd == DL_GO || cmd == DL_RESETCFG || cmd == DL_REBOOT) {
                    printf("BOOT %s bReq=%u fw_bytes=%lu\r\n",
                           dl_name(cmd), cmd, (unsigned long)fw_bytes_received);
                }

                if (cmd == DL_GETVER) {
                    status_led_dl();
                    // FAST DL_GETVER: returns chip id 0x4373 or postboot 0xA123.
                    bootrom_id_t *id = (bootrom_id_t *)ctrl_buf;
                    fill_bootrom_id(id, postboot);
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(bootrom_id_t)));
                }

                if (cmd == DL_START) {
                    status_led_dl();
                    fw_bytes_received = 0;
                    fw_last_log_bytes = 0;
                    dl_state = DL_WAITING;
                    postboot = false;
                    dl_started = true;
                    fw_go_seen = false;
                    getstate_poll_count = 0;
                    last_getstate_bytes = 0;
                    runnable_seen = false;
                    runnable_seen_ms = 0;
                    dl_go_reconnect_at_ms = 0;

                    rdl_state_t *st = (rdl_state_t *)ctrl_buf;
                    fill_rdl_state(st, DL_WAITING, 0);
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(rdl_state_t)));
                }

                if (cmd == DL_GETSTATE) {
                    uint32_t state = DL_READY;

                    if (fw_go_seen) {
                        state = DL_RUNNABLE;
                    } else if (!dl_started && fw_bytes_received == 0) {
                        state = DL_READY;
                    } else if (dl_started) {
                        if (fw_bytes_received >= CYW4373_EXPECTED_FW_SIZE) {
                            state = DL_RUNNABLE;
                        } else if (fw_bytes_received > 0) {
                            // RUNNABLE if host has been silent for 300 ms (all blocks sent)
                            int64_t us_since_bulk = absolute_time_diff_us(last_bulk_time, get_absolute_time());
                            if (us_since_bulk > 300000) {
                                state = DL_RUNNABLE;
                            } else {
                                state = DL_READY;
                            }
                        } else {
                            state = DL_READY;
                        }
                    }

                    if (state == DL_RUNNABLE && !runnable_seen) {
                        runnable_seen = true;
                        runnable_seen_ms = to_ms_since_boot(get_absolute_time());
                        printf("BOOT RUNNABLE seen at %lu ms\r\n", (unsigned long)runnable_seen_ms);
                    }

                    // Minimal logging: only key milestones to avoid UART saturation at 115200
                    static uint32_t last_getstate_log_bytes = 0;
                    if (state == DL_RUNNABLE) {
                        printf("BOOT DL_GETSTATE -> RUNNABLE bytes=%lu\r\n", (unsigned long)fw_bytes_received);
                        last_getstate_log_bytes = fw_bytes_received;
                    } else if (fw_bytes_received >= last_getstate_log_bytes + FW_PROGRESS_LOG_STEP) {
                        printf("BOOT progress: %lu / %lu bytes\r\n",
                               (unsigned long)fw_bytes_received,
                               (unsigned long)CYW4373_EXPECTED_FW_SIZE);
                        last_getstate_log_bytes = fw_bytes_received;
                    }

                    rdl_state_t *st = (rdl_state_t *)ctrl_buf;
                    fill_rdl_state(st, state, fw_bytes_received);
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(rdl_state_t)));
                }

                if (cmd == DL_GO) {
                    postboot = true;
                    fw_go_seen = true;
                    dl_state = DL_RUNNABLE;
                    uint32_t now = to_ms_since_boot(get_absolute_time());
                    dl_go_reconnect_at_ms = now + CYW4373_DL_GO_RECONNECT_DELAY_MS;
                    printf("BOOT DL_GO received\r\n");

                    rdl_state_t *st = (rdl_state_t *)ctrl_buf;
                    fill_rdl_state(st, DL_RUNNABLE, fw_bytes_received);
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(rdl_state_t)));
                }

                if (cmd == DL_CHECK_CRC) {
                    // Usually unused by brcmfmac. Return zero-success rdl_state.
                    rdl_state_t *st = (rdl_state_t *)ctrl_buf;
                    uint32_t state = (fw_bytes_received >= CYW4373_EXPECTED_FW_SIZE) ? DL_RUNNABLE :
                                     (fw_bytes_received > 0 ? DL_READY : DL_WAITING);
                    fill_rdl_state(st, state, fw_bytes_received);
                    printf("BOOT DL_CHECK_CRC -> state=%lu bytes=%lu\r\n",
                           (unsigned long)state, (unsigned long)fw_bytes_received);
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(rdl_state_t)));
                }

                if (cmd == DL_GO_PROTECTED) {
                    postboot = true;
                    fw_go_seen = true;
                    dl_state = DL_RUNNABLE;
                    uint32_t now = to_ms_since_boot(get_absolute_time());
                    dl_go_reconnect_at_ms = now + CYW4373_DL_GO_RECONNECT_DELAY_MS;
                    rdl_state_t *st = (rdl_state_t *)ctrl_buf;
                    fill_rdl_state(st, DL_RUNNABLE, fw_bytes_received);
                    printf("BOOT DL_GO_PROTECTED received, reconnect planned\r\n");
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(rdl_state_t)));
                }

                if (cmd == DL_EXEC) {
                    // Normally not used for CYW4373 firmware path.
                    rdl_state_t *st = (rdl_state_t *)ctrl_buf;
                    fill_rdl_state(st, dl_state, fw_bytes_received);
                    printf("BOOT DL_EXEC ignored safely -> state=%lu bytes=%lu\r\n",
                           (unsigned long)dl_state, (unsigned long)fw_bytes_received);
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(rdl_state_t)));
                }

                if (cmd == DL_RESETCFG) {
                    needs_reconnect = true;
                    dl_go_reconnect_at_ms = 0;
                    bootrom_id_t *id = (bootrom_id_t *)ctrl_buf;
                    fill_bootrom_id(id, true);
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(bootrom_id_t)));
                }

                if (cmd == DL_REBOOT) {
                    needs_reconnect = true;
                    dl_go_reconnect_at_ms = 0;
                    rdl_state_t *st = (rdl_state_t *)ctrl_buf;
                    fill_rdl_state(st, DL_RUNNABLE, fw_bytes_received);
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(rdl_state_t)));
                }

                return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(ctrl_buf)));
            }
            return false;
        }
        return true;
    }

    // Runtime mode
    if (!boot_mode) {
        if (stage == CONTROL_STAGE_SETUP) {
            if (request->bmRequestType == 0x21 && request->bRequest == 0) {
                uint16_t rx_len = min_u16(request->wLength, sizeof(ctrl_buf));
                memset(ctrl_buf, 0, sizeof(ctrl_buf));
                memset(bcdc_buf, 0, sizeof(bcdc_buf));
                runtime_out_seq++;
                runtime_out_pending = true;
                runtime_out_len = rx_len;
                last_bcdc_cmd = 0;
                last_bcdc_len = 0;
                last_bcdc_flags = 0;
                last_iovar_valid = false;
                return tud_control_xfer(rhport, request, ctrl_buf, rx_len);
            }

            if (request->bmRequestType == 0xA1 && request->bRequest == 1) {
                uint16_t tx_len = min_u16(request->wLength, sizeof(ctrl_buf));
                if (runtime_out_pending) {
                    bool ok = parse_runtime_out(runtime_out_len);
                    if (ok) {
                        runtime_out_pending = false;
                    }
                }
                clear_ctrl();
                uint16_t resp_len = prepare_bcdc_response(ctrl_buf, tx_len);
                if (resp_len > tx_len) resp_len = tx_len;
                // Pad remainder with zeros so host gets full wLength (Canon expects full buffer)
                if (resp_len < tx_len) {
                    memset(ctrl_buf + resp_len, 0, tx_len - resp_len);
                }
                return tud_control_xfer(rhport, request, ctrl_buf, tx_len);
            }

            printf("RUNTIME unhandled setup bm=0x%02X bReq=0x%02X len=%u\r\n",
                   request->bmRequestType, request->bRequest, request->wLength);
            return false;
        }

        return true;
    }

    return false;
}

//--------------------------------------------------------------------
// Bulk OUT polling: firmware download
//
// Polling is used instead of tud_vendor_rx_cb(), because TinyUSB vendor
// RX callback signatures differ across Pico SDK / TinyUSB versions.
//--------------------------------------------------------------------
static void drain_vendor_out(void) {
    uint8_t tmp[512];

    if (!tud_vendor_available()) return;

#if CYW4373_ASSUME_DL_START_ON_BULK
    if (!dl_started && boot_mode) {
        dl_started = true;
        getstate_poll_count = 0;
        last_getstate_bytes = fw_bytes_received;
        printf("BOOT bulk arrived before DL_START; assuming download started\r\n");
    }
#endif

    while (tud_vendor_available()) {
        uint32_t count = tud_vendor_read(tmp, sizeof(tmp));
        if (count == 0) break;

        if (boot_mode) {
            fw_bytes_received += count;
            last_bulk_time = get_absolute_time();
            dl_state = DL_READY;
        }
    }

    if (!boot_mode) return;

    if (fw_bytes_received >= CYW4373_EXPECTED_FW_SIZE && fw_last_log_bytes < CYW4373_EXPECTED_FW_SIZE) {
        printf("BOOT expected firmware size reached: %lu bytes\r\n", (unsigned long)fw_bytes_received);
        fw_last_log_bytes = fw_bytes_received;
    }

    if (fw_bytes_received > 0 && (fw_bytes_received - fw_last_log_bytes) >= FW_PROGRESS_LOG_STEP) {
        printf("BOOT bulk progress=%lu\r\n", (unsigned long)fw_bytes_received);
        fw_last_log_bytes = fw_bytes_received;
    }
}

//--------------------------------------------------------------------
// USB mount callbacks
//--------------------------------------------------------------------
void tud_mount_cb(void) {
    printf("USB mounted as %s\r\n", boot_mode ? "boot 04b4:bd29" : "runtime 04b4:0bdc");
    if (boot_mode) {
        status_led_boot();
    } else {
        status_led_runtime();
        open_ep1_in();
        send_interrupt_event();
    }
}
void tud_umount_cb(void) {
    printf("USB unmounted\r\n");
    ep1_in_opened = false;
}
void tud_suspend_cb(bool remote_wakeup_en) {
    printf("USB suspend (remote_wakeup_en=%d)\r\n", remote_wakeup_en);
    status_led_off();
}
void tud_resume_cb(void) {
    printf("USB resume\r\n");
    if (!boot_mode) {
        status_led_runtime();
        send_interrupt_event();
        tud_remote_wakeup();
    } else {
        status_led_boot();
    }
}

//--------------------------------------------------------------------
// Reconnect
//--------------------------------------------------------------------
static void switch_to_runtime_reconnect(void) {
    if (!boot_mode) return;

    printf("SWITCH to runtime mode 04b4:0bdc\r\n");
    tud_disconnect();
    pending_if_event_ms = 0;  // cancel any deferred event from previous session
    for (int i = 0; i < 50; i++) {
        tud_task();
        sleep_ms(10);
    }
    boot_mode = false;
    postboot = true;
    dongle_up = false;
    pending_if_event_ms = 0;
    ep81_first_event = true;
    runtime_out_pending = false;
    runtime_out_len = 0;
    status_led_runtime();
    for (int i = 0; i < 10; i++) {
        tud_task();
        sleep_ms(10);
    }
    tud_connect();
    for (int i = 0; i < 20; i++) {
        tud_task();
        sleep_ms(10);
    }
    printf("Runtime connect requested\r\n");
}

//--------------------------------------------------------------------
// Main
//--------------------------------------------------------------------
int main(void) {
    board_init();

    stdio_uart_init_full(uart0, 115200, 0, 1);
    sleep_ms(100);

    gpio_init(WL_REG_ON_PIN);
    gpio_set_dir(WL_REG_ON_PIN, GPIO_IN);
    gpio_pull_down(WL_REG_ON_PIN);

#if STATUS_LED_ENABLE
    gpio_init(STATUS_WS2812_PIN);
    gpio_set_dir(STATUS_WS2812_PIN, GPIO_OUT);
    gpio_put(STATUS_WS2812_PIN, 0);

    gpio_init(STATUS_GPIO_LED_PIN);
    gpio_set_dir(STATUS_GPIO_LED_PIN, GPIO_OUT);
    gpio_put(STATUS_GPIO_LED_PIN, 0);
    status_led_wait_power();
#endif

    printf("\r\n=== CYW4373 Emulator v4.23 BCDC event frame fix ===\r\n");
    printf("UART0 TX=GPIO0 RX=GPIO1 baud=115200\r\n");
    printf("LED indicator: WS2812 GPIO16, external LED GPIO25\r\n");
    printf("Bulk OUT receive mode: polling, no TinyUSB RX callback dependency\r\n");
    printf("v4.19 fixed base + minimal runtime BCDC DATA/ACK parser patch\r\n");
    printf("WL_REG_ON delay = %lu ms\r\n", (unsigned long)POWER_STABLE_DELAY_MS);
    printf("USB boot 04b4:bd29, runtime 04b4:0bdc\r\n");
    printf("bcdDevice=0x0001, boot product string includes trailing 0x01\r\n");
    printf("Descriptor endpoint order: EP81 interrupt first, then EP82/EP02 bulk\r\n");
    printf("RP2040-safe descriptors: bulk MPS=64, bMaxPower=0x32, EP81 interval=9\r\n");
    printf("NVRAM/OTP fallback variables enabled: manfid/prodid/boardtype/devid/macaddr\r\n");
    printf("bDeviceClass=0xFF, interface FF/02/FF\r\n");
    printf("EP81 interrupt IN, EP82 bulk IN, EP02 bulk OUT\r\n");
    printf("Waiting for WL_REG_ON / Power Control on GPIO%d HIGH...\r\n", WL_REG_ON_PIN);

    uint32_t wait_start = to_ms_since_boot(get_absolute_time());
    while (!gpio_get(WL_REG_ON_PIN)) {
        // TinyUSB is not initialized yet here; do not call tud_task().
        sleep_ms(10);
#if WL_REG_ON_WAIT_TIMEOUT_MS > 0
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - wait_start) > WL_REG_ON_WAIT_TIMEOUT_MS) {
            printf("WARN: WL_REG_ON timeout, starting anyway\r\n");
            break;
        }
#else
        (void)wait_start;
#endif
    }

    printf("WL_REG_ON active; stable wait %lu ms\r\n", (unsigned long)POWER_STABLE_DELAY_MS);
    status_led_power();
    sleep_ms(POWER_STABLE_DELAY_MS);
    printf("Expected FW size: %lu bytes\r\n", (unsigned long)CYW4373_EXPECTED_FW_SIZE);
    printf("Final checked build: fast DL_GETSTATE/DL_GETVER, no tud_task before tusb_init\r\n");
    printf("IOCTL_RESP_TIMEOUT target: reply well below 2000 ms\r\n");
    printf("Real NVRAM dump + MAC OTP tuple fallback enabled\r\n");
    printf("DL_GETSTATE=8 bytes state+bytes, DL_GETVER=24 bytes chip id\r\n");
    printf("Auto runtime fallback: %s, delay=%lu ms\r\n",
           CYW4373_AUTO_RUNTIME_FALLBACK ? "ON" : "OFF",
           (unsigned long)CYW4373_AUTO_RUNTIME_FALLBACK_DELAY_MS);

    tusb_init();
    last_bulk_time = get_absolute_time();
    status_led_boot();
    printf("TinyUSB started\r\n");

    // Explicitly enable USB device pull-up/connect.
    // Some TinyUSB/Pico SDK builds auto-connect at tusb_init(), but this makes it unambiguous.
    tud_connect();
    printf("USB connect requested\r\n");

    while (1) {
        tud_task();
        drain_vendor_out();

        // Runtime heartbeat: ping EP81 every 200 ms to keep host aware we are alive
        if (!boot_mode && dongle_up) {
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if ((now_ms - last_heartbeat_ms) >= 200) {
                last_heartbeat_ms = now_ms;
                send_interrupt_event();  // short 16-byte ping
            }
        }

        // Deferred BRCMF_E_IF event delivery (mimics real firmware async timing)
        if (pending_if_event_ms != 0) {
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if ((int32_t)(now_ms - pending_if_event_ms) >= 0) {
                pending_if_event_ms = 0;
                printf("Deferred BRCMF_E_IF -> send_bulk_event\r\n");
                send_brcmf_if_event();
            }
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (needs_reconnect) {
            needs_reconnect = false;
            dl_go_reconnect_at_ms = 0;
            switch_to_runtime_reconnect();
            continue;
        }

        if (boot_mode && dl_go_reconnect_at_ms != 0) {
            if ((int32_t)(now - dl_go_reconnect_at_ms) >= 0) {
                dl_go_reconnect_at_ms = 0;
                printf("AUTO reconnect after DL_GO, no DL_RESETCFG\r\n");
                switch_to_runtime_reconnect();
                continue;
            }
        }

#if CYW4373_AUTO_RUNTIME_FALLBACK
        if (boot_mode && reconnect_at_ms == 0 && runnable_seen && !fw_go_seen) {
            if ((int32_t)(now - (runnable_seen_ms + CYW4373_AUTO_RUNTIME_FALLBACK_DELAY_MS)) >= 0) {
                reconnect_at_ms = now;
                printf("AUTO runtime fallback: no DL_GO after RUNNABLE\r\n");
            }
        }
#endif

        if (boot_mode && reconnect_at_ms != 0) {
            if ((int32_t)(now - reconnect_at_ms) >= 0) {
                reconnect_at_ms = 0;
                switch_to_runtime_reconnect();
                continue;
            }
        }

#if CYW4373_AUTO_RUNTIME_FALLBACK
        // If bulk stopped, mark runnable so host can proceed on next GETSTATE.
        if (boot_mode && dl_state == DL_READY && fw_bytes_received > 0) {
            int64_t us_since_bulk = absolute_time_diff_us(last_bulk_time, get_absolute_time());
            if (us_since_bulk > 50000) {
                dl_state = DL_RUNNABLE;
            }
        }
#endif
    }

    return 0;
}
;
            if (us_since_bulk > 50000) {
                dl_state = DL_RUNNABLE;
            }
        }
#endif
    }

    return 0;
}
