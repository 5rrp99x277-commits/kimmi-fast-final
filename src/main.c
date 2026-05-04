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
#define STATUS_LED_ENABLE      0

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
#define BRCMF_C_GET_VAR             262
#define BRCMF_C_SET_VAR             263
#define BRCMF_POSTBOOT_ID           0xA123

//--------------------------------------------------------------------
// Real CYW4373 log values
//--------------------------------------------------------------------
#define CYW4373_CHIP_ID                 0x00004373u
#define CYW4373_CHIP_REV                0u
#define CYW4373_RAM_SIZE                0x000C0000u
#define CYW4373_EXPECTED_FW_SIZE        622592u
#define CYW4373_FW_VERSION_STRING       "wl0: Aug 3 2022 20:30:47 version 13.10.246.286 (4b0a74a CY WLTEST) FWID 01-e73c9ff8"
#define CYW4373_CLM_VERSION_STRING      "API: 18.1 Data: Murata.Type2AE Compiler: 1.35.0 ClmImport: 1.39.1 Customization: v3 23/02/09 Creation: 2023-02-09 04:01:47"
#define CYW4373_EVENT_MSGS_FILL         0xFFu

//--------------------------------------------------------------------
// Tolerance knobs
//--------------------------------------------------------------------
#define CYW4373_ASSUME_DL_START_ON_BULK          1
#define CYW4373_AUTO_RUNTIME_FALLBACK            1
#define CYW4373_AUTO_RUNTIME_FALLBACK_DELAY_MS   1200u
#define CYW4373_AUTO_RUNTIME_MIN_BYTES           32768u
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
static bool runtime_out_pending = false;
static uint16_t runtime_out_len = 0;
static uint32_t runtime_out_seq = 0;

// Backup iovar storage — survives even if bcdc_buf is touched between OUT and IN
static char last_iovar_name[64] = {0};
static bool last_iovar_valid = false;

static const uint8_t EMU_MAC[6] = {0x02, 0xCA, 0x37, 0x03, 0x87, 0x01};

// Realistic Murata Type 2AE / CYW4373 NVRAM identity fallback.
// If duplicate variables exist in NVRAM, Broadcom tools usually use the last one.
// We keep only one active emulator MAC: 02:CA:37:03:87:01.
#define CYW4373_MANFID     0x02D0u
#define CYW4373_PRODID     0x4373u
#define CYW4373_SROMREV    11u
#define CYW4373_BOARDREV   0x1301u
#define CYW4373_BOARDNUM   9492u
#define CYW4373_BOARDTYPE  0x083Du
#define CYW4373_DEVID      0x4418u
#define CYW4373_VENDID     0x14E4u
#define CYW4373_NOCRC      1u
#define CYW4373_AA2G       3u
#define CYW4373_AA5G       3u

static const uint8_t CYW4373_MAC_OTP_TUPLE[] = {
    0x80,       // Cypress tuple start
    0x07,       // length = tag + 6 MAC bytes
    0x19,       // tag = macaddr
    0x02, 0xCA, 0x37, 0x03, 0x87, 0x01
};

static const char CYW4373_NVRAM_TEXT[] =
    "manfid=0x2d0\n"
    "prodid=0x4373\n"
    "sromrev=11\n"
    "macaddr=02:CA:37:03:87:01\n"
    "boardrev=0x1301\n"
    "boardnum=9492\n"
    "boardtype=0x83d\n"
    "customvar1=0x222d0000\n"
    "aa2g=3\n"
    "aa5g=3\n"
    "devid=0x4418\n"
    "nocrc=1\n"
    "vendid=0x14e4\n"
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
            return NULL; // non-printable character — not an iovar name
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
        printf("%s DL_GETVER -> chip=0x%04lx\r\n",
               boot_mode ? "BOOT" : "RUNTIME/POSTBOOT",
               (unsigned long)id->chip);
        return tud_control_xfer(rhport, request, ctrl_buf,
                                min_u16(request->wLength, sizeof(*id)));
    }

    if (cmd == DL_GETSTATE) {
        rdl_state_t *st = (rdl_state_t *)ctrl_buf;
        uint32_t state = DL_WAITING;

        if (postboot || !boot_mode || fw_go_seen) {
            state = DL_RUNNABLE;
        } else if (!dl_started && fw_bytes_received == 0) {
            state = DL_READY;
        } else if (fw_bytes_received > 0) {
            // RUNNABLE only when host has been silent for 300 ms (all blocks sent)
            int64_t us_since_bulk = absolute_time_diff_us(last_bulk_time, get_absolute_time());
            if (us_since_bulk > 300000) {
                state = DL_RUNNABLE;
            } else {
                state = DL_READY;
            }
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
    // FIX: account for cmd=0 wrapper — skip embedded 4-byte command if present
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
                break;
            case BRCMF_C_DOWN:
                printf("RUNTIME BRCMF_C_DOWN -> OK\r\n");
                break;
            case BRCMF_C_GET_REVINFO:
                payload_len = last_bcdc_len ? last_bcdc_len : 68;
                if (payload_len < 68 && max_payload >= 68) payload_len = 68;
                if (payload_len > max_payload) payload_len = max_payload;
                memset(payload, 0, payload_len);
                if (payload_len >= 8) {
                    payload[0] = 0x73; payload[1] = 0x43; payload[2] = 0x00; payload[3] = 0x00;
                }
                if (payload_len >= 48) {
                    payload[44] = 0x73; payload[45] = 0x43; payload[46] = 0x00; payload[47] = 0x00;
                }
                if (payload_len >= 60) {
                    payload[48] = (uint8_t)(CYW4373_BOARDTYPE & 0xFF);
                    payload[49] = (uint8_t)((CYW4373_BOARDTYPE >> 8) & 0xFF);
                    payload[52] = (uint8_t)(CYW4373_BOARDREV & 0xFF);
                    payload[53] = (uint8_t)((CYW4373_BOARDREV >> 8) & 0xFF);
                }
                printf("RUNTIME GET_REVINFO -> chip 0x4373, boardtype=0x%04X boardrev=0x%04X\r\n",
                       CYW4373_BOARDTYPE, CYW4373_BOARDREV);
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
            printf("RUNTIME answer cur_etheraddr = %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                   EMU_MAC[0], EMU_MAC[1], EMU_MAC[2], EMU_MAC[3], EMU_MAC[4], EMU_MAC[5]);

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
            payload_len = 32;
            if (payload_len > max_payload) payload_len = max_payload;
            memset(payload, CYW4373_EVENT_MSGS_FILL, payload_len);
            printf("RUNTIME answer event_msgs len=%u\r\n", payload_len);

        } else if (strncmp(iovar, "country", 20) == 0) {
            payload_len = max_payload >= 12 ? 12 : max_payload;
            if (payload_len >= 4) memcpy(payload, "RU\0\0", 4);
            if (payload_len >= 12) memcpy(payload + 8, "RU\0\0", 4);

        } else if (strncmp(iovar, "mpc", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 1);

        } else if (strncmp(iovar, "manfid", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_MANFID);
            printf("RUNTIME answer manfid=0x%04X\r\n", CYW4373_MANFID);

        } else if (strncmp(iovar, "prodid", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_PRODID);
            printf("RUNTIME answer prodid=0x%04X\r\n", CYW4373_PRODID);

        } else if (strncmp(iovar, "sromrev", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_SROMREV);
            printf("RUNTIME answer sromrev=%u\r\n", CYW4373_SROMREV);

        } else if (strncmp(iovar, "boardrev", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_BOARDREV);
            printf("RUNTIME answer boardrev=0x%04X\r\n", CYW4373_BOARDREV);

        } else if (strncmp(iovar, "boardnum", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_BOARDNUM);
            printf("RUNTIME answer boardnum=%u\r\n", CYW4373_BOARDNUM);

        } else if (strncmp(iovar, "boardtype", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_BOARDTYPE);
            printf("RUNTIME answer boardtype=0x%04X\r\n", CYW4373_BOARDTYPE);

        } else if (strncmp(iovar, "devid", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_DEVID);
            printf("RUNTIME answer devid=0x%04X\r\n", CYW4373_DEVID);

        } else if (strncmp(iovar, "vendid", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_VENDID);
            printf("RUNTIME answer vendid=0x%04X\r\n", CYW4373_VENDID);

        } else if (strncmp(iovar, "macaddr", 20) == 0) {
            payload_len = max_payload >= 8 ? 8 : max_payload;
            memcpy(payload, EMU_MAC, payload_len >= 6 ? 6 : payload_len);
            printf("RUNTIME answer macaddr = %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                   EMU_MAC[0], EMU_MAC[1], EMU_MAC[2], EMU_MAC[3], EMU_MAC[4], EMU_MAC[5]);

        } else if (strncmp(iovar, "vars", 20) == 0 ||
                   strncmp(iovar, "nvram", 20) == 0 ||
                   strncmp(iovar, "nvram_dump", 20) == 0 ||
                   strncmp(iovar, "cisdump", 20) == 0 ||
                   strncmp(iovar, "otpdump", 20) == 0) {
            payload_len = (uint16_t)(strlen(CYW4373_NVRAM_TEXT) + 1);
            if (payload_len > max_payload) payload_len = max_payload;
            memcpy(payload, CYW4373_NVRAM_TEXT, payload_len);
            printf("RUNTIME answer %s with NVRAM/OTP text len=%u\r\n",
                   safe_iovar_name((uint8_t*)iovar, 64) ? iovar : "dump", payload_len);

        } else if (strncmp(iovar, "nocrc", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_NOCRC);
            printf("RUNTIME answer nocrc=%u\r\n", CYW4373_NOCRC);

        } else if (strncmp(iovar, "aa2g", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_AA2G);
            printf("RUNTIME answer aa2g=%u\r\n", CYW4373_AA2G);

        } else if (strncmp(iovar, "aa5g", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, CYW4373_AA5G);
            printf("RUNTIME answer aa5g=%u\r\n", CYW4373_AA5G);

        } else if (strncmp(iovar, "mac_otp_tuple", 20) == 0 ||
                   strncmp(iovar, "otp_mac_tuple", 20) == 0) {
            payload_len = sizeof(CYW4373_MAC_OTP_TUPLE);
            if (payload_len > max_payload) payload_len = max_payload;
            memcpy(payload, CYW4373_MAC_OTP_TUPLE, payload_len);
            printf("RUNTIME answer MAC OTP tuple len=%u\r\n", payload_len);

        } else if (strncmp(iovar, "mimo_bw_cap", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);
            printf("RUNTIME answer mimo_bw_cap=0\r\n");

        } else if (strncmp(iovar, "ampdu_ba_wsize", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 64);
            printf("RUNTIME answer ampdu_ba_wsize=64\r\n");

        } else if (strncmp(iovar, "join_pref", 20) == 0) {
            payload_len = copy_u32_payload(payload, max_payload, 0);
            printf("RUNTIME answer join_pref=0\r\n");

        } else if (strncmp(iovar, "cap", 20) == 0 ||
                   strncmp(iovar, "chanspecs", 20) == 0 ||
                   strncmp(iovar, "escanresults", 20) == 0) {
            payload_len = last_bcdc_len ? last_bcdc_len : 4;
            if (payload_len > max_payload) payload_len = max_payload;
            memset(payload, 0, payload_len);

        } else {
            printf("RUNTIME unknown GET_VAR '%s' -> zero success\r\n",
                   safe_iovar_name((uint8_t*)iovar, 128) ? iovar : "?");
            payload_len = 0;
        }

    } else if (effective_cmd == BRCMF_C_SET_VAR) {
        printf("RUNTIME SET_VAR '%s' -> OK\r\n",
               last_iovar_valid ? last_iovar_name : "?");
        payload_len = 0;

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
    return sizeof(bcdc_hdr_t) + payload_len;
}

static bool is_supported_bcdc_cmd(uint32_t cmd) {
    return cmd == BRCMF_C_GET_VAR ||
           cmd == BRCMF_C_SET_VAR ||
           cmd == BRCMF_C_GET_REVINFO ||
           cmd == BRCMF_C_UP ||
           cmd == BRCMF_C_DOWN ||
           cmd == BRCMF_C_SET_SCAN_CHANNEL_TIME ||
           cmd == BRCMF_C_SET_SCAN_UNASSOC_TIME;
}

static bool bcdc_header_all_zero(const bcdc_hdr_t *hdr) {
    return hdr->cmd == 0 && hdr->len == 0 && hdr->flags == 0 && hdr->status == 0;
}

static bool parse_runtime_out(uint16_t total) {
    if (total > sizeof(ctrl_buf)) total = sizeof(ctrl_buf);

    printf("parse_out: total=%u", total);
    uint16_t dump_len = total < 64 ? total : 64;
    for (uint16_t i = 0; i < dump_len; i++) {
        printf(" %02X", ctrl_buf[i]);
    }
    printf("\r\n");

    if (total < sizeof(bcdc_hdr_t)) {
        printf("parse_out: too short\r\n");
        return false;
    }

    bcdc_hdr_t *hdr = (bcdc_hdr_t *)ctrl_buf;

    if (bcdc_header_all_zero(hdr)) {
        printf("parse_out: empty/all-zero header, not ready\r\n");
        return false;
    }

    if (hdr->cmd != 0 && !is_supported_bcdc_cmd(hdr->cmd)) {
        // Keep unknown non-zero IOCTL commands instead of falling back to cmd=0.
        // prepare_bcdc_response() will ACK/noop them, which is safer during init.
        printf("parse_out: unknown nonzero cmd=%lu len=%lu flags=0x%08lX status=%lu -> preserve/ACK\r\n",
               (unsigned long)hdr->cmd,
               (unsigned long)hdr->len,
               (unsigned long)hdr->flags,
               (unsigned long)hdr->status);
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
        printf("parse_out: no valid iovar cmd=%lu len=%lu flags=0x%08lX\r\n",
               (unsigned long)last_bcdc_cmd,
               (unsigned long)last_bcdc_len,
               (unsigned long)last_bcdc_flags);
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

        printf("CTRL %s bm=0x%02X bReq=%u wLen=%u boot=%u pending=%u seq=%lu\r\n",
               stage == CONTROL_STAGE_DATA ? "DATA" : "ACK",
               request->bmRequestType,
               request->bRequest,
               request->wLength,
               (unsigned)boot_mode,
               (unsigned)runtime_out_pending,
               (unsigned long)runtime_out_seq);

        bool ok = parse_runtime_out(runtime_out_len);
        printf("CTRL -> parse_runtime_out %s at %s\r\n",
               ok ? "done" : "not-ready",
               stage == CONTROL_STAGE_DATA ? "DATA" : "ACK");

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

                printf("BOOT %s bReq=%u wLen=%u fw_bytes=%lu\r\n",
                       dl_name(cmd), cmd, len, (unsigned long)fw_bytes_received);

                if (cmd == DL_GETVER) {
                    status_led_dl();
                    // FAST DL_GETVER: returns chip id 0x4373 or postboot 0xA123.
                    bootrom_id_t *id = (bootrom_id_t *)ctrl_buf;
                    fill_bootrom_id(id, postboot);
                    printf("BOOT DL_GETVER -> chip=0x%04lX\r\n", (unsigned long)id->chip);
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
                    printf("BOOT DL_START received\r\n");
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(rdl_state_t)));
                }

                if (cmd == DL_GETSTATE) {
                    // FAST DL_GETSTATE response path:
                    // brcmfmac waits for URB completion and times out if this control transfer is not answered quickly.
                    uint32_t state = DL_WAITING;

                    if (fw_bytes_received != last_getstate_bytes) {
                        last_getstate_bytes = fw_bytes_received;
                        getstate_poll_count = 0;
                    } else {
                        getstate_poll_count++;
                    }

                    if (fw_go_seen) {
                        state = DL_RUNNABLE;
                    } else if (!dl_started && fw_bytes_received == 0) {
                        // Real bootloader can report READY before DL_START.
                        state = DL_READY;
                    } else if (dl_started && fw_bytes_received > 0) {
                        // RUNNABLE only when host has been silent for 300 ms (all blocks sent)
                        int64_t us_since_bulk = absolute_time_diff_us(last_bulk_time, get_absolute_time());
                        if (us_since_bulk > 300000) {
                            state = DL_RUNNABLE;
                        } else {
                            state = DL_READY;
                        }
                    }

                    if (state == DL_RUNNABLE && !runnable_seen) {
                        runnable_seen = true;
                        runnable_seen_ms = to_ms_since_boot(get_absolute_time());
                        printf("BOOT RUNNABLE seen at %lu ms\r\n", (unsigned long)runnable_seen_ms);
                    }

                    printf("BOOT DL_GETSTATE immediate response: state=%lu bytes=%lu expected=%lu polls=%lu\r\n",
                           (unsigned long)state,
                           (unsigned long)fw_bytes_received,
                           (unsigned long)CYW4373_EXPECTED_FW_SIZE,
                           (unsigned long)getstate_poll_count);

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
                    printf("BOOT DL_GO received, reconnect planned at %lu ms\r\n", (unsigned long)dl_go_reconnect_at_ms);

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
                    printf("BOOT DL_RESETCFG -> reconnect scheduled\r\n");
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(bootrom_id_t)));
                }

                if (cmd == DL_REBOOT) {
                    needs_reconnect = true;
                    dl_go_reconnect_at_ms = 0;
                    rdl_state_t *st = (rdl_state_t *)ctrl_buf;
                    fill_rdl_state(st, DL_RUNNABLE, fw_bytes_received);
                    printf("BOOT DL_REBOOT -> reconnect scheduled\r\n");
                    return tud_control_xfer(rhport, request, ctrl_buf, min_u16(len, sizeof(rdl_state_t)));
                }

                printf("BOOT unknown vendor-IN bReq=%u -> zero success\r\n", cmd);
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
                printf("CTRL SETUP bm=0x%02X bReq=%u wLen=%u boot=%u pending=%u seq=%lu\r\n",
                       request->bmRequestType,
                       request->bRequest,
                       request->wLength,
                       (unsigned)boot_mode,
                       (unsigned)runtime_out_pending,
                       (unsigned long)runtime_out_seq);
                return tud_control_xfer(rhport, request, ctrl_buf, rx_len);
            }

            if (request->bmRequestType == 0xA1 && request->bRequest == 1) {
                uint16_t tx_len = min_u16(request->wLength, sizeof(ctrl_buf));
                if (runtime_out_pending) {
                    bool ok = parse_runtime_out(runtime_out_len);
                    printf("CTRL -> late parse_runtime_out %s before IN\r\n",
                           ok ? "done" : "not-ready");
                    if (ok) {
                        runtime_out_pending = false;
                    }
                }
                clear_ctrl();
                uint16_t resp_len = prepare_bcdc_response(ctrl_buf, tx_len);
                if (resp_len > tx_len) resp_len = tx_len;
                const char *name = last_iovar_valid ? last_iovar_name : "";
                printf("RUNTIME BCDC IN: cmd=%lu iovar='%s' resp_len=%u host_len=%u\r\n",
                       (unsigned long)last_bcdc_cmd,
                       name,
                       resp_len,
                       request->wLength);
                return tud_control_xfer(rhport, request, ctrl_buf, resp_len);
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

    // If a chunk was received recently but the host stopped polling control endpoint,
    // this branch fires when tud_vendor_available() returns 0.
    // No need to change DL state here; it stays READY because last_bulk_time was updated
    // by the chunk itself.
}


//--------------------------------------------------------------------
// Reconnect helpers
//--------------------------------------------------------------------
static void switch_to_runtime_reconnect(void) {
    if (!needs_reconnect && !runnable_seen) return;

    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (needs_reconnect) {
        if (now < reconnect_at_ms) return;
    } else if (runnable_seen) {
        if (now < (runnable_seen_ms + CYW4373_AUTO_RUNTIME_FALLBACK_DELAY_MS)) return;
    }

    // Deassert then reassert USB pull-up to force bus reset / re-enumeration.
    tud_disconnect();
    sleep_ms(30);
    tud_connect();

    boot_mode = false;
    needs_reconnect = false;
    reconnect_at_ms = 0;
    dl_go_reconnect_at_ms = 0;
    runnable_seen = false;
    status_led_runtime();
    printf("USB mounted as runtime 04b4:0bdc\r\n");
}

static void check_dl_go_reconnect(void) {
    if (dl_go_reconnect_at_ms == 0) return;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now >= dl_go_reconnect_at_ms) {
        tud_disconnect();
        sleep_ms(30);
        tud_connect();

        boot_mode = false;
        needs_reconnect = false;
        reconnect_at_ms = 0;
        dl_go_reconnect_at_ms = 0;
        status_led_runtime();
        printf("SWITCH to runtime mode 04b4:0bdc\r\n");
    }
}

//--------------------------------------------------------------------
// Entry
//--------------------------------------------------------------------
int main(void) {
    stdio_uart_init();
    board_init();
    printf("=== CYW4373-Emulator on RP2040-Zero ===\r\n");

    if (STATUS_LED_ENABLE) {
        gpio_init(STATUS_WS2812_PIN);
        gpio_set_dir(STATUS_WS2812_PIN, GPIO_OUT);
        gpio_put(STATUS_WS2812_PIN, 0);

        gpio_init(STATUS_GPIO_LED_PIN);
        gpio_set_dir(STATUS_GPIO_LED_PIN, GPIO_OUT);
        gpio_put(STATUS_GPIO_LED_PIN, 0);
    }

    gpio_init(WL_REG_ON_PIN);
    gpio_set_dir(WL_REG_ON_PIN, GPIO_IN);

    tud_init(BOARD_TUD_RHPORT);

    // Wait for Canon WL_REG_ON high. (Canon pin 1 -> RP2040 GPIO2).
    // If not connected, tie GPIO2 to 3.3V so the emulator starts on boot.
    bool wl_reg_on_seen = false;
    uint32_t wait_start = to_ms_since_boot(get_absolute_time());

    while (1) {
        tight_loop_contents();

        tud_task();

        if (!wl_reg_on_seen) {
            if (gpio_get(WL_REG_ON_PIN)) {
                wl_reg_on_seen = true;
                status_led_power();
                printf("WL_REG_ON high, delay %u ms before USB attach\r\n", POWER_STABLE_DELAY_MS);
                sleep_ms(POWER_STABLE_DELAY_MS);
                tud_connect();
                status_led_boot();
                printf("USB mounted as boot 04b4:bd29\r\n");
            } else {
                if (WL_REG_ON_WAIT_TIMEOUT_MS > 0) {
                    uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - wait_start;
                    if (elapsed > WL_REG_ON_WAIT_TIMEOUT_MS) {
                        printf("WL_REG_ON timeout after %u ms, starting anyway\r\n", WL_REG_ON_WAIT_TIMEOUT_MS);
                        sleep_ms(POWER_STABLE_DELAY_MS);
                        tud_connect();
                        status_led_boot();
                        printf("USB mounted as boot 04b4:bd29\r\n");
                        break;
                    }
                }
            }
        }

        if (boot_mode) {
            drain_vendor_out();
            check_dl_go_reconnect();
            switch_to_runtime_reconnect();
        } else {
            tud_task();
        }
    }
}
