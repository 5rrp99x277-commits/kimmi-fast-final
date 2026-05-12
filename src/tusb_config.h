#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUSB_MCU                OPT_MCU_RP2040
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS                 OPT_OS_NONE
#endif

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE       64
#endif

//--------------------------------------------------------------------
// VENDOR CLASS CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_VENDOR              1
#define CFG_TUD_VENDOR_RX_BUFSIZE   4096
#define CFG_TUD_VENDOR_TX_BUFSIZE   256

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
