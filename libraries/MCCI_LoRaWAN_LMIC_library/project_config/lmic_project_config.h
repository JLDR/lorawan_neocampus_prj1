/*
 * Project-specific definitions
 *
 * Thiebolt F.  jan.19  added duty-cycle override and remove older DR symbols
 * Thiebolt F.  dec.18  initial release
 */
// DEBUG level
#define LMIC_DEBUG_LEVEL    2

// Bands definitions
#define CFG_eu868 1
//#define CFG_us915 1
//#define CFG_au921 1
//#define CFG_as923 1
// #define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP	/* for as923-JP */
//#define CFG_in866 1
#define CFG_sx1276_radio 1
//#define LMIC_USE_INTERRUPTS

// WARNING: dangerous options
#define DISABLE_MCMD_DCAP_REQ               // disable 1% duty-cycle ;)
#define LMIC_ENABLE_DeviceTimeReq       1   // get time through LoRa ??

//#define LMIC_DISABLE_DR_LEGACY              // turn off legacy DR_* symbols that vary by bandplan.

