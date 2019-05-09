


#ifndef Fonctions_h_
#define Fonctions_h_

#include          <inttypes.h>
#include          <SPI.h>
#include          <Wire.h>
#include          <lmic.h>
#include          <hal/hal.h>
#include          <SFE_BMP180.h>
#include          <Adafruit_Si7021.h>
//#include          "SSD1306Wire.h"
//#include          "SSD1306.h"
//#include          <Adafruit_SSD1306.h>
//#include          <Fonts/FreeMonoOblique9pt7b.h>
//#include          <Adafruit_GFX.h>        // automatiquement appelée par Adafruit_SSD1306.h
//#include          <splash.h>
//#include          <STM32LowPower.h>       // à éviter avec Windows
//extern "C" {
//  #include        <os.h>
//}

// ********************* MACROS *********************
#define SIZEOFEXPR(x) sizeof(x)

// BSFrance's SX1276 related defs
#define RADIO_RST         PA2
#define RADIO_DIO0        PA3
#define RADIO_DIO1        PB0     // requires link from DIO1 to PB0 pin on BSFrance's stm32 boards
//#define OLED_RST          PB5
//#define OLED_SDA          PB7
//#define OLED_SCL          PB6
#define LED               PC13

#define RADIO_MOSI        PA7     // https://github.com/matthijskooijman/arduino-lmic/blob/master/README.md
#define RADIO_MISO        PA6
#define RADIO_SCK         PA5
#define RADIO_NSS         PA4

//#define SCREEN_WIDTH      128     // OLED display width, in pixels
//#define SCREEN_HEIGHT     64      // OLED display height, in pixels

#ifdef _BLACK_PILL
  #undef  LED
  #define LED             PB12
#endif /* _BLACK_PILL */

// LoRaWAN defines
#define DISABLE_DUTY_CYCLE      1         // disabling 1% constraint
#define DISABLE_ADR_MODE        1         // disabling Automatic Data Rate (ADR) mode
#define JOIN_TIMEOUT            45        // seconds between two consecutive joins
#define SX1276_RegVersion       0x42      // contains version of device
#define ALTITUDE                135.7     // Altitude of Toulouse in meters


typedef enum {
    dtSunday, dtMonday, dtTuesday, dtWednesday, dtThursday, dtFriday, dtSaturday
} dtDays_t;

typedef enum {dtStatusNotSet, dtStatusSet, dtStatusSync
} dtStatus_t ;

/* prototypage des fonctions */
extern void os_getArtEui(u1_t *);
extern void os_getDevEui(u1_t *);
extern void os_getDevKey(u1_t *);
extern void I2C_scanner(void);
//extern uint8_t BlinkFunction(uint8_t, uint16_t);
extern void do_send(osjob_t *);
extern void endLoop(void);
extern void onEvent(ev_t);
extern double MesTemp(void);
extern double MesPress(void);
extern void Mesures(void);
extern void ModelFound(void);
extern void MesureTemp(void);
extern void MesureHumidity(void);
//extern void initMyDisplay(void);



#endif      // Fonctions_h_
