 /*
 * Notes: 
 * - DIO1 need to get exterally connected to PB0
 * - correct <LMIC lib>/src/hal/hal.cpp according to
 * https://www.thethingsnetwork.org/forum/t/big-stm32-boards-topic/13391/74
 * https://lorawan.univ-tlse3.fr/admin#/dashboard
 * François Jan.19  add low power mode
 * François Dec.18  initial release
 */

#include "Fonctions.h"

// 
//// This EUI must be in little-endian format, so least-significant-byte first. When copying an EUI from ttnctl output, this means to reverse
//// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
//static const u1_t DevEUI[8] PROGMEM = {0xAD, 0xDE, 0x02, 0x00, 0xAD, 0xDE, 0xAD, 0xDE};     // test avec 0xDEADDEAD0002DEAD
//static const u1_t AppEUI[8] PROGMEM = {0xAD, 0xDE, 0x42, 0xAD, 0xDE, 0x42, 0xAD, 0xDE};     // neOCamous-Lora test 0xDEAD42DEAD42DEAD
//// This key should be in big endian format (or, since it is not really a number but a block of memory, endianness does not really apply). 
//// In practice, a key taken from ttnctl can be copied as-is.
//static const u1_t AppKey[16] PROGMEM = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
#ifdef DISABLE_DUTY_CYCLE
const unsigned TX_INTERVAL = 30;
#else
const unsigned TX_INTERVAL = 300;
#endif
uint8_t                 _res;
uint8_t                 i, j, k, l, m, n, t, p;
char                    TabASCII[30];
uint8_t                 PaquetMesures[24];
char                    Etat;
double                  Mesure;
double                  Temp, Press, Press_SeaLevel, Altitude, Mes1;
bool                    _need2reboot = false;     // flag to tell a reboot is requested
char                    *string1 = "Connection between";
char                    *string2 = "end-device<->gateway";

// In LoRa mode the DIO pins are used as follows:
// DIO0: TxDone and RxDone
// DIO1: RxTimeout
// Pin mapping https://codeforwin.org/2018/07/how-to-declare-initialize-and-access-structures-in-c.html
const lmic_pinmap lmic_pins = {                         // declare and intialize a structure (oslmic.c) extern const struct lmic_pinmap lmic_pins;
    .nss = RADIO_NSS,                                   // #define RADIO_NSS PA4
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RADIO_RST,                                   // #define RADIO_RST PA2
    .dio = {RADIO_DIO0, RADIO_DIO1, LMIC_UNUSED_PIN},   // DIO2 is only useful when using FM modulation
};                                                      // For the DIO pins, the three numbers refer to DIO0, DIO1 and DIO2 respectively

SFE_BMP180 CapteurBMP180;
Adafruit_Si7021 CapteurSi7021 = Adafruit_Si7021();
//Adafruit_SSD1306 MyOled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);
  Serial.print(F("Le port série de la carte BSFrance est accessible avec le terminal\n"));
  Serial.print(F("Le taux de communication qui doit être programmé est 115200 bauds\n"));
  Serial.print(F("\n[setup] beginning of setup ..."));
  Serial.flush();
  // BSFrance stm32 boards feature an oled display that needs to get a reset procedure
  Serial.print(F("\n[setup] reset OLED display ..."));
  Serial.flush();
  //pinMode(OLED_RST, OUTPUT);
  //digitalWrite(OLED_RST, LOW);    // oled reset
  //delay(250);
  //digitalWrite(OLED_RST, HIGH);   // oled normal mode
  delay(50);
  Serial.println(F("\nI2C bus activation ..."));
  Serial.flush();
  delay(1000);
  if (CapteurBMP180.begin())
    Serial.println("BMP180 init success");
  else {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    while(1);                     // Pause forever.
  }
  if (!CapteurSi7021.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (1);
  } else {
    ModelFound();
    //Serial.println(F("init success"));
  }
  //initMyDisplay();
  Serial.print(F("\n[setup] reset sx1276 ..."));
  Serial.flush();
  pinMode(RADIO_RST, OUTPUT);
  digitalWrite(RADIO_RST, LOW);   // manual reset of the device
  delay(5);                       // min is 100us
  pinMode(RADIO_RST, INPUT);      // sx1276 module feature an internal pull-up
  Serial.print(F("\n[setup] setup & create SPI bus ..."));      // Configure & Instantiate SPI bus
  Serial.flush();
  SPI.begin();
  pinMode(RADIO_NSS, OUTPUT);
  digitalWrite(RADIO_NSS, HIGH);  // desactivate NSS
  // read version register
  digitalWrite(RADIO_NSS, LOW);
  SPI.transfer(0x7F & SX1276_RegVersion);
  _res = SPI.transfer(0x00);
  digitalWrite(RADIO_NSS, HIGH);
  Serial.print(F("\n[setup] SX1276 reg ver = 0x"));
  Serial.println(_res, HEX);
  Serial.flush();
  //BlinkFunction(15, 150);
  //LowPower.begin();       // problème avec l'IDE sous Windows
  os_init();
  delay(250);
  Serial.print(F("\n[setup] call to LMIC reset ..."));  // Reset the MAC state. Session and pending data transfers will be discarded.
  Serial.flush();
  LMIC_reset();
  delay(250);
  Serial.print(F("\n[setup] setup LoRaWAN link ..."));  // configure link
  Serial.flush();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  #ifdef DISABLE_DUTY_CYCLE
    Serial.print(F("\n[setup] DISABLED 1% DUTY-CYCLE !!!"));
    Serial.flush();
    delay(1000);
  #endif
  #ifndef DISABLE_ADR_MODE
    LMIC_setAdrMode(true);
  #else
    LMIC_setAdrMode(false);         // ADR mode disabled
    uint8_t _res = 0;               // set channels
    _res = LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF12),  BAND_CENTI);  // g-band
    if( _res != 1 ) {
      Serial.print(F("\n[setup] channel setup failed ..."));
      Serial.flush();
    }
    //LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);    // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);        // g-band
    //LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);    // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);        // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);        // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);        // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);        // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);        // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);        // g-band
  #endif
  LMIC_setDrTxpow(EU868_DR_SF12, 14);         // EU868_DR_SF12 = 0, (enum dans lorabase_eu868.h)
  delay(250);
  LMIC.globalDutyRate = 0;
  Serial.println(F("\n[setup] end of setup ..."));
  Serial.flush();
  //for (int channel = 1; channel < 8; channel++) LMIC_disableChannel(channel);
  //for (int channel = 1; channel < 8; channel++) LMIC_enableChannel(channel);
  //LMIC_setLinkCheckMode(0);                 // Must be called only if a session is established
  I2C_scanner();
  Temp = MesTemp();
  Press = MesPress();
  MesureTemp();               // Si7021
  MesureHumidity();           // Si7021
}

void loop() {
  os_runloop_once();          // oslmic.c
  endLoop();
}





/* ######################################################################################################## */
// END of file
