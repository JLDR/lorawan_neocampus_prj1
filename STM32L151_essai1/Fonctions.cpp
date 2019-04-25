/*
  DateTime.cpp - Arduino Date and Time library
  Copyright (c) Michael Margolis.  All right reserved.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
*/

#include "Fonctions.h"

#define LEAP_YEAR(_year) ((_year%4) == 0)

/* ########################## VARIABLES EXTERNES ########################## */
extern uint8_t              i, j, k, l, m, n, t, p;
extern char                 TabASCII[30];
extern uint8_t              PaquetMesures[26];                 
extern const uint16_t       TX_INTERVAL = 300;
// This EUI must be in little-endian format, so least-significant-byte first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
static const u1_t DevEUI[8] PROGMEM = {0xAD, 0xDE, 0x02, 0x00, 0xAD, 0xDE, 0xAD, 0xDE};     // test avec 0xDEADDEAD0002DEAD
static const u1_t AppEUI[8] PROGMEM = {0xAD, 0xDE, 0x42, 0xAD, 0xDE, 0x42, 0xAD, 0xDE};     // neOCamous-Lora test 0xDEAD42DEAD42DEAD
// This key should be in big endian format (or, since it is not really a number but a block of memory, endianness does not really apply). 
// In practice, a key taken from ttnctl can be copied as-is.
static const u1_t AppKey[16] PROGMEM = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
//extern const u1_t DevEUI[8] PROGMEM;
//extern const u1_t AppEUI[8] PROGMEM;
//extern const u1_t AppKey[16] PROGMEM;
extern bool                 _need2reboot;             // flag to tell a reboot is requested
extern char                 Etat;
extern double               Mesure;
extern double               Temp, Press, Press_SeaLevel, Altitude, Mes1;
extern char                 *string1;
extern char                 *string2;

/* ########################## VARIABLES LOCALES ########################## */
static byte monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static uint8_t mydata[] = "Hello from stm32l151";
static uint8_t frame_deg[] = "C and ";
static uint8_t frame_CapteurBMP180[] = "mbar";
static osjob_t sendjob;                             // structure de oslmic.h
bool _joined = false;                               // flag to tell if JOIN has been established
uint8_t _lastJoin = -1;                             // seconds elapsed since last JOIN trial

/* ########################## Objets ########################## */
extern SFE_BMP180 CapteurBMP180;
extern Adafruit_Si7021 CapteurSi7021;
//extern Adafruit_SSD1306 MyOled;
/****************************************************************************************************/
/* Fonction de l'OS pour récupérer l'ensemble des paramètres connus du capteur et du serveur.       */
/****************************************************************************************************/
void os_getArtEui(u1_t *buf) {      // buffer de type uint8_t 
  memcpy_P(buf, AppEUI, 8);
}
void os_getDevEui(u1_t *buf) {
  memcpy_P(buf, DevEUI, 8);
}
void os_getDevKey(u1_t *buf) {
  memcpy_P(buf, AppKey, 16);
}
/****************************************************************************************************/
/* Fonction pour scanner le bus I2C. The i2c_scanner uses the return value of the                   */
/* Write.endTransmisstion to see if a device did acknowledge to the address.                        */
/****************************************************************************************************/
void I2C_scanner(void) {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0) Serial.println("No I2C devices found");
  else Serial.println("scanning done");
  delay(1000);
}
/****************************************************************************************************/
/* Blink                                                                                            */
/****************************************************************************************************/
//uint8_t BlinkFunction(uint8_t nbr_bcl, uint16_t delai) {
//  uint8_t inc;
//  for (k = 0; k < nbr_bcl; k++) {
//    digitalWrite(LED, HIGH);
//    delay(delai);
//    digitalWrite(LED, LOW);
//    delay(delai);
//  }
//  return 1;
//}
/****************************************************************************************************/
/* Data sending                                                                                     */
/* LMIC_setTxData2 : prepare upstream data transmission at the next possible time.                  */
/* int LMIC_setTxData2(u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed)                        */
/* void os_setTimedCallback(osjob_t *job, ostime_t time, osjobcb_t cb)                              */
/****************************************************************************************************/
void do_send(osjob_t *j) {            // oslmic.h (structure avec type prédéfini)
  if (LMIC.opmode & OP_TXRXPEND) {    // Check if there is not a current TX/RX job running (lmic.h)
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {                            // Prepare upstream data transmission at the next possible time.
    //LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);    // int LMIC_setTxData2(u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed);
    Mesures();                                              // Here the sensor information should be retrieved (Pressure: 300...1100 hPa)
    //LMIC_setTxData2(1, PaquetMesures, sizeof(PaquetMesures), 1);      // we want confirmation (ACK ?) (lmic.c)
    LMIC_setTxData2(1, PaquetMesures, sizeof(PaquetMesures), 0);      // no acknowledge
    Serial.println(F("Packet queued"));
  }
  // we immediately set a new call
  Serial.print(F("\n[DATA] next data sending in "));
  Serial.print(TX_INTERVAL, DEC);
  Serial.println(F(" seconds ..."));
  Serial.flush();
  //os_setTimedCallback(&(*j), os_getTime() + sec2osticks(TX_INTERVAL), do_send);  // oslmic.c
  os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
}
/****************************************************************************************************/
/* process end of main loop: specific functions executed every seconds                              */
/****************************************************************************************************/
void endLoop(void) {
  static unsigned long _lastCheck = 0;    // elapsed ms since last check
  static bool _firstTimeJob = true;       // set osjob timer after join
  // check if a reboot has been requested ...
  if (_need2reboot) {
    Serial.print(F("\n[endLoop] a reboot has been asked ..."));
    Serial.flush();
    while(true) {
      delay(1000);
      Serial.print(F("\n[endLoopT] reboot manually your board ..."));
      Serial.flush();
    }
  }
  //do {
  //} while((millis() - _lastCheck) < (unsigned long)1000UL);                 // https://www.arduino.cc/reference/en/language/functions/time/millis/
  if (((millis() - _lastCheck) >= (unsigned long)1000UL) != true ) return;  // is a second elapsed ?
  _lastCheck = millis();        // at least one second elapsed ...
  Serial.print(F("."));         // serial link activity marker ...
  if (not _joined) {            // check for joined, if not joined and last trial was JOIN_TIMEOUT seconds earlier, retry!
    if ((_lastJoin >= (unsigned)JOIN_TIMEOUT) == true ) {
      _lastJoin = 0;
      Serial.print(F("\n[endLoop] (re)starting OTAA join ..."));
      Serial.flush();
      LMIC_startJoining();
    } else _lastJoin++;
    return;
  } else _lastJoin = 0;
  // at this point, we're connected (JOIN) ...
  if (_joined == true and _need2reboot == false and _firstTimeJob == true ) {
    _firstTimeJob = false;
    do_send(&sendjob);
  }
}
/****************************************************************************************************/
/* Procédures d'interruption à développer ici, même si elles ne sont pas utilisées par le programme */
/* principal. Tous les évènements potentiels sont déclarés dans lmic.h sous forme d'une             */
/* énumération. C'est le même principe que pour les fonctions ISR proposées par AVR.                */
/****************************************************************************************************/
void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(F(": "));
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      _joined = false;
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print(F("netid: "));
        Serial.println(netid, DEC);
        Serial.print(F("devaddr: "));
        Serial.println(devaddr, HEX);
        Serial.print(F("artKey: "));
        for (int i = 0; i < sizeof(artKey); ++i) {
          Serial.print(artKey[i], HEX);
        }
        Serial.println("");
        Serial.print(F("nwkKey: "));
        for (int i = 0; i < sizeof(nwkKey); ++i) {
          Serial.print(nwkKey[i], HEX);
        }
        Serial.println("");
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      _joined = true;
      break;
      /*  This event is defined but not used in the code. No point in wasting codespace on it.
      case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
      */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      _joined = false;
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      _joined = false;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK) Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      /* Schedule next transmission for END-USER DATA
      if (_joined == true and _need2reboot == false) {
        Serial.print(F("\n[DATA] next data sending in "));
        Serial.print(TX_INTERVAL,DEC);
        Serial.println(F(" seconds ...")); 
        Serial.flush();
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      }
      */
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      _joined = false;
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
      /* This event is defined but not used in the code. No point in wasting codespace on it.
      case EV_SCAN_FOUND:
        Serial.println(F("EV_SCAN_FOUND"));
        break;
      */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}
/****************************************************************************************************/
/* Mesure de la température avec initialisation et acquisition d'une valeur flottante.              */
/* The measurement is stored in the variable T. Function returns 1 if successful, 0 if failure.     */
/****************************************************************************************************/
double MesTemp(void) {
  //Serial.println(F("Démarrage de la mesure avec la méthode startTemperature()"));
  Etat = CapteurBMP180.startTemperature();
  Serial.print(F("Réponse de la méthode : "));
  Serial.println((uint8_t)Etat, DEC);
  if ((uint8_t)Etat == 5) {
    delay((uint8_t)Etat * 2);                          // Wait for the measurement to complete
    Etat = CapteurBMP180.getTemperature(Temp);
    if (Etat != 0) {
      Serial.print("temperature : ");
      dtostrf(Temp, 5, 2, TabASCII);
      for (k = 0; k < 5; k++) Serial.print(TabASCII[k]);
      Serial.print(" °C ou ");
      Mes1 = ((9.0 / 5.0) * Temp) + 32.0;
      Serial.print(Mes1, 2);              // deux décimales
      Serial.println(" °F");
      return Temp; 
    } else return 0.0;
  } else Serial.println("error starting temperature measurement\n");
}
/****************************************************************************************************/
/* Mesure de la pression atmosphérique avec initialisation et acquisition d'une valeur flottante.   */
/* The measurement is stored in the variable T. Function returns 1 if successful, 0 if failure.     */
/****************************************************************************************************/
double MesPress(void) {
  double MyTemp;        // valeur locale utilisée pour calcul
  MyTemp = MesTemp();
  Etat = CapteurBMP180.startPressure('3');
  if (Etat != 0) {
    delay(Etat);        
    Etat = CapteurBMP180.getPressure(Press, MyTemp);
    if (Etat != 0) {
      Serial.print("absolute Capteur BMP180 : ");
      dtostrf(Press, 7, 2, TabASCII);
      for (k = 0; k < 7; k++) Serial.print(TabASCII[k]);
      Serial.print(" mb or ");
      Mes1 = Press * 0.0295333727;
      Serial.print(Mes1, 2);          // deux décimales
      Serial.println(" inHg");
      Press_SeaLevel = CapteurBMP180.sealevel(Press ,ALTITUDE);
      Serial.print("relative (sea-level) Capteur BMP180 : ");
      Serial.print(Press_SeaLevel, 2);
      Serial.print(" mb or, ");
      Mes1 = Press_SeaLevel * 0.0295333727;
      Serial.print(Mes1, 2);
      Serial.println(" inHg");
      Altitude = CapteurBMP180.altitude(Press, Press_SeaLevel);
      Serial.print("computed altitude : ");
      Serial.print((uint16_t)Altitude);
      Serial.print(" meters or ");
      Mes1 = Altitude * 3.28084;
      Serial.print(Mes1, 1);
      Serial.println(" feet");
      return Press; 
    } else return 0.0;
  } else Serial.println("error starting Capteur BMP180 measurement\n"); 
}
/****************************************************************************************************/
/* Acquisition de la température et de la pression atmosphérique avec mise en forme dans le tableau */
/* PaquetMesures qui sera transmis en tant que paquet de la trame LoRa.                             */
/****************************************************************************************************/
void Mesures(void) {                // uint8_t *Mesures(void);
  memset(PaquetMesures, '\0', sizeof(PaquetMesures));
  Mesure = MesTemp();
  dtostrf(Mesure, 5, 2, TabASCII);
  for (k = 0; k < 5; k++) PaquetMesures[k] = (uint8_t)TabASCII[k];
  PaquetMesures[5] = 0x20;
  PaquetMesures[6] = 0xA7;                                                // => °
  for (k = 0; k < 6; k++) PaquetMesures[7 + k] = frame_deg[k];            // "C and ";
  Mesure = MesPress(); 
  dtostrf(Mesure, 7, 2, TabASCII);
  for (k = 0; k < 7; k++) PaquetMesures[13 + k] = (uint8_t)TabASCII[k];
  PaquetMesures[20] = 0x20;
  for (k = 0; k < 4; k++) PaquetMesures[21 + k] = frame_CapteurBMP180[k]; // "mbar";
  PaquetMesures[26] = '\0';
  //return (uint8_t *)PaquetMesures[0];
  Serial.print(F("Paquet transmis : "));
  for (k = 0; k < 27; k++) {
    Serial.print("0x");
    Serial.print((char)PaquetMesures[k], HEX);
    Serial.print(' ');
  }
  Serial.println();
}
/****************************************************************************************************/
/* Recherche du modèle de capteur Si7021 ou autre.                                                  */
/****************************************************************************************************/
void ModelFound(void) {
  switch(CapteurSi7021.getModel()) {
    case SI_Engineering_Samples:
      Serial.println("SI engineering samples");
      break;
    case SI_7013:
      Serial.println("Si7013");
      break;
    case SI_7020:
      Serial.println("Si7020");
      break;
    case SI_7021:
      Serial.println("Si7021");
      break;
    case SI_UNKNOWN:
    default:
      Serial.println("Unknown");
  }
  Serial.print(F("Rev("));
  Serial.print(CapteurSi7021.getRevision());
  Serial.println(F(")"));
  Serial.print(" Serial #");
  Serial.print(CapteurSi7021.sernum_a, HEX);
  Serial.println(CapteurSi7021.sernum_b, HEX);
}
/****************************************************************************************************/
/* Mesure de la température.                                                                        */
/****************************************************************************************************/
void MesureTemp(void) {
  Serial.print(F("Température du capteur Si7021 : "));
  Serial.print(CapteurSi7021.readTemperature(), 2);
  Serial.println(" °C");
}
/****************************************************************************************************/
/* Mesure de l'humidité.                                                                            */
/****************************************************************************************************/
void MesureHumidity(void) {
  Serial.print(F("Humidité capteur Si7021 : "));
  Serial.print(CapteurSi7021.readHumidity(), 2);
  Serial.println(" % HR");
}
/****************************************************************************************************/
/* Initialisation de l'afficheur OLED.                                                              */
/****************************************************************************************************/
/*void initMyDisplay(void) {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!MyOled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {         // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);                                              // Don't proceed, loop forever
  }
  MyOled.clearDisplay();
  MyOled.setTextSize(3);
  MyOled.setTextColor(WHITE);
  MyOled.setCursor(0,0);
  //MyOled.drawPixel(120, 28, WHITE);     // useful to understand the position of pixels
  //MyOled.drawPixel(127, 31, WHITE); 
  MyOled.println("LoRaWAN");
  //MyOled.setFont(&FreeMonoOblique9pt7b);
  MyOled.setTextSize(1);
  MyOled.println(string1);
  MyOled.println(string2);
//  MyOled.setTextSize(2);                  // Draw 2X-scale text
//  MyOled.setTextColor(WHITE);
//  MyOled.setCursor(12, 0);
//  MyOled.println(F("scroll"));
  MyOled.display();                       // Show initial text
//  delay(100);
//  MyOled.startscrollright(0x00, 0x0F);   // Scroll in various directions, pausing in-between:
//  delay(2000);
//  MyOled.stopscroll();
  delay(500);
  MyOled.setFont();
}*/

/* ######################################################################################################## */
// END of file
