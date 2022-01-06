/*=============================================================================
 * Useful references:
 * https://github.com/KevWal/ProMini/blob/master/ProMini.ino
 * https://github.com/daveake/FlexTrack/blob/master/gps.ino#L94-L111
 * http://ava.upuaut.net/?p=750
 * http://ava.upuaut.net/?p=627
 * https://ukhas.org.uk/projects:dl-fldigi
 * https://ukhatelem.org.uk/communication:protocol?s[]=checksum
 * https://web.archive.org/web/20150218055034/http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
 *============================================================================= */

 /* Notes:
  * - Interrupts for LED heartbeat conflict with PWM control, needed for radio TX
  * - Intent is for no code to be blocking, in case of restart needed in flight
  */

  #include <I2C.h>                  // lightweight alternative to Wire library https://github.com/DSSCircuits/I2C-Master-Library
  #include <OneWire.h>               
  #include <DallasTemperature.h>  
  #include <TinyGPS++.h>            // https://github.com/mikalhart/TinyGPSPlus/releases
  #include <SoftwareSerial.h>  
  #include <util/crc16.h>
  #include <string.h>
  #include <avr/wdt.h>              // watchdog setup
  #include <avr/interrupt.h>        // interrupt timer

  #define PIN_LED       12     // status LED
  #define PIN_SS_RX     6
  #define PIN_SS_TX     7
  #define PIN_TEMP      A0
  #define PIN_OpL_RESET 8
  #define PIN_RADIO_TX  9
  #define PIN_RADIO_EN  10  
  
  #define DEBUG      1       // reports code position for detailed debugging
  #define DEBUG_NMEA 1       // echos recieved gps data

/*  ------------------------+-------------+-----------------+
 *  Variable Type           |  Min        |    Max          |
 *  ------------------------+-------------+-----------------+
 *  byte, int8_t, bool      | -256        |   +255          |
 *  uint8_t                 |  0          |   +511          |  
 *  int, int16_t            | -32768      |   +32767        |
 *  unsigned int, uint16_t  |  0          |   +65535        |
 *  long, int32_t           | -2147483648 |   +2147483647   |
 *  unsigned long, uint32_t |  0          |   +4294967295   | 
 *  ------------------------+-------------+-----------------+ */
 
  SoftwareSerial      ss(PIN_SS_RX, PIN_SS_TX);
  TinyGPSPlus         gps;   

  bool isInFlight = false;
  bool hasFirstFixBeenHad = false;

  bool errOpenlog = true;
  bool errBMP180 = true;
  bool errGPSData = true;
  
  bool errGPSFix = true;
  bool errGPSModes = true;

  const uint16_t GPS_DATA_TIMEOUT  = 5000;
  const uint16_t GPS_FIX_TIMEOUT = 5000; 
  const uint16_t I2C_TIMEOUT = 3000;               //timeout for i2c bus - should be <<8s otherwise watchdog timer will cause reset
  const uint16_t REF_PRESS_SAMPLE_PERIOD = 1000;
  
  char telemString[110];
  char checksum[10];  
  typedef struct {
    uint8_t  err;              
    int16_t  tempExt, tempInt;
    uint16_t id = 0;
    uint32_t pressure, refPressure;
    char latStr[12], lngStr[12];
  } data;
  data telem; // instantiate above struct
        
//--------------------------------------------------------------------------------------------------------
// SETUP
// Nothing should be blocking in seteup in case unit resets in flight
// Where relevant, failed checks are flagged and attempted again in loop()
//--------------------------------------------------------------------------------------------------------

void setup(){

//  wdt_enable(WDTO_8S);

  pinMode(PIN_LED,           OUTPUT);   // set LED pin
  pinMode(PIN_OpL_RESET,     OUTPUT);   // used to reset openlog to confirm ready status
  pinMode(PIN_RADIO_TX,      OUTPUT);   // set radio output pin
  pinMode(PIN_RADIO_EN,      OUTPUT);   // set radio output pin 
  pinMode(PIN_SS_RX,         INPUT);    //  
  pinMode(PIN_SS_TX,         OUTPUT);   // 

  digitalWrite(PIN_RADIO_EN, LOW);  
  digitalWrite(PIN_LED, HIGH);          // solid LED during setup

  Serial.begin(9600);                 // start harware serial for GPS comms
  I2c.begin();              
  I2c.timeOut(I2C_TIMEOUT);
  ss.begin(19200);
  
  gpsSetNMEA();
  
 //Set PWM frequency of pin 9 - this will be different if pin changes; used for NTX2B radio comms
 //see http://playground.arduino.cc/Main/TimerPWMCheatshee and https://www.arduino.cc/en/Tutorial/PWM
   TCCR1B = TCCR1B & 0b11111000 | 0x01;
  
  //Send headers to datafile
  snprintf(telemString,sizeof(telemString),"$$KISSR;Pref=%lu;", uint32_t(telem.refPressure));
  ss.println(telemString);
  ss.println("$$KISSR;sentenceId;time;Lat;Lng;gpsAltitude;nSatellites;Fix;Tint;TExt;Pbmp;VCC;errorCode;checksum");

}



void loop(){

//--------------------------------------------------------------------------------------------------------
// Preliminaries
// <10ms providing there are no errors
// Up to 3secs if openlog needs init
//--------------------------------------------------------------------------------------------------------

//  wdt_reset();  // check in with watchdog
  
  #if (DEBUG) 
    ss.println(F("&&loop()"));
  #endif

  if (errOpenlog) {
    errOpenlog = !openlogInit(); 
  }

  if (errBMP180) {
    errBMP180 = !bmpPing();
    if (!errBMP180) {
      telem.refPressure = bmpRefPressure();
    }
  }


  
//--------------------------------------------------------------------------------------------------------
// NMEA Loop
// Approx 5secs when there is no fix, driven by GPS_FIX_TIMEOUT
//--------------------------------------------------------------------------------------------------------

  gpsWake();    // note 500ms delay in function 
  errGPSModes = gpsSetModes();
  getGPSData(gps, gpsFixQuality, errGPSData, errGPSFix);

  if (!errGPSFix) {
    hasFirstFixBeenHad = true;
    digitalWrite(PIN_LED, LOW); 
  }

  if (!errGPSFix && !errGPSData) {  
    gpsSleep();
  }


  
//--------------------------------------------------------------------------------------------------------
// Collect data from sensors and for telemetry string
// Approx 300ms
//--------------------------------------------------------------------------------------------------------

  dtostrf(gps.location.lat(),8,6,telem.latStr);   // Convert lat long into string format
  dtostrf(gps.location.lng(),8,6,telem.lngStr);

  telem.tempInt = ((float)analogRead(PIN_TEMP) * 3.3 / 1024.0 - 0.5) * 1000.0;
  telem.tempExt = bmpTemperature();
  telem.pressure = bmpPressure();  
  telem.err = errorByte(errOpenlog, errBMP180, errGPSData, errGPSFix, errGPSModes);

  snprintf(telemString,sizeof(telemString),"$$KISSR;%u;%lu;%s;%s;%lu;%hu;%i;%i;%lu;%lu;%hu;", 
    uint16_t(telem.id++),                   // %u   uint16_t
    uint32_t(gps.time.value()),             // %lu  n/a (tinypgs++)
    telem.latStr,                           // %s   char 
    telem.lngStr,                           // %s   char
    uint32_t(gps.altitude.meters()),        // %lu  n/a (tinypgs++)
    uint8_t(gps.satellites.value()),        // %hu  n/a (tinypgs++)
    int16_t(telem.tempInt),                 // %i   int16_t
    int16_t(telem.tempExt),                 // %i   int16_t
    uint32_t(telem.pressure),               // %lu  uint32_t
    uint32_t(readVcc()),                    // %lu  uint32_t
    uint8_t(telem.err));                    // %hu  uint8_t

  snprintf(checksum, sizeof(checksum), "*%04X\n", getChecksum(telemString));
  memcpy(telemString + strlen(telemString), checksum, strlen(checksum) + 1);
  ss.println(telemString);    // send to s/w serial port
  
//--------------------------------------------------------------------------------------------------------
// TX telemetry string
// Approx 15secs - can only shorten by increasing baud rate or shortening telemString
// Could consider interrupts for TX, but not sure there is a benefit
//--------------------------------------------------------------------------------------------------------

  if (hasFirstFixBeenHad) {   // don't bother with tranmission until we've had a fix (but carry on if its then lost)
    digitalWrite(PIN_RADIO_EN, HIGH);
    delay(500); // confirm the need for this delay
    rttyTxString(telemString); 
    digitalWrite(PIN_RADIO_EN, LOW);
  }
  
  flashLED();
  if (telem.err) {  
    delay(200);
    flashLED();
  }
}

void flashLED() {
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  delay(100);
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}


uint8_t errorByte(bool errOPL, bool errBMP, bool errDATA, bool errFIX, bool errMODES) {
  
  /*   |  bit 6  |  bit 5  |  bit 4   |   bit 3  |   bit 2  |  bit 1  |  bit 0   | 
   *   |   n/a   |   n/a   | gpsModes | gpsFix   | gpsData  | bmp180  | openlog  |*/

  return 0 << 7 | 0 << 6 | 0 << 5 | errMODES << 4 | errFIX << 3 | errDATA << 2 | errBMP << 1 | errOPL;

}
