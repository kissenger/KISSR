/*=============================================================================
 * Useful references:
 * https://github.com/KevWal/ProMini/blob/master/ProMini.ino
 * https://github.com/daveake/FlexTrack/blob/master/gps.ino#L94-L111
 * http://ava.upuaut.net/?p=750
 * http://ava.upuaut.net/?p=627
 * https://ukhas.org.uk/projects:dl-fldigi
 * https://ukhatelem.org.uk/communication:protocol?s[]=checksum
 * https://web.archive.org/web/20150218055034/http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
 * http://tt7hab.blogspot.com/2014/09/the-new-tracking-solution.html
 *============================================================================= */

 /* Notes:
  * - Interrupts for LED heartbeat conflict with PWM control, needed for radio TX, so removed
  * - Intent is for no code to be blocking, in case of restart needed in flight
  * 
  * Approach:
  * - Setup sets pin modes, starts serial comms and pwm mode, but does not check or setup sensors
  * - Logic is that this is sone once per loop anyway, in case of loose connection that can be solved by consistently rechecking  
  * - Inside loop do the following:
  *   o Check sensors are present and setup correctly; if not then re initialise
  *   o Listen to GPS and determine if we have a fix, and if so capture the data
  *   o Formulate telemetry string
  *   o Transmit telemetry
  * - There are two modes:
  *   o Ground mode: no pause between loops (regular tx for ground testing); flash LED once per loop (heartbeat)
  *   o Flight mode: wait 30s between loops and no LED flash (save battery)
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
  #define PIN_SWITCH    11  
  
  #define DEBUG      1       // reports code position for detailed debugging
  #define DEBUG_NMEA 1       // echos recieved gps data

  #define GROUND 1
  #define FLIGHT 0
 
  SoftwareSerial      ss(PIN_SS_RX, PIN_SS_TX);
  TinyGPSPlus         gps;   

  bool switchStatus;
  uint32_t txInterval = 0;
  uint32_t lastTxTime = 0;
  
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
  pinMode(PIN_SWITCH,        INPUT);   // 
  
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

//--------------------------------------------------------------------------------------------------------
// LOOP
// 
//--------------------------------------------------------------------------------------------------------


void loop(){
  
  #if (DEBUG) 
    ss.println(F("&&loop()"));
  #endif
  
  uint32_t loopTime = millis();
  wdt_reset();

  switchStatus = digitalRead(PIN_SWITCH);
  if (switchStatus == GROUND) {
    ss.println("Ground Mode");
    txInterval = 0;
    digitalWrite(PIN_LED, HIGH);
  } else if (switchStatus == FLIGHT) {
    ss.println("Flight Mode");    
    txInterval = 30000;
    digitalWrite(PIN_LED, LOW);    
  }

  if (loopTime - lastTxTime > txInterval) {
    ss.println(loopTime);
    ss.println(lastTxTime);
    ss.println(loopTime - lastTxTime);
    ss.println(txInterval);

    checkSensorStatus(errOpenlog, errBMP180);
    
    gpsWake();    // note 500ms delay in function 
    errGPSModes = gpsSetModes();
    getGPSData(gps, errGPSData, errGPSFix);
    

    if (!errGPSFix && !errGPSData && switchStatus == FLIGHT) {  
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
  
    snprintf(telemString,sizeof(telemString),"$$KISSR;%u;%lu;%s;%s;%lu;%hu;%i;%i;%lu;%hu;", 
      uint16_t(telem.id++),                   // %u   uint16_t
      uint32_t(gps.time.value()),             // %lu  n/a (tinypgs++)
      telem.latStr,                           // %s   char 
      telem.lngStr,                           // %s   char
      uint32_t(gps.altitude.meters()),        // %lu  n/a (tinypgs++)
      uint8_t(gps.satellites.value()),        // %hu  n/a (tinypgs++)
      int16_t(telem.tempInt),                 // %i   int16_t
      int16_t(telem.tempExt),                 // %i   int16_t
      uint32_t(telem.pressure),               // %lu  uint32_t
      uint8_t(telem.err));                    // %hu  uint8_t
  
    snprintf(checksum, sizeof(checksum), "*%04X\n", getChecksum(telemString));
    memcpy(telemString + strlen(telemString), checksum, strlen(checksum) + 1);
    ss.println(telemString);    // send to s/w serial port
    
  //--------------------------------------------------------------------------------------------------------
  // TX telemetry string
  // Approx 15secs - can only shorten by increasing baud rate or shortening telemString
  // Could consider interrupts for TX, but not sure there is a benefit
  //--------------------------------------------------------------------------------------------------------
  
    enableTx();
    rttyTxString(telemString); 
    disableTx();
    lastTxTime = loopTime;

    if (switchStatus == GROUND) {
      flashLED();
    }
  
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

void checkSensorStatus(bool & errL, bool & errB) {

  if (errL) {
    errL = !openlogInit(); 
  }

  if (errB) {
    errB = !bmpPing();
    if (!errB) {
      telem.refPressure = bmpRefPressure();
    }
  }
  
}
