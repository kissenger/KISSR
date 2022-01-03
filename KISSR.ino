/*=============================================================================
 * Useful references:
 * https://github.com/KevWal/ProMini/blob/master/ProMini.ino
 * https://github.com/daveake/FlexTrack/blob/master/gps.ino#L94-L111
 * http://ava.upuaut.net/?p=750
 * http://ava.upuaut.net/?p=627
 * https://ukhatelem.org.uk/communication:protocol?s[]=checksum
 * https://web.archive.org/web/20150218055034/http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
 *============================================================================= */

  #include <I2C.h>                  // lightweight alternative to Wire library https://github.com/DSSCircuits/I2C-Master-Library
  #include <OneWire.h>               
  #include <DallasTemperature.h>  
  #include <TinyGPS++.h>            // https://github.com/mikalhart/TinyGPSPlus/releases
  #include <SoftwareSerial.h>  
  #include <util/crc16.h>
  #include <string.h>
  #include <avr/wdt.h>              // watchdog setup
  #include <avr/interrupt.h>        // interrupt timer
//  #include <LowPower.h>             // low power sleep modes

  #define PIN_LED       12     // status LED
  #define PIN_SS_RX     6
  #define PIN_SS_TX     7
  #define PIN_TEMPS     11
  #define PIN_OpL_RESET 8
  #define PIN_RADIO_TX  9
  #define PIN_RADIO_EN  10  

  #define DEBUG      1       // reports code position for detailed debugging
  #define DEBUG_NMEA 1       // echos recieved gps data
  
  #define SLOW_BLINK 1       // semantic definition of led modes
  #define FAST_BLINK 2
  #define IN_FLIGHT  3

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
  TinyGPSCustom       GSA_fix(gps, "GPGSA", 2);       // create custom gps telem (fix quality)   
  OneWire             ds(PIN_TEMPS);
  DallasTemperature   DS18B20(&ds);               
  DeviceAddress       addrText = {0x28, 0x96, 0x04, 0x5F, 0x07, 0x00, 0x00, 0x3A};  
  DeviceAddress       addrTint = {0x28, 0xD1, 0xBC, 0x75, 0x04, 0x00, 0x00, 0x34}; 
  
  // check gps status
  bool gpsStatusPSM = false;
  bool gpsStatusFM = false;
  bool gpsStatusGNSS = false;  
  bool gpsStatusSave = false;
  
  bool isInFlight = false;
  uint8_t ledMode;

  bool errOpenlog = true;
  bool errBMP180 = true;
  bool errGPSData = true;
  bool errGPSFix = true;

  const uint16_t GPS_DATA_TIMEOUT  = 5000;
  const uint16_t GPS_FIX_TIMEOUT = 5000; 
  const uint16_t I2C_TIMEOUT = 3000;               //timeout for i2c bus - should be <<8s otherwise watchdog timer will cause reset
  const uint16_t REF_PRESS_SAMPLE_PERIOD = 1000;
  
  char telemString[110];
  char checksum[10];  
  typedef struct {
    uint8_t  err;              
    int16_t  tempExt, tempInt, tempBMP;
    uint16_t id = 0;
    uint32_t pressure, refPressure;
    char latStr[12], lngStr[12];
  } data;
  data telem; // instantiate above struct
        

// SETUP
// Nothing should be blocking in seteup in case unit resets in flight
// Where relevant, failed checks are flagged and attempted again in loop()

void setup(){
  
  // Set watchdog to 8s delay - unit will reset if not reset within 8s
  // Safety feature in case device hangs in flight.
//  wdt_enable(WDTO_8S);

    //Set pin modes
  pinMode(PIN_LED,           OUTPUT);   // set LED pin
  pinMode(PIN_OpL_RESET,     OUTPUT);   // used to reset openlog to confirm ready status
  pinMode(PIN_RADIO_TX,      OUTPUT);   // set radio output pin
  pinMode(PIN_RADIO_EN,      OUTPUT);   // set radio output pin 
  pinMode(PIN_SS_RX,         INPUT);    //  
  pinMode(PIN_SS_TX,         OUTPUT);   // 

  // Initialise some pins
  digitalWrite(PIN_RADIO_EN, LOW);  
  digitalWrite(PIN_LED, HIGH);          // solid LED during setup
  
  // Begin things
  Serial.begin(9600);                 // start harware serial for GPS comms
  I2c.begin();              
  I2c.timeOut(I2C_TIMEOUT);
  ss.begin(19200);
  
  delay(100);  // settling period
  
  ss.println(F("&&setup()"));

  errOpenlog = !openlogInit();
  errBMP180 = !bmpPing();
  if (!errBMP180) {
    telem.refPressure = bmpRefPressure();
  }

  gpsStatusFM = gpsSetFM();
  gpsStatusGNSS = gpsSetGNSS(); 
  gpsStatusPSM = gpsSetPSM();
  gpsStatusSave = gpsSaveSettings();  
  gpsSetNMEA();
  
 //Set PWM frequency of pin 9 - this will be different if pin changes; used for NTX2B radio comms
 //see http://playground.arduino.cc/Main/TimerPWMCheatshee and https://www.arduino.cc/en/Tutorial/PWM
   TCCR1B = TCCR1B & 0b11111000 | 0x01;
  
  //Send headers to datafile
  snprintf(telemString,sizeof(telemString),"$$KISSR;Pref=%lu;", uint32_t(telem.refPressure));
  ss.println(telemString);
  ss.println("$$KISSR;sentenceId;time;Lat;Lng;gpsAltitude;nSatellites;Fix;Text;Tint;Tbmp;Pbmp;VCC;errorCode;checksum");

  // Initialize Timer1 interrupts to control LED
  interruptInit();
  ledMode = setLedMode(FAST_BLINK);

  ss.print(F("Setup complete with error code: "));
  ss.println(errorByte(errOpenlog, errBMP180, 0, 0), BIN);
}



//===============================================================================
// 
//                                   LOOP 
//
//===============================================================================

void loop(){

//---------------------------------------------
// Preliminaries
//---------------------------------------------

  wdt_reset();  // check in with watchdog
  
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
  
  gpsWake();    // note 500ms delay in function

//--------------------------------------------------------------------------------------------------------
//Loop until we get a full GPS sentence - dont move on unless there is an error or we have a full sentence
//--------------------------------------------------------------------------------------------------------
  
  #if (DEBUG) 
    ss.println(F("&&NMEA loop"));
  #endif 

  uint32_t now = millis();
  uint32_t gpsDataTime = now;
  
  while (1) {                                   //keep trying until we get readable data or timeout

    // if we have serial data then check for coherent sentence
    if (Serial.available()) {

      char c = Serial.read();
      errGPSData = false;
      gpsDataTime = millis();           
      
      #if (DEBUG_NMEA)
        ss.write(c);
      #endif
      
      //see if we got a complete, readable sentence
      if (gps.encode(c)) {
      
        // hang around and wait for solid 3D fix with fresh data
        if (strcmp(GSA_fix.value(), "3") == 0 && gps.satellites.value() >= 5 && gps.location.age() < 2000) {
          errGPSFix = false;
          break;
        }

        // if it doesn't come then move on
        if (millis() - now > GPS_FIX_TIMEOUT) {
          errGPSFix = true;
          break;  // waited too long for a fix, move on
        }
      }
    }

    // No serial data recieved in this loop
    // If we've been waiting too long then set errors and move on
    else {
      if (millis() - gpsDataTime > GPS_DATA_TIMEOUT) {  
        errGPSFix = true;
        errGPSData = true;  
        break;    
      }   
    }
  }

//-----------------------------------------------------------------
//once we are out of the NMEA loop, we can turn off the gps module
//-----------------------------------------------------------------
  //go to sleep only if we have a good fix, recieving data ok
  if (!errGPSFix && !errGPSData) {  
    
    #if (DEBUG) 
      ss.println(F("&&Attempt to sleep"));
    #endif 

    if (!gpsStatusFM) {
      gpsStatusFM = gpsSetFM(); 
    }
    
    if (!gpsStatusGNSS) {
      gpsStatusGNSS = gpsSetGNSS();      
    }

    if (!gpsStatusPSM) {
      gpsStatusFM = gpsSetFM();
    }
    
    if (gpsStatusPSM) {
      gpsSleep();
    }
    
  }

//-------------------------------------------------------------------------------
//Collect data from sensors, form telemetry string, print to serial and transmit
//-------------------------------------------------------------------------------
  //Convert lat long into string format
  dtostrf(gps.location.lat(),8,6,telem.latStr);
  dtostrf(gps.location.lng(),8,6,telem.lngStr);
  
  //Get temperatures - note that temperatures are multiplied by 10 to store as integer
  DS18B20.requestTemperatures();
  telem.tempExt = DS18B20.getTempC(addrText)*10;   
  telem.tempInt = DS18B20.getTempC(addrTint)*10; 

  //Get pressure
  telem.pressure = bmpPressure();
  telem.tempBMP = bmpTemperature();

  //Construct error byte
  telem.err = errorByte(errOpenlog, errBMP180, errGPSData, errGPSFix);

  //Generate sentences
  snprintf(telemString,sizeof(telemString),"$$KISSR;%u;%lu;%s;%s;%lu;%hu;%hu;%i;%i;%i;%lu;%lu;%hu;", 
    uint16_t(telem.id++),                   // %u   uint16_t
    uint32_t(gps.time.value()),             // %lu  n/a (tinypgs++)
    telem.latStr,                           // %s   char 
    telem.lngStr,                           // %s   char
    uint32_t(gps.altitude.meters()),        // %lu  n/a (tinypgs++)
    uint8_t(gps.satellites.value()),        // %hu  n/a (tinypgs++)
    uint8_t(GSA_fix.value()),               // %hu  n/a (tinypgs++)
    int16_t(telem.tempExt),                 // %i   int16_t
    int16_t(telem.tempInt),                 // %i   int16_t
    int16_t(telem.tempBMP),                 // %i   int16_t
    uint32_t(telem.pressure),               // %lu  uint32_t
    uint32_t(readVcc()),                    // %lu  uint32_t
    uint8_t(telem.err));                    // %hu  uint8_t

  snprintf(checksum, sizeof(checksum), "*%04X\n", getChecksum(telemString));
  memcpy(telemString + strlen(telemString), checksum, strlen(checksum) + 1);
  ss.println(telemString);    // send to s/w serial port

//  // send to radio for transmission, use enable pin to turn on and off
//  digitalWrite(PIN_RADIO_EN, HIGH);
//  rtty_txstring(telem_string); 
//  digitalWrite(PIN_RADIO_EN, LOW);
//
//  ss.println()

  #if (DEBUG) 
    ss.println();
    ss.print(F("Error code: "));
    ss.println(errorByte(errOpenlog, errBMP180, errGPSData, errGPSFix), BIN);
  #endif 

  // LED state
  // SOLID_ON - in setup()
  // FAST_BLINK - setup() complete, errorCode <> 0 (eg, no GPS fix)
  // SLOW BLINK - setup() complete, errorCode == 0 
  // When flight mode is triggered, LED is off

  if (isInFlight) {
    ledMode = setLedMode(IN_FLIGHT);
  } else {
    if (!errorByte(errOpenlog, errBMP180, errGPSData, errGPSFix)) {
      ledMode = setLedMode(SLOW_BLINK);
    } else {
      if (ledMode == SLOW_BLINK) {  // dont want to set variables quickly as might affect interrupt??
        ledMode = setLedMode(FAST_BLINK);
      }
    }    
  }

}


uint8_t errorByte(bool errOPL, bool errBMP, bool errDATA, bool errFIX) {
  
  /*   |  bit 6  |  bit 5  |  bit 4  |   bit 3  |   bit 2  |  bit 1  |  bit 0   | 
   *   |   n/a   |   n/a   | n/a     | gpsFix   | gpsData  | bmp180  | openlog  |*/

  return 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | errFIX << 3 | errDATA << 2 | errBMP << 1 | errOPL;

}
