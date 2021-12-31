
  #define      SOFT_VERSION F("KISSR_014")  

/*
 *=============================================================================
 * Credit where credit is due:
 * https://github.com/KevWal/ProMini/blob/master/ProMini.ino
 * https://github.com/daveake/FlexTrack/blob/master/gps.ino#L94-L111
 * http://ava.upuaut.net/?p=750
 * http://ava.upuaut.net/?p=627
 * https://ukhatelem.org.uk/communication:protocol?s[]=checksum
 * https://web.archive.org/web/20150218055034/http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
 *=============================================================================  
 * Work List:
 * expand comments - ensure comment against all includes and that theyre all necessary
 * can watchdog check-ins be simplified?
 * implment status pin - LED off if no error, but flash once per loop for heartbeat?
 * !ERR03 error logging in openlog_init sub
 * !ERR04 catch i2c error not working
 * !ERR05 use interrupt to blink LED rather than sleep
 * !EER06 disable GPS LED?
 *============================================================================= */

//-----------------------------------------------------------------------------
//Includes
//-----------------------------------------------------------------------------

  #include     <I2C.h>                  // lightweight alternative to Wire library https://github.com/DSSCircuits/I2C-Master-Library
  #include     <OneWire.h>               
  #include     <DallasTemperature.h>  
  #include     <TinyGPS++.h>            // https://github.com/mikalhart/TinyGPSPlus/releases
  #include     <SoftwareSerial.h>  
  #include     <util/crc16.h>
  #include     <string.h>
  #include     <avr/wdt.h>              // watchdog setup
  #include     <avr/interrupt.h>        // interrupt timer
  #include     <LowPower.h>             // low power sleep modes
  
//-----------------------------------------------------------------------------
//Define pin modes
//-----------------------------------------------------------------------------
  #define      PIN_LED       3     // status LED
  #define      PIN_SS_RX     6
  #define      PIN_SS_TX     7
  #define      PIN_TEMPS     11
  #define      PIN_OpL_RESET 8
  #define      PIN_RADIO_TX  9
  #define      PIN_RADIO_EN  10  

//-----------------------------------------------------------------------------
//Define error codes
//-----------------------------------------------------------------------------
/*   |  bit 6  |  bit 5  |  bit 4  |   bit 3  |   bit 2  |  bit 1  |  bit 0   | 
 *   |   n/a   |   n/a   |   n/a   | i2c_data | opl_init | gps_fix | gps_data |
  -----------------------------------------------------------------------------*/
  uint8_t err_gps_data = 1;         //no GPS data recieved over serial buffer
  uint8_t err_gps_fix  = 1;         //no GPS fix
  uint8_t err_opl_init = 1;         //openlog card not present
  uint8_t err_i2c_data = 1;         //no data from i2c bus - not connected?

//-----------------------------------------------------------------------------
//Debug turns on (1) or off (0) a bunch of debug statements. Normally use (0)
//-----------------------------------------------------------------------------
  #define DEBUG          1       // reports code position for detailed debugging
  #define DEBUG_NMEA     0       // echos recieved gps data

//-----------------------------------------------------------------------------
//Initialise objects
//-----------------------------------------------------------------------------
  SoftwareSerial      ss(PIN_SS_RX, PIN_SS_TX);       // setup software serial (RX,TX)
  TinyGPSPlus         gps;                            // create a gps instance  
  TinyGPSCustom       GSA_fix(gps, "GPGSA", 2);       // create custom gps telem (fix quality)   
  OneWire             ds(PIN_TEMPS);                  // ds18b20 temperature module
  DallasTemperature   DS18B20(&ds);               
  DeviceAddress       addrText = {0x28, 0x96, 0x04, 0x5F, 0x07, 0x00, 0x00, 0x3A};  
  DeviceAddress       addrTint = {0x28, 0xD1, 0xBC, 0x75, 0x04, 0x00, 0x00, 0x34}; 

//-----------------------------------------------------------------------------
//Initialise Variables
//-----------------------------------------------------------------------------

/*  ------------------------+-------------+-----------------+
 *  Variable Type           |  Min        |    Max          |
 *  ------------------------+-------------+-----------------+
 *  byte, int8_t            | -256        |   +255          |
 *  uint8_t                 |  0          |   +511          |  
 *  int, int16_t            | -32768      |   +32767        |
 *  unsigned int, uint16_t  |  0          |   +65535        |
 *  long, int32_t           | -2147483648 |   +2147483647   |
 *  unsigned long, uint32_t |  0          |   +4294967295   | 
 *  ------------------------+-------------+-----------------+ */
 
  // these are used to manage timeouts and sample interval
  uint32_t loop_time         = 0;          //will roll over after approx 49 days, no need to handle this
  uint32_t gps_data_time     = 0;
  uint16_t gps_data_timeout  = 5000;       //if weve been waiting this long for gps data then timeout
  uint16_t gps_fix_timeout   = 5000;       //if weve been waiting this long for gps fix then timeout  
  uint16_t opl_init_timeout  = 3000;       //delay waiting for openlog to send chars
  uint16_t i2c_timeout       = 3000;       //timeout for i2c bus - should be <<8s otherwise watchdog timer will cause reset
  uint16_t sample_period     = 0;          //in millisecs, 0=fast as possible
  int8_t   n_loops;                        //number of loops reqd to generate required delay
  uint16_t ref_press_sample_time = 1000;   // sample period when finding reference pressure

  // used to report on gps settings
  uint8_t PSM_set_success  = 0;
  uint8_t GNSS_set_success = 0;
  uint8_t FM_set_success   = 0;
  uint8_t CFG_save_success = 0;
  
  // these used to handle print and checksum data
  char telem_string[110];
  char checksum[10];  

  // defines all the variables used by sensors
  typedef struct {
    volatile uint8_t  err;               // error code is short int unsigned 
//  uint8_t  acc[3];                     // acceleration *1000 (ie 1000=1.000)         - ADXL not currently used
    int16_t  t_ext, t_int, bmp_tmp;      // signed int - temp could be negative and is stored as *10 so cannot be a short (might expect temps <-25deg, which would be stored as -250)
    uint16_t id=0;                       // id is unsigned int - no problem if it roles over
    uint32_t bmp_prs, bmp_prs_ref;       // pressure is a long int
    char latstr[12], lngstr[12];         // char strings to hold lat and long
  } data;
  data telem; // instantiate above struct
    
//===============================================================================
// 
//                                    INTERRUPTS 
//
//===============================================================================

volatile boolean led_state;
//volatile uint16_t led_blink_timer = 3000;         // led is on/off for 0.1s
volatile uint32_t led_blink_period = 20000;       // blink every 2s

ISR(TIMER1_COMPA_vect) {
  
  cli();          // disable global interrupts   

//  if (telem.err) {       //there is an error so blink off
    if (led_state) {     //led is on so needs to be blinked off

      digitalWrite(PIN_LED, LOW);  //turn off led
      led_state = false;

      TCCR1A = 0; 
      TCCR1B = 0;                        // stop timer
      TCNT1 = 0;                         // reset counter
      TCCR1B = bit(WGM12) | bit(CS11);   // CTC, scale to clock / 8 (presaler set to 8)
      OCR1A = (led_blink_period * 2) - 1; // timer max value
      TIMSK1 = bit(OCIE1A);
      
    }
    else {               //led is off so needs to be turned back on
      
      digitalWrite(PIN_LED, HIGH);  //turn off led
      led_state = true;
      
      TCCR1A = 0; 
      TCCR1B = 0;                        // stop timer
      TCNT1 = 0;                         // reset counter
      TCCR1B = bit(WGM12) | bit(CS11);   // CTC, scale to clock / 8 (presaler set to 8)
      OCR1A = (led_blink_period * 2) - 1;
      TIMSK1 = bit(OCIE1A);  
   
    }
//  }

  sei();          // enable global interrupts

}
    
//===============================================================================
// 
//                                    SETUP 
//
//===============================================================================

void setup(){
  
  //set watchdog to 8s delay
  wdt_enable(WDTO_8S);
  #if (DEBUG)
    ss.println(F("&&setup()"));
  #endif 
  
  //Begin things
  Serial.begin(9600);                 // start harware serial for GPS comms
  ss.begin(19200);                    // start software serial for debugging and logging
  I2c.begin();                        // for communicating with i2c peripherals
  I2c.timeOut(i2c_timeout);
  
  //Identify software version
  ss.println();
  ss.print(F("&&------ "));
  ss.print(SOFT_VERSION);
  ss.println(F(" by Gordon ------ "));

  //Set pin modes
  pinMode(PIN_LED,           OUTPUT);   // set LED pin
  pinMode(PIN_OpL_RESET,     OUTPUT);   // used to reset openlog to confirm ready status
  pinMode(PIN_RADIO_TX,      OUTPUT);   // set radio output pin
  pinMode(PIN_RADIO_EN,      OUTPUT);   // set radio output pin 
  pinMode(PIN_SS_RX,         INPUT);    //  
  pinMode(PIN_SS_TX,         OUTPUT);   //  

  //Initialise some pins
  digitalWrite(PIN_RADIO_EN, LOW);  

  //Call initialisation functions
  telem.bmp_prs_ref = getBMP_press_ref();     // record ref pressure - incurs delay of (ref_press_sample_time) ms
  setGPS_flight_mode();
  setGPS_GNSS_mode();  
  setGPS_save_settings();
  setGPS_power_save();
  
 //Set PWM frequency of pin 9 - this will be different if pin changes; used for NTX2B radio comms
 //see http://playground.arduino.cc/Main/TimerPWMCheatshee and https://www.arduino.cc/en/Tutorial/PWM
   TCCR1B = TCCR1B & 0b11111000 | 0x01;
  
  //Send headers to datafile
  snprintf(telem_string,sizeof(telem_string),"$$KISSR;ref_pressure=;%lu;", uint32_t(telem.bmp_prs_ref));
  ss.println(telem_string);
  ss.println("$$KISSR;sentence_id;time;latitude;longitude;altitude;satellites;fix;temperature_external;temperature_internal;temp_bmp;pressure_bmp;vcc_volts;error_code;checksum");



  // initialize Timer1
  cli();          // disable global interrupts  
  TCCR1A = 0;
  TCCR1B = 0;                        // stop timer
  TCNT1 = 0;                         // reset counter
  TCCR1B = bit(WGM12) | bit(CS11);   // CTC, scale to clock / 8 (presaler set to 8)
  OCR1A = (led_blink_period * 2) - 1;
  TIMSK1 = bit(OCIE1A); 
  sei();          // enable global interrupts
  
  digitalWrite(PIN_LED, HIGH);   
  led_state = true;
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

  if (err_opl_init) initOpenLog();      // if there is an error flag, try to initiate again

  //sleep until sample_period is up
  //sleeps for 2secs at a time, so will tend to be up to 2secs early .. its good enough
  //Sleep options are: SLEEP_15MS,SLEEP_30MS,SLEEP_60MS,SLEEP_120MS,SLEEP_250MS,SLEEP_500MS,SLEEP_1S,SLEEP_2S,SLEEP_4S,SLEEP_8S
/*
  if (loop_time > 5000) {
    n_loops = (sample_period - millis() + loop_time) / 2000;
    for (uint8_t i = 1; i <= n_loops; i++) {
      #if (DEBUG)
        ss.print(F("&&Wait loop:")); ss.print(i); ss.print(F("/")); ss.println(n_loops);
      #endif
      LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
    }
  } */

//record loop time, used for error trapping and sample timing
  loop_time = millis();

//wake up the GPS unit - note subroutine includes 500ms delay
  setGPS_wake();

//--------------------------------------------------------------------------------------------------------
//Loop until we get a full GPS sentence - dont move on unless there is an error or we have a full sentence
//--------------------------------------------------------------------------------------------------------
  
  #if (DEBUG) 
    ss.println(F("&&NMEA loop"));
  #endif 

  gps_data_time = millis();    // reset timer
  
  while (true) {                                   //keep trying until we get readable data or timeout

    if (Serial.available()) {

      char c = Serial.read();                      // reading into a variable makes dubugging easier
      
      #if (DEBUG)
        if (err_gps_data) ss.println("&&err_gps_data = 0");
      #endif
      
      err_gps_data = 0;      // we have data! clear any error
      gps_data_time = millis();   // got data so reset timer
            
      #if (DEBUG_NMEA)
        ss.write(c);
      #endif
      
      //see if we got a complete, readable sentence
      if (gps.encode(c)) {
      
        // hang around and wait for solid 3D fix with fresh data - but if it doesnt come use timeout ...
        if (strcmp(GSA_fix.value(),"3") == 0 && gps.satellites.value() >= 5 && gps.location.age() < 2000) {
          
          #if (DEBUG)
            if (err_gps_fix) ss.println(F("&&err_gps_fix = 0"));
          #endif
          
          err_gps_fix = 0;
                    
          break;   // got a fix, move on
        }
        else if (millis() - loop_time > gps_fix_timeout) {
          
          #if (DEBUG)
            if (!err_gps_fix) ss.println(F("&&err_gps_fix = 1"));
          #endif

          err_gps_fix = 1;
          
          break;  // waited too long for a fix, move on
        }
      }
    }

    else {         // no data available at the moment

      if (millis() - gps_data_time > gps_data_timeout) {  // if we have timed out 
  
        #if (DEBUG)
          if (!err_gps_data) ss.println(F("&&err_gps_data = 1"));  
        #endif 
        
        err_gps_data = 1;  
  
        break;    // move on
      }   
    }
  }

//-----------------------------------------------------------------
//once we are out of the NMEA loop, we can turn off the gps module
//-----------------------------------------------------------------
  //go to sleep only if we have a good fix, recieving data ok, and not about to time out
  if (!err_gps_fix && !err_gps_data && millis() - loop_time < sample_period - 1000) {  
    
    #if (DEBUG) 
      ss.println(F("&&Attempt to sleep"));
    #endif 
    
    if (!FM_set_success) setGPS_flight_mode(); //if flight mode isnt set, then try again
    
    if (!GNSS_set_success) {
      
      setGPS_GNSS_mode();                          //if GNSS mode is not set, try again
      if (!GNSS_set_success) setGPS_power_save();  //if GNSS is now set, try power save mode
      
    }
    
    if (PSM_set_success) setGPS_sleep();       //if that works, send it to sleep
    
  }

//-------------------------------------------------------------------------------
//Collect data from sensors, form telemetry string, print to serial and transmit
//-------------------------------------------------------------------------------
  //Convert lat long into string format
  dtostrf(gps.location.lat(),8,6,telem.latstr);
  dtostrf(gps.location.lng(),8,6,telem.lngstr);
  
  //Get temperatures - note that temperatures are multiplied by 10 to store as integer
  DS18B20.requestTemperatures();
  telem.t_ext = DS18B20.getTempC(addrText)*10;   
  telem.t_int = DS18B20.getTempC(addrTint)*10; 

  //Get pressure
  telem.bmp_prs = getBMP_press();
  telem.bmp_tmp = getBMP_temp();

  //Get accelerations - note multiplied by 1000 to store as integer
/*  getADXL(adxl); 
  for (int i = 0; i <= 2; i++) {  
    telem.acc[i] = (adxl[i] * 0.0039 * ADXL_CALIB[i] + ACCL_OFFSET[i]) * 1000; //accl *1000
  }
*/
  //Construct error byte
  telem.err = err_opl_init << 2 | err_gps_fix << 1 | err_gps_data ;

  //Generate sentences
// next line include accs which are no longer planned for flight
//  snprintf(telem_string,sizeof(telem_string),"$$KISSR;%i;%li;%s;%s;%li;%i;%i;%i;%i;%li;%05i;%05i;%05i;%li;%i;", int(telem.id++), long(gps.time.value()), telem.latstr, telem.lngstr, long(gps.altitude.meters()), int(gps.satellites.value()), int(GSA_fix.value()), int(telem.text), int(telem.tint), long(telem.prss), int(telem.acc[0]), int(telem.acc[1]), int(telem.acc[2]), long(readVcc()), int(telem.err));
  snprintf(telem_string,sizeof(telem_string),"$$KISSR;%u;%lu;%s;%s;%lu;%hu;%hu;%i;%i;%i;%lu;%lu;%hu;", uint16_t(telem.id++), uint32_t(gps.time.value()), telem.latstr, telem.lngstr, uint32_t(gps.altitude.meters()), uint8_t(gps.satellites.value()), uint8_t(GSA_fix.value()), int16_t(telem.t_ext), int16_t(telem.t_int), int16_t(telem.bmp_tmp), uint32_t(telem.bmp_prs), uint32_t(readVcc()), uint8_t(telem.err));
  
/* See http://www.cplusplus.com/reference/cstdio/printf/ for print codes
 * ------+-----------------------------------+-----------------------
 * format|     snprintf                      |    delacaration
 * ------+-----------------------------------+-----------------------
 * %u    |  uint16_t(telem.id++)             |     uint16_t
 * %lu   |  uint32_t(gps.time.value())       |     n/a (tinypgs++)
 * %s    |  telem.latstr                     |     char 
 * %s    |  telem.lngstr                     |     char
 * %lu   |  uint32_t(gps.altitude.meters())  |     n/a (tinypgs++)
 * %hu   |  uint8_t(gps.satellites.value())  |     n/a (tinypgs++)
 * %hu   |  uint8_t(GSA_fix.value())         |     n/a (tinypgs++)
 * %i    |  int16_t(telem.text)              |     int16_t
 * %i    |  int16_t(telem.tint)              |     int16_t
 * %i    |  int16_t(telem.t_prs)             |     int16_t
 * %lu   |  uint32_t(telem.prss)             |     uint32_t
 * %lu   |  uint32_t(readVcc())              |     uint32_t
 * %hu   |  uint8_t(telem.err))              |     uint8_t
 * ------+-------------------------------+-----------------------
 */

  snprintf(checksum, sizeof(checksum), "*%04X\n", get_CRC16_checksum(telem_string));
  memcpy(telem_string + strlen(telem_string), checksum, strlen(checksum) + 1);
  ss.println(telem_string);    // send to s/w serial port

  // send to radio for transmission, use enable pin to turn on and off
  digitalWrite(PIN_RADIO_EN, HIGH);
  rtty_txstring(telem_string); 
  digitalWrite(PIN_RADIO_EN, LOW);

//---------------------------------------------------------------
//Check error status, LED low indicates no error
//---------------------------------------------------------------
//  if (telem.err) digitalWrite(PIN_LED, HIGH);
//  else digitalWrite(PIN_LED, LOW);
    //Sleep options are: SLEEP_15MS,SLEEP_30MS,SLEEP_60MS,SLEEP_120MS,SLEEP_250MS,SLEEP_500MS,SLEEP_1S,SLEEP_2S,SLEEP_4S,SLEEP_8S

  // LED heartbeat if there are no errors, once per loop - replace with interrupt at set interval
/*  digitalWrite(PIN_LED, HIGH); 
  if (!telem.err) {            
    LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
    digitalWrite(PIN_LED, LOW);
  }     */
  
}

//===============================================================================
// 
//                             SUBROUTINES 
//
//===============================================================================

//*******************************************************************************
// Openlog Routines:
// 1) initOpenLog --> listens for '<' character at start-up as indication of working card
// 2) statOpenLog --> tests for openlog status and returns erros as appropriate
//*******************************************************************************

void initOpenLog() {
// resets openlog. openlog should then send 12<, which indicates it is booted and ready to go

  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.print(F("&&initOpenLog(): "));
  #endif
    
  loop_time = millis();
       
  //Reset OpenLog so we can listen for '<' character
  digitalWrite(PIN_OpL_RESET, LOW);
  //LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);  //sleep is more efficient than delay
  delay(60);
  digitalWrite(PIN_OpL_RESET, HIGH);

  while (millis() - loop_time < opl_init_timeout){ // check for '<', but not more than 3s

    if(ss.available()) {
      
      char c = ss.read();
      
      #if (DEBUG) 
        ss.print(c);
      #endif    
      
      if(c == '<') {
        err_opl_init = 0; 
        break;
      }
      
    }
  }
  
  #if (DEBUG) 
    ss.println();
  #endif
  
}

/********************************************************************************
* GPS Routines:
* 1) setGPS_power_save --> sends command to put GPS module into power saving mode
* 2) setGPS_GNSS_mode --> sends command to disable all GNSS options except GPS (needed for power saving mode)
* 3) setGPS_sleep --> sends command to put module into sleep mode
* 4) setGPS_wake --> sends wake command
* 5) sendUBX --> routine to send a UBX command including appropriate checksums to module
* 6) getUBX_ACK --> routing to recieve and process acknowledgements recieved from the unit
********************************************************************************/

void setGPS_power_save(){
// command to put gps module into power saving mode: UBX-CFG-RXM

  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&setGPS_power_save()"));
  #endif

  uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01};  // Expected checksums: 0x22, 0x92 

  sendUBX(setPSM,  sizeof(setPSM)/ sizeof(uint8_t));                             
  PSM_set_success = getUBX_ACK(setPSM);

  #if (DEBUG)
    ss.print(F("&&PSM_set_success = "));
    ss.println(PSM_set_success);
  #endif  
}

//-------------------------------------------------------------------------------

void setGPS_GNSS_mode() {    
// command to enable only GPS data (not GLONASS etc) - reqd to enable
// power saving mode: UBX-CFG-GNSS

  wdt_reset();  // check in with watchdog
  #if (DEBUG)
    ss.println(F("&&setGPS_GNSS_mode()"));
  #endif
  
  uint8_t setGNSS[] = { 0xB5, 0x62, 0x06, 0x3E, 0x2C, 0x00, 0x00, 0x00,
                        0x20, 0x05, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00,
                        0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00,
                        0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00,
                        0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00,
                        0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00,
                        0x01, 0x01};   // Expected checksums: 0xFC, 0x11
                        
  sendUBX(setGNSS,  sizeof(setGNSS)/ sizeof(uint8_t));                             
  GNSS_set_success = getUBX_ACK(setGNSS);

  #if (DEBUG)
    ss.print(F("&&GNSS_set_success = "));
    ss.println(GNSS_set_success);
  #endif
}

//-------------------------------------------------------------------------------

void setGPS_sleep() {
// command gps unit to sleep: UBX-RXM-PMREQ

  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&setGPS_sleep()"));
  #endif
    
  uint8_t setSLEEP[] = { 0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 
                         0x00, 0x00, 0x02, 0x00, 0x00, 0x00}; // Expected checksums: 0x4D, 0x3B
    
  sendUBX(setSLEEP,  sizeof(setSLEEP)/ sizeof(uint8_t));
  //LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
  delay (500);

}


//-------------------------------------------------------------------------------

void setGPS_wake() {

  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&setGPS_wake()"));
  #endif

  /*
  uint8_t setWAKE[] = { 0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00,  
                        0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
                        
  sendUBX(setWAKE,  sizeof(setWAKE)/ sizeof(uint8_t));
  _delay_ms(500);
  */
   
  Serial.flush();                 //flush the buffer
  Serial.write(0xFF);             //dummy message to wake up gps module if it was asleep
  //LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);  
  delay(500);
  
}


//-------------------------------------------------------------------------------

void setGPS_flight_mode() {    
// Initialise GPS module
// 1) select flight mode (dynamic mode 6): UBX-CFG-NAV5
// 2) disable all NMEA message except GGA

  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&setGPS_flight_mode()"));
  #endif
  
  uint8_t setDM6[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,  
                      0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 
                      0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 
                      0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                      0x00, 0x00};   // Expected checksums: 0x16, 0xDC
   
  sendUBX(setDM6,  sizeof(setDM6)/ sizeof(uint8_t));
  FM_set_success = getUBX_ACK(setDM6);
  
  #if (DEBUG)
    ss.print(F("&&FM_set_success = "));
    ss.println(FM_set_success);
  #endif
      
  // turn off all NMEA telem except GGA and GSA (fix info)
  Serial.print(F("$PUBX,40,GLL,0,0,0,0*5C\r\n"));
  Serial.print(F("$PUBX,40,ZDA,0,0,0,0*44\r\n"));
  Serial.print(F("$PUBX,40,VTG,0,0,0,0*5E\r\n"));
  Serial.print(F("$PUBX,40,GSV,0,0,0,0*59\r\n"));
//  Serial.print(F("$PUBX,40,GSA,0,0,0,0*4E\r\n"));
  Serial.print(F("$PUBX,40,RMC,0,0,0,0*47\r\n"));

}

//-------------------------------------------------------------------------------

void setGPS_save_settings() {
//save settings so they done get lost between power-downs: UBX-CFG-CFG

  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&setGPS_save_settings()"));
  #endif

  uint8_t setCFG[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 
                      0x00, 0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 
                      0x00, 0x04, 0x03};   // Expected checksums: 0x3E, 0xB7

  sendUBX(setCFG,  sizeof(setCFG)/ sizeof(uint8_t));
  CFG_save_success = getUBX_ACK(setCFG);
  
  #if (DEBUG)
    ss.print(F("&&GPS_save_success = "));
    ss.println(CFG_save_success);
  #endif
}

//-------------------------------------------------------------------------------

void sendUBX(uint8_t *MSG, uint8_t len) { 
//based on code from http://ava.upuaut.net/?p=750

  //declarations
   uint8_t CK_A = 0;
   uint8_t CK_B = 0;

  //calculate checksums, see protocol document page 127
  for (uint8_t ubxi=2; ubxi<len; ubxi++) {
    CK_A = CK_A + MSG[ubxi];
    CK_B = CK_B + CK_A;
  }

  //write ubx message 
//  setGPS_wake();                  //ensure module is awake
  
  for(uint8_t i=0; i<len; i++) {      //write the ubx message
    Serial.write(MSG[i]);
  }
  Serial.write(CK_A);             //write checksums
  Serial.write(CK_B);

  #if (DEBUG)
    ss.print(F("&&DB_V: CK_A = "));
    ss.print(CK_A, HEX);
    ss.print(F(" CK_B = "));
    ss.println(CK_B, HEX);
  #endif 
  
}

//-------------------------------------------------------------------------------

boolean getUBX_ACK(uint8_t *MSG) {
// reads from GPS and returns 1 if expected packets are received
// from http://ava.upuaut.net/?p=750
  uint8_t  b;
  uint8_t  ackByteID = 0;
  uint8_t  ackPacket[10];
  uint32_t startTime = millis();
 
// Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B
 
// Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }
 
    // Timeout if no valid response in 1 seconds
    if (millis() - startTime > 1000) {
      return false;
    }
 
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      }
      else {
        ackByteID = 0; // Reset and look again, invalid order
      }
    }
  }
}

//*******************************************************************************
// Sensor Routines:
// 3) readVcc --> reads internal vcc voltage 
// 4) refPressure --> samples pressure to get reference pressure, called from setup
// 5) getPressure --> processes BMP180 output to calculate temp and pressure (temp not used)
// 6) getBMP180_other --> requests 2 bytes from I2C bus at BMP180 address, processes and return 
// 7) getBMP180_press --> requests 3 bytes from I2C bus at BMP180 address, processes and return
// 8) readFrom --> utility to read requested number of bytes from I2C bus
// 9) writeTo --> utility to write to I2C bus (used in eg initAcc)
//******************************************************************************* 

uint32_t readVcc() {
//source code: https://web.archive.org/web/20150218055034/http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
//useful refs: http://www.protostack.com/blog/2011/02/analogue-to-digital-conversion-on-an-atmega168/
//             https://www.gammon.com.au/adc

  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&readVcc()"));
  #endif
  
//declare variables
  uint32_t result;
  
// Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  // the above line returns 01001110
  // (REFS1,REFS0)=(0,1)=AVcc
  // (MUX3,MUX2,MUX1,MUX0)=(1,1,1,0)= select 1.1v input to ADC

//Get ADC data
  delay(2);                         // Wait for Vref to settle - dont use sleep here as might affect ADC
  ADCSRA |= _BV(ADSC);              // Convert
  while (bit_is_set(ADCSRA,ADSC));  // Wait for conversion to complete

//Form result 
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  // 1024 (ADC steps) * 1.1 (ref volts) * 1000 (convert to mv) = 1126400

  return result;
}

//-------------------------------------------------------------------------------

uint32_t getBMP_press_ref() {
// get reference pressure 
  
  wdt_reset();  // check in with watchdog
  #if (DEBUG)
    ss.println(F("&&getBMP_press_ref()"));
  #endif
  
  uint8_t  i    = 0;
  uint32_t refP = 0;  
  loop_time = millis();

  while (millis() - loop_time < ref_press_sample_time) { 
    refP += getBMP_press();
    i++;
  } 

  loop_time = 0;      // reset loop time as its used as a condition in the first loop
  refP /= uint32_t(i);

  return refP;
  
}

//-------------------------------------------------------------------------------

uint32_t getBMP_temp(){
  
//prelims
  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&getBMP_temp()"));
  #endif

// declare variables 
  int32_t    x1, x2, b5, ut, t;    // signed long (4 byte)   
  uint16_t   ac5 = 25315;
  uint16_t   ac6 = 15361;
  int16_t    mc  = -11786;
  int16_t    md  = 2628;

//calculate BMP180 internal temperature
  ut    = int32_t(getBMP180_ut());
  x1    = (ut - ac6) * ac5 >> 15;
  x2    = ((int32_t)mc << 11) / (x1 + md);
  b5    = (int32_t)x1 + x2;
  t     = ((int32_t)b5 + 8L) / 16L;         // temp from bmp180 module - it is degC*10
/*  
  #if (DEBUG) 
    ss.println(F("-getBMP_temp()----"));
    ss.print(F("&&ut   = "));  ss.println(ut);
    ss.print(F("&&x1   = "));  ss.println(x1);
    ss.print(F("&&x2   = "));  ss.println(x2);
    ss.print(F("&&b5   = "));  ss.println(b5);
    ss.print(F("&&t    = "));  ss.println(t);    
  #endif 
*/  
  return t;
}
//-------------------------------------------------------------------------------

uint32_t getBMP_press(){
  
//prelims
  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&getBMP_press()"));
  #endif

// declare variables 
  int32_t    x1, x2, x3, b3, b5, b6, t, ut, up;    // signed long (4 byte)   
  uint32_t   b4, b7, p;                            // unsigned long
  uint8_t    oss = 1;                     
  int16_t    ac1 = 7072;
  int16_t    ac2 = -1021; 
  int16_t    ac3 = -14500;  
  uint16_t   ac4 = 33551;
  uint16_t   ac5 = 25315;
  uint16_t   ac6 = 15361;
  int16_t    b1  = 6515;
  int16_t    b2  = 34;
  int16_t    mb  = -32768;
  int16_t    mc  = -11786;
  int16_t    md  = 2628;
  
//calculate BMP180 internal temperature
  ut    = int32_t(getBMP180_ut());
  x1    = (ut - ac6) * ac5 >> 15;
  x2    = ((int32_t)mc << 11) / (x1 + md);
  b5    = (int32_t)x1 + x2;
/*  
  #if (DEBUG) 
    ss.println(F("-getBMP_press()----"));
    ss.print(F("&&ut   = "));  ss.println(ut);
    ss.print(F("&&x1   = "));  ss.println(x1);
    ss.print(F("&&x2   = "));  ss.println(x2);
    ss.print(F("&&b5   = "));  ss.println(b5);
  #endif 
*/ 
//calculate BMP180 pressure
  up = int32_t(getBMP180_up());
  b6 = (int32_t)b5 - 4000L;
  x1 = (b2 * (b6 * b6 >> 12)) >>12;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = (((ac1 * 4L + x3) << oss) + 2L) / 4L;
/*
  #if (DEBUG) 
    ss.print(F("&&up   = "));  ss.println(up);
    ss.print(F("&&b6   = "));  ss.println(b6);
    ss.print(F("&&x1   = "));  ss.println(x1);  
    ss.print(F("&&x2   = "));  ss.println(x2);
    ss.print(F("&&x3   = "));  ss.println(x3);  
    ss.print(F("&&b3   = "));  ss.println(b3);  
  #endif
*/ 
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = ac4 * (x3 + 32768L) >> 15;
  b7 = (up - b3) * (50000 >> oss);
  if (b7 < 0x80000000) p = (b7 * 2L) / b4;
  else                 p = (b7 / b4) * 2L;
/*
  #if (DEBUG) 
    ss.print(F("&&x1   = "));  ss.println(x1);
    ss.print(F("&&x2   = "));  ss.println(x2);  
    ss.print(F("&&x3   = "));  ss.println(x3);
    ss.print(F("&&b4   = "));  ss.println(b4);  
    ss.print(F("&&b7   = "));  ss.println(b7);
    ss.print(F("&&prss = "));  ss.println(p);
  #endif 
*/ 
  x1 = (p >> 8) * (p >> 8);
  x1 = ((int32_t)x1 * 3038L) >> 16;
  x2 = -(7357L * int32_t(p)) >> 16;
  p = p + (x1 + x2 + 3791L) / 16L;
/*
  #if (DEBUG)   
    ss.print(F("&&x1   = "));  ss.println(x1);
    ss.print(F("&&x2   = "));  ss.println(x2);  
    ss.print(F("&&prss = "));  ss.println(p);
  #endif 
*/
  return p;
}

//-------------------------------------------------------------------------------

uint16_t getBMP180_ut() {
//reads 2 bytes from I2C bus, formats and returns

//prelims
  wdt_reset();  // check in with watchdog
  #if (DEBUG)
    ss.println(F("&&getBMP180_ut()"));
  #endif
  
// declare variables
  uint8_t    buff[2];
  uint16_t   result;
  uint8_t    oss = 1;     

/*0x77 is the I2C address of the BMP180 module
  0xF4 is the I2C address of the control register
  0xF6 is the I2C address of the result register
  0x2E is the command required for temperature calc
  0x74 is the command required for pressure calc */
     
  // send command, wait, read response
  writeToI2C(0x77, 0xF4, 0x2E);         // send to I2C device (0x77) at control register (0xF4) the temp request (0x2E)
  delay(15);                         // 5 in datasheet but higher wait needed to make it work, dont understand why
  readFromI2C(0x77, 0xF6, 2, buff);     // read data from I2C (0x77) result 0xF6 2bytes into variable 'buff'
  
  // process telem
  result =  (uint16_t)buff[0] << 8 | (uint16_t)buff[1];
/*
  // debugging
  #if (DEBUG) 
    ss.println(F("-getBMP180_ut()---")); 
    ss.print("buff[0]="); ss.println(buff[0],BIN);
    ss.print("buff[1]="); ss.println(buff[1],BIN);
    ss.print("result (BIN)="); ss.println(result,BIN);
    ss.print("result (DEC)="); ss.println(result,DEC); 
  #endif 
*/
  return result;
}

//-------------------------------------------------------------------------------

uint32_t getBMP180_up() {
//reads 3 bytes from I2C bus, formats and returns

//prelims
  wdt_reset();  // check in with watchdog
  #if (DEBUG)
    ss.println(F("&&getBMP180_up()"));
  #endif
  
// declare variables
  uint8_t    buff[3] = {0};
  uint32_t   result;
  uint8_t    oss = 1;      

/*0x77 is the I2C address of the BMP180 module
  0xF4 is the I2C address of the control register
  0xF6 is the I2C address of the result register
  0x2E is the command required for temperature calc
  0x74 is the command required for pressure calc*/
        
// send command, wait, read response
  writeToI2C(0x77, 0xF4, 0x74);      // send to I2C device (0x77) at control register (0xF4) the pressure request (0x74)
  delay(15); 
  readFromI2C(0x77, 0xF6, 3, buff);  // read data from I2C (0x77) result 0xF6 3bytes into variable 'buff'

//process output
  result = ((uint32_t)buff[0] << 16 | (uint32_t)buff[1] << 8) | (uint32_t)buff[2];
  result >>= (8 - oss);
/*
// debugging
#if (DEBUG) 
    ss.println(F("-getBMP180_up()---"));  
    ss.print("buff[0]="); ss.println(buff[0],BIN);
    ss.print("buff[1]="); ss.println(buff[1],BIN);
    ss.print("buff[2]="); ss.println(buff[2],BIN); 
    ss.print("result (BIN)="); ss.println(result,BIN);   
    ss.print("result (DEC)="); ss.println(result,DEC); 
  #endif 
*/
  return result;
}

//-------------------------------------------------------------------------------

void * readFromI2C(uint8_t dev, uint8_t add, uint8_t num, uint8_t buff[]) {
  
  //reads num bytes starting from address register to buff array
  //See http://dsscircuits.com/articles/arduino-i2c-master-library for library commands

  err_i2c_data = I2c.read(dev,add,num);
  if (err_i2c_data > 1) err_i2c_data = 1;        // write function will return value 1-7 on error - reset it to 1
    
  for (uint8_t i=0; i<num; i++) {                //could use I2c.available loop as implemented in previous KISSR versions
    buff[i]=I2c.receive();
    // debug
    //ss.print("i="); ss.print(i); ss.print(" "); ss.println(buff[i], BIN);    
  }
    
  return buff;
}

//-------------------------------------------------------------------------------

void writeToI2C(uint8_t DEVICE, int8_t address, int8_t val) {

  //Writes a value to address register on ACC
  //See http://dsscircuits.com/articles/arduino-i2c-master-library for library commands

  err_i2c_data = I2c.write(DEVICE,address,val);
  
  //debug
  ss.print(F("err_i2c_data="));ss.println(err_i2c_data);
  
  if (err_i2c_data > 1) err_i2c_data = 1;        // write function will return value 1-7 on error - reset it to 1

}

//*******************************************************************************
// Radio Routines:
// 1) get_CRC16_checksum --> calculated checksum for telemetry string
// 2) rtty_txstring --> process string and transmit each byte
// 3) rtty_txbyte --> processes byte into bits
// 4) rtty_txbit --> transmits a bit
//*******************************************************************************

uint16_t get_CRC16_checksum (char *string) {
//source: https://ukhas.org.uk/communication:protocol?s[]=checksum

//declare variables
  size_t i;
  uint16_t crc = 0xFFFF;
  uint8_t c;
 
 //calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}

//-------------------------------------------------------------------------------

void rtty_txstring (char * string) {
// from: http://ava.upuaut.net/?p=627

  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&rtty_txstring()"));
  #endif

  char c;
  c = *string++;
   
  while ( c != '\0'){    //stop when we get to the end of the line
    rtty_txbyte (c);     //send byte
    c = *string++;       //get the next character
  }
}

//-------------------------------------------------------------------------------

void rtty_txbyte (char c) {
// from http://ava.upuaut.net/?p=627
  
  uint8_t i;
  rtty_txbit (0); // Start bit
   
  // Send bits for for char LSB first
  for (i = 0; i < 7; i++) {          // Change this here 7 or 8 for ASCII-7 / ASCII-8
    if (c & 1) rtty_txbit(1); 
    else rtty_txbit(0);
    c = c >> 1;
  }
   
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}
 
//-------------------------------------------------------------------------------

void rtty_txbit (uint8_t bit) {
// based on source from http://ava.upuaut.net/?p=627

  // frequency is centred on 434272MHz, bandwidth is 440
  if (bit)  analogWrite(PIN_RADIO_TX, 47);  // 434272216Hz
  else      analogWrite(PIN_RADIO_TX, 30);  // 434271776 Hz
  delay(20);                            //50 baud

}  
