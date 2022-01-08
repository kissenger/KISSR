

void getGPSData(TinyGPSPlus & g, bool & errData, bool & errFix) {
  
  #if (DEBUG) 
    ss.println(F("&&NMEA loop"));
  #endif
  
  uint32_t now = millis();
  uint32_t gpsDataTime = now;
  
  while (1) {                                   //keep trying until we get readable data or timeout

    // if we have serial data then check for coherent sentence
    if (Serial.available()) {
      
      gpsDataTime = millis();
      errGPSData = false;      
      char c = Serial.read();     
      
      #if (DEBUG_NMEA)
        ss.write(c);
      #endif
      
      //see if we got a complete, readable sentence
      if (gps.encode(c)) {
      
        // hang around and wait for solid 3D fix with fresh data
//        if (strcmp(gpsFixQuality.value(), "3") == 0 && gps.satellites.value() >= 5 && gps.location.age() < 2000) {
        if (gps.satellites.value() >= 5 && gps.location.age() < 2000) {          
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


//-------------------------------------------------------------------------------
// Command to put gps module into power saving mode: UBX-CFG-RXM
// Returns 1 if successful, 0 otherwise
bool gpsSetPSM(){

  wdt_reset();  // check in with watchdog
  
  #if (DEBUG) 
    ss.println(F("&&setGPS_power_save(): "));
  #endif

  uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01};  // Expected checksums: 0x22, 0x92 

  sendUBX(setPSM,  sizeof(setPSM)/ sizeof(uint8_t));
     
  return getUBX_ACK(setPSM) == 1;

}

//-------------------------------------------------------------------------------
// Command to enable only GPS data (not GLONASS etc) - reqd to enable PSM: UBX-CFG-GNSS
// Returns 1 if successful, 0 otherwise
bool gpsSetGNSS() {    


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
  return getUBX_ACK(setGNSS) == 1;

}



//-------------------------------------------------------------------------------
// Command to enable only GPS data (not GLONASS etc) - reqd to enable PSM: UBX-RXM-PMREQ
// Does not return any value

bool gpsStatusPSM = false;
bool gpsStatusFM = false;
bool gpsStatusGNSS = false;  

void gpsSleep() {

  #if (DEBUG) 
    ss.println(F("&&setGPS_sleep()"));
  #endif
  
  if (gpsStatusPSM) {  
    uint8_t setSLEEP[] = { 0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 
                           0x00, 0x00, 0x02, 0x00, 0x00, 0x00}; // Expected checksums: 0x4D, 0x3B
      
    sendUBX(setSLEEP,  sizeof(setSLEEP)/ sizeof(uint8_t));
//    delay (500);  // TODO: What is this delay for?
  }

}

//-------------------------------------------------------------------------------
// Attempts to set the required modes on the gps unit
// Returns error code 1 if any of the modes failed to set correctly (0 otherwise)
bool gpsSetModes() {

  ss.print(F("gpsStatusFM: "));
  ss.println(gpsStatusFM);
  ss.print(F("gpsStatusGNSS: "));
  ss.println(gpsStatusGNSS);  
  ss.print(F("gpsStatusPSM: "));
  ss.println(gpsStatusPSM);  

  if (!gpsStatusFM) {
    gpsStatusFM = gpsSetFM(); 
  }
  
  if (!gpsStatusGNSS) {
    gpsStatusGNSS = gpsSetGNSS();      
  }

  if (!gpsStatusPSM) {
    gpsStatusPSM = gpsSetPSM();
  }
  
  return !gpsStatusFM | !gpsStatusGNSS | !gpsStatusPSM;

}

//-------------------------------------------------------------------------------

void gpsWake() {

  wdt_reset();  // check in with watchdog
  #if (DEBUG) 
    ss.println(F("&&setGPS_wake()"));
  #endif
   
  Serial.flush();                 //flush the buffer
  Serial.write(0xFF);             //dummy message to wake up gps module if it was asleep

  delay(500);
  
}


//-------------------------------------------------------------------------------

bool gpsSetFM() {    
// Initialise GPS module
// select flight mode (dynamic mode 6): UBX-CFG-NAV5


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
  return getUBX_ACK(setDM6) == 1;
  
//  #if (DEBUG)
//    openlog.print(F("&&FM_set_success = "));
//    openlog.println(FM_set_success);
//  #endif
      


}

// turn off all NMEA telem except GGA and GSA (fix info)
void gpsSetNMEA() {

  Serial.print(F("$PUBX,40,GLL,0,0,0,0*5C\r\n"));
  Serial.print(F("$PUBX,40,ZDA,0,0,0,0*44\r\n"));
  Serial.print(F("$PUBX,40,VTG,0,0,0,0*5E\r\n"));
  Serial.print(F("$PUBX,40,GSV,0,0,0,0*59\r\n"));
  Serial.print(F("$PUBX,40,RMC,0,0,0,0*47\r\n"));
}

//-------------------------------------------------------------------------------

bool gpsSaveSettings() {
//save settings so they dont get lost between power-downs: UBX-CFG-CFG

  wdt_reset();  // check in with watchdog
  
  #if (DEBUG) 
    ss.println(F("&&setGPS_save_settings()"));
  #endif

  uint8_t setCFG[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 
                      0x00, 0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 
                      0x00, 0x04, 0x03};   // Expected checksums: 0x3E, 0xB7

  sendUBX(setCFG,  sizeof(setCFG)/ sizeof(uint8_t));
  return getUBX_ACK(setCFG) == 1;
  
//  #if (DEBUG)
//    openlog.print(F("&&GPS_save_success = "));
//    openlog.println(CFG_save_success);
//  #endif
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
