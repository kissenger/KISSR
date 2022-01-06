//*******************************************************************************
// Radio Routines:
// 1) get_CRC16_checksum --> calculated checksum for telemetry string
// 2) rtty_txstring --> process string and transmit each byte
// 3) rtty_txbyte --> processes byte into bits
// 4) rtty_txbit --> transmits a bit
//*******************************************************************************

uint16_t getChecksum (char *string) {
//source: https://ukhas.org.uk/communication:protocol?s[]=checksum

//declare variables
  size_t i;
  uint16_t crc = 0xFFFF;
  uint8_t c;
 
 //calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update(crc, c);
  }
 
  return crc;
}

//-------------------------------------------------------------------------------
// Transmit char string from radio unit
// from: http://ava.upuaut.net/?p=627

void rttyTxString (char * string) {

//  wdt_reset();  // check in with watchdog
  
  #if (DEBUG) 
    ss.println(F("&&rttyTxString()"));
  #endif

  char c;
  c = *string++;
   
  while ( c != '\0'){    //stop when we get to the end of the line
    rttyTxByte(c);       //send byte
    c = *string++;       //get the next character
  }
}

//-------------------------------------------------------------------------------

void rttyTxByte (char c) {
// from http://ava.upuaut.net/?p=627
  
  uint8_t i;
  rttyTxBit(0); // Start bit
   
  // Send bits for for char LSB first
  for (i = 0; i < 7; i++) {          // Change this here 7 or 8 for ASCII-7 / ASCII-8
    if (c & 1) rttyTxBit(1); 
    else rttyTxBit(0);
    c = c >> 1;
  }
   
  rttyTxBit(1); // Stop bit
  rttyTxBit(1); // Stop bit
}
 
//-------------------------------------------------------------------------------

void rttyTxBit (uint8_t bit) {
// based on source from http://ava.upuaut.net/?p=627

  // frequency is centred on 434272MHz, bandwidth is 440
  if (bit) {
    analogWrite(PIN_RADIO_TX, 47);  // 434272216Hz
  } else {
    analogWrite(PIN_RADIO_TX, 30);  // 434271776 Hz
  }
  delay(20);                                //50 baud
//  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below.
//  delayMicroseconds(10150); 

}  
