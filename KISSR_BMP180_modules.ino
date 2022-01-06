
//-------------------------------------------------------------------------------
// Write to pressure and check for response 
// Send to I2C device (0x77) at control register (0xF4) the pressure request (0x74), and await response
bool bmpPing() {

  return i2cWrite(0x77, 0xF4, 0x74) == 0;    

}

//-------------------------------------------------------------------------------

uint32_t bmpRefPressure() {
// get reference pressure 
  
  wdt_reset();  // check in with watchdog
  
  #if (DEBUG)
    ss.println(F("&&bmpRefPressure()"));
  #endif
  
  uint8_t i = 0;
  uint32_t refP = 0;  
  uint32_t now = millis();

  while (millis() - now < REF_PRESS_SAMPLE_PERIOD) { 
    refP += bmpPressure();
    i++;
  } 

  refP /= uint32_t(i);  

  return refP;
  
}

//-------------------------------------------------------------------------------

uint32_t bmpTemperature(){
  
  wdt_reset();  // check in with watchdog
  
  #if (DEBUG) 
    ss.println(F("&&bmpTemperature()"));
  #endif

// declare variables 
  int32_t    x1, x2, b5, ut, t;    // signed long (4 byte)   
  uint16_t   ac5 = 25315;
  uint16_t   ac6 = 15361;
  int16_t    mc  = -11786;
  int16_t    md  = 2628;

//calculate BMP180 internal temperature
  ut    = int32_t(readUt());
  x1    = (ut - ac6) * ac5 >> 15;
  x2    = ((int32_t)mc << 11) / (x1 + md);
  b5    = (int32_t)x1 + x2;
  t     = ((int32_t)b5 + 8L) / 16L;         // temp from bmp180 module - it is degC*10
/*
  // don't delete
  ss.println(F("-getBMP_temp()----"));
  ss.print(F("&&ut   = "));  ss.println(ut);
  ss.print(F("&&x1   = "));  ss.println(x1);
  ss.print(F("&&x2   = "));  ss.println(x2);
  ss.print(F("&&b5   = "));  ss.println(b5);
  ss.print(F("&&t    = "));  ss.println(t);    
*/
  return t;
}
//-------------------------------------------------------------------------------

uint32_t bmpPressure(){
  
  wdt_reset();  // check in with watchdog
  
  #if (DEBUG) 
    ss.println(F("&&bmpPressure()"));
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
  ut    = int32_t(readUt());
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
  up = int32_t(readUp());
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
// Reads 2 bytes from I2C bus, formats and returns

uint16_t readUt() {

  wdt_reset();  // check in with watchdog
  
  #if (DEBUG)
    ss.println(F("&&readUt()"));
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
  i2cWrite(0x77, 0xF4, 0x2E);         // send to I2C device (0x77) at control register (0xF4) the temp request (0x2E)
  delay(15);                         // 5 in datasheet but higher wait needed to make it work, dont understand why
  i2cRead(0x77, 0xF6, 2, buff);     // read data from I2C (0x77) result 0xF6 2bytes into variable 'buff'
  
  // process telem
  result =  (uint16_t)buff[0] << 8 | (uint16_t)buff[1];
/*
  // don't delete
  ss.println(F("-getBMP180_ut()---")); 
  ss.print("buff[0]="); ss.println(buff[0],BIN);
  ss.print("buff[1]="); ss.println(buff[1],BIN);
  ss.print("result (BIN)="); ss.println(result,BIN);
  ss.print("result (DEC)="); ss.println(result,DEC); 
*/
  return result;
}

//-------------------------------------------------------------------------------
// Reads 3 bytes from I2C bus, formats and returns

uint32_t readUp() {

  wdt_reset();  // check in with watchdog
  
  #if (DEBUG)
    ss.println(F("&&readUp()"));
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
  i2cWrite(0x77, 0xF4, 0x74);      // send to I2C device (0x77) at control register (0xF4) the pressure request (0x74)
  delay(15); 
  i2cRead(0x77, 0xF6, 3, buff);  // read data from I2C (0x77) result 0xF6 3bytes into variable 'buff'

//process output
  result = ((uint32_t)buff[0] << 16 | (uint32_t)buff[1] << 8) | (uint32_t)buff[2];
  result >>= (8 - oss);

/*
  // don't delete
  ss.println(F("-getBMP180_up()---"));  
  ss.print("buff[0]="); ss.println(buff[0],BIN);
  ss.print("buff[1]="); ss.println(buff[1],BIN);
  ss.print("buff[2]="); ss.println(buff[2],BIN); 
  ss.print("result (BIN)="); ss.println(result,BIN);   
  ss.print("result (DEC)="); ss.println(result,DEC); 
*/
  return result;
}

//-------------------------------------------------------------------------------
// Reads num bytes starting from address register to buff array
// See https://github.com/DSSCircuits/I2C-Master-Library/blob/master/examples/HMC5883L/HMC5883L.pde

void * i2cRead(uint8_t dev, uint8_t add, uint8_t num, uint8_t buff[]) {

  uint8_t err = I2c.read(dev, add, num);
//  if (err_i2c_data > 1) err_i2c_data = 1;        // write function will return value 1-7 on error - reset it to 1
    
  for (uint8_t i = 0; i < num; i++) {              // could use I2c.available loop as implemented in previous KISSR versions
    buff[i] = I2c.receive(); 
  }
    
  return buff;
}

//-------------------------------------------------------------------------------
// Writes a value to address register on ACC
// See https://github.com/DSSCircuits/I2C-Master-Library/blob/master/examples/HMC5883L/HMC5883L.pde
// and https://rheingoldheavy.com/changing-the-i2c-library/
// this function returns the error value
uint8_t i2cWrite(uint8_t DEVICE, int8_t address, int8_t val) {

  return I2c.write(DEVICE, address, val);
  
}
