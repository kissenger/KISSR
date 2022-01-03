
//source code: https://web.archive.org/web/20150218055034/http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
//useful refs: http://www.protostack.com/blog/2011/02/analogue-to-digital-conversion-on-an-atmega168/
//             https://www.gammon.com.au/adc

uint32_t readVcc() {

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
