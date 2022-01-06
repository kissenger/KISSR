//
//uint16_t blinkOn = 1;   // initialise to a value otherwise wired stuff happens
//uint16_t blinkOff = 1;
//
//// Interrupt Service Routine: 
//// http://www.gammon.com.au/interrupts
//// https://microcontrollerslab.com/arduino-timer-interrupts-tutorial/
//// http://www.avrbeginners.net/architecture/timers/timers.html
//
//ISR(TIMER1_COMPA_vect) {
//
//  if (blinkOn == 0) {
//    digitalWrite(PIN_LED, LOW);
//  } else {
//    bool pinState = digitalRead(PIN_LED);
//    digitalWrite(PIN_LED, !pinState);
//    if (pinState) {
//      OCR1A = blinkOff;
//    } else {
//      OCR1A = blinkOn;
//    } 
//  }
//}
//
//// Initialise Timer1 interrupts to control LED flash
//// Prescaler set to 1024 meaning each clock cycle is 1/(8MHz/1024)=0.000128s
//// Hence max time delay is 65535*0.000128=8.32secs
//void interruptInit() {
//  noInterrupts();
//  TCCR1A = 0;  // initilise timer 1 registers
//  TCCR1B = 0;   
//  TCCR1B = bit(WGM12) | bit(CS12) | bit(CS10);   // set prescaler to 1024
//  TIMSK1 = bit(OCIE1A);
//  TCNT1 = 0;   // reset counter
//  interrupts();
//}
//
//// Set the LED blink rates according to desired mode
//// If succesful returns the flight mode requested, otherwise returns 0
//byte setLedMode(byte mode) {
//
//  if (mode == FAST_BLINK) {
//    blinkOn = 1000;
//    blinkOff = 1000;
//    return mode;
//  } else if (mode == SLOW_BLINK) {
//    blinkOn = 500;
//    blinkOff = 65535;
//    return mode;
//  } else if (mode == IN_FLIGHT) {
//    blinkOn = 0;
//    blinkOff = 0;
//  }
//  return 0;
//}
