
bool openlogInit() {
// resets ss. openlog should then send 12<, which indicates it is booted and ready to go

  wdt_reset();  // check in with watchdog
  
  uint32_t now = millis();
       
  //Reset OpenLog so we can listen for '<' character
  digitalWrite(PIN_OpL_RESET, LOW);
  delay(60);
  digitalWrite(PIN_OpL_RESET, HIGH);

  while (millis() - now < 3000){ // check for '<', but not more than 3s

    if(ss.available()) {
      
      char c = ss.read();
      ss.print(c);  
         
      if(c == '<') {
        return true;
      }
      
    }
    
  }

  return false;
  
}
