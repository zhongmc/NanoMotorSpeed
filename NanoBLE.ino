#include <Arduino.h>



void initNanoBle(long baudrate )
{
//  connectToHM10();
  
    Serial.begin( baudrate );
    delay(30);
}



bool connectToHM10(){
  bool found = false;
 
    // 0 9600 1 19200 2 38400 3 57600 4 115200  5 4800 6 2400 7 1200 8 230400
   long baudrate[6] ={4800,9600,19200,38400,57600,115200};

  long baud;
  for(int j=0; j<6; j++)
   {
      baud = baudrate[j];
    
      Serial.begin(baudrate[j]);
      delay(100);
      Serial.write("AT\r\n"); 
      delay(500);

      if( Serial.available())
        Serial.write("--");
        
      while (Serial.available()) {
        found = true;
        Serial.write(Serial.read());
      }
      
       if( found == true )
       {
        Serial.println("OK!");
        Serial.println( baud );
        return true;
       }
       delay(100);
   }
   return false;
}    
