#include <Wire.h>
//@author neil hazra

void setup()
{
     Serial.begin(9600);
     Wire.begin(0x62);
     /*
     Wire.beginTransmission(0x62);
     Wire.write(0x18);
     Wire.write(32);  
     Wire.endTransmission();  
     delay(10); 
     Wire.beginTransmission(0x62);
     Wire.write(0x19);
     Wire.write(72);  
     Wire.endTransmission();  
     delay(10); 
     Wire.beginTransmission(0x62);
     Wire.write(0x1a);
     Wire.write(0x66);  
     Wire.endTransmission();  
     delay(10); 
     Wire.beginTransmission(0x62);
     Wire.write(0x1e);
     Wire.write(0x00);  
     Wire.endTransmission();  
     delay(10);   
   */  
}

void loop()
{
      Wire.beginTransmission(0x62);
      Wire.write(0x00);
      Wire.write(0x04);  
      Wire.endTransmission();  
      delay(20);

      Wire.beginTransmission(0x62);
      Wire.write(0x80 | 0x0f);
      Wire.endTransmission();	
      Wire.requestFrom(0x62, 2);
      int distanceh = Wire.read();
      int distanceL =  Wire.read();
      Wire.endTransmission();  
      delay(20); 
      Serial.print(";"); 
      Serial.print(distanceL | distanceh<<8 ,DEC);
      Serial.print(";\n");
      delay(50);
}
