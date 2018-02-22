#include <Wire.h>
byte readingData[8];



void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}


void loop() {
 Wire.beginTransmission(0x39);
 Wire.write(0x1c);
 Wire.endTransmission();
 Wire.requestFrom(0x39,2);
 Serial.println(Wire.read(),BIN);
 Serial.println(Wire.read(),BIN);
  
 Wire.beginTransmission(0x39);
 Wire.write(0x08);
 Wire.write(0x00);
 Wire.endTransmission();

 Wire.beginTransmission(0x39);
 Wire.write(0x09);
 Wire.write(0x00);
 Wire.endTransmission();
 
 Wire.beginTransmission(0x39);
 Wire.write(0x0A);
 Wire.write(0x00);
 Wire.endTransmission();
 
 Wire.beginTransmission(0x39);
 Wire.write(0x0B);
 Wire.write(0x00);
 Wire.endTransmission();


}

