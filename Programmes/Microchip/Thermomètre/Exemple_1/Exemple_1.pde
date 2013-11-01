//////////////////////////////////////////////////////////////////
//©2011 bildr
//Released under the MIT License - Please reuse change and share
//Simple code for the TMP102, simply prints temperature via serial
//////////////////////////////////////////////////////////////////

#include <Wire.h>
int tmp102Address = 0x48;
byte res;
int val;

void setup(){
  Serial.begin(9600);
  Wire.begin();
}

void loop(){
  Wire.requestFrom(tmp102Address,2); 

  byte MSB = Wire.receive();
  byte LSB = Wire.receive();

  int TemperatureSum = ((MSB << 8) | LSB) >> 4; //it's a 12bit int, using two's compliment for negative

  float celsius = TemperatureSum*0.0625;
  float fahrenheit = (TemperatureSum*0.1125) + 32;  

  Serial.print("Celsius: ");
  Serial.println(celsius);

  Serial.print("Fahrenheit: ");
  Serial.println(fahrenheit);

  delay(1000);
}
