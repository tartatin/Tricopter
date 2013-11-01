/**************************************************************************/
/*                                  I2C                                   */
/**************************************************************************/

#include <Wire.h> // I2C library, gyroscope

/*****************************************************************/
void setupI2C()
{
    Wire.begin();
}

/*****************************************************************/
//Writes val to address register on ACC
void i2cWriteTo(int DEVICE, byte address, byte val) {
   Wire.beginTransmission(DEVICE); //start transmission to ACC 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

/*****************************************************************/
//reads num bytes starting from address register on ACC in to buff array
void i2cReadFrom(int DEVICE, byte address, int num, byte buff[]) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC
  
  int i = 0;
  while(Wire.available())    //ACC may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}
