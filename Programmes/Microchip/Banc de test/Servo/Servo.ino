// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h>

/*****************************************************************/
// Servo
Servo Servo1;

void setup() 
{   
    Servo1.attach(9);
    Servo1.write(90);
}

/*****************************************************************/
void loop() 
{
    while(true)
    {
   }
}
