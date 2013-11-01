// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h>
 
Servo Motor1;
Servo Motor2;
Servo Motor3;

Servo Servo1;
 
void setup() 
{ 
  //pinMode(3, INPUT);
  //pinMode(2, INPUT);
  
  // Pull up activée si high
  //digitalWrite(3, LOW);
  //digitalWrite(2, LOW);
  
  //Serial.begin(9600); 
  
  Motor1.attach(9);
  Motor2.attach(10);
  Motor3.attach(11);
  
  Servo1.attach(5);
  
  /*Motor1.writeMicroseconds(900);
  Motor2.writeMicroseconds(900);
  Motor3.writeMicroseconds(900);
  
  delay(7000);*/
}

int getServoValueFromAngle(int pAngle)
{
  int lTemp = pAngle*pAngle - 169*pAngle + 7897;
  lTemp = (int) (lTemp / 126);
  lTemp = max(min(lTemp, 160), 16);
  
  return lTemp;
}

void setServoAngle(int pAngle)
{
  int lValue = getServoValueFromAngle(pAngle);
  Servo1.write(lValue);
}

void loop() 
{ 
   int lValue = 1350;
   int lLoopCounter = 0;
   
   int lFirst = 950;
   int lMin = 1250;
   int lMax = 1600;
   
   int lServoAngle;
   
   while(true)
   {
     // Initialisation du servo
     lServoAngle = -57;
     setServoAngle(lServoAngle);
     
     // Initialisation des moteurs
     Motor1.writeMicroseconds(lFirst);
     Motor2.writeMicroseconds(lFirst);
     Motor3.writeMicroseconds(lFirst);
    
     // On attend que tout se mette en place
     delay(7000);
     
     // Lancement des moteurs
     Motor1.writeMicroseconds(lValue);
     Motor2.writeMicroseconds(lValue);
     Motor3.writeMicroseconds(lValue);
     
     for(int i = 0; i < 110; i++)
     {
       float lStep = (float) i / 110.0f;
       
       Motor1.writeMicroseconds(lMin + (lMax-lMin) * abs(sin( (lStep + 0.0f/3.0f) * 2.0f * 3.14159265f)));
       Motor2.writeMicroseconds(lMin + (lMax-lMin) * abs(sin( (lStep + 1.0f/3.0f) * 2.0f * 3.14159265f)));
       Motor3.writeMicroseconds(lMin + (lMax-lMin) * abs(sin( (lStep + 2.0f/3.0f) * 2.0f * 3.14159265f)));
       
       lServoAngle += 1;
       setServoAngle(lServoAngle);
       delay(100);
     }
     
     Motor1.writeMicroseconds(lFirst);
     Motor2.writeMicroseconds(lFirst);
     Motor3.writeMicroseconds(lFirst);
     
     delay(2000);
     
     Motor1.writeMicroseconds(0);
     Motor2.writeMicroseconds(0);
     Motor3.writeMicroseconds(0);
     
     for(int i = 0; i < 50; i++)
     {
       delay(100); 
     }
   }
} 
