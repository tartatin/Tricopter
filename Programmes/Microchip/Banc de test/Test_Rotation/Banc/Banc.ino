// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h>
 
Servo Motor1;
Servo Motor2;
Servo Motor3;
 
void setup() 
{ 
  Motor1.attach(11);
}

void loop() 
{ 
   int lValue = 1350;
   int lLoopCounter = 0;
   
   int lFirst = 950;
   
   while(true)
   {
     // Initialisation des moteurs
     Motor1.writeMicroseconds(lFirst);
    
     // On attend que tout se mette en place
     delay(7000);
     
     // Lancement des moteurs
     Motor1.writeMicroseconds(lValue);
     delay(15000);
     
     Motor1.writeMicroseconds(lFirst);
     delay(2000);
     
     Motor1.writeMicroseconds(0);
     
     while(true);
   }
} 
