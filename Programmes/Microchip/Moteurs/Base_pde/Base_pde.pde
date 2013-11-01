// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
 
int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo.write(890);
  
  delay(7000);
} 
 
void loop() 
{ 
   //myservo.write(1500);
   //delay(10000);
   
   myservo.write(1300);
   
   delay(15000);
   
/*   for(int i = 1000; i <= 1600; i = i+10)
   {
       myservo.write(i);
       delay(200); 
   }*/
   
   myservo.write(1000);
  
   /* for(int i = 1600; i <= 2000; i = i+200)
    {
       myservo.write(i);
       delay(2000); 
    }
    
    for(int i = 2000; i >= 1200; i = i-200)
    {
       myservo.write(i);
       delay(2000); 
    }*/
    
    delay(15000);
} 
