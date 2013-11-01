// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h>
 
Servo myservo;  // create servo object to control a servo
int MaxLoop = 100;
 
void setup() 
{ 
  pinMode(3, INPUT);
  pinMode(2, INPUT);
  
  // Pull up activée si high
  digitalWrite(3, LOW);
  digitalWrite(2, LOW);
  
  Serial.begin(9600); 
  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo.writeMicroseconds(900);
  
  delay(7000);
}

void printPuissance(int pPuissance)
{
   Serial.print("@");
   Serial.println(pPuissance);
}

float getCurrent()
{
   return (analogRead(3) * 0.066705f);
}

float getVoltage()
{
   return (analogRead(2) * 0.020152f);
}

// Vérifie que les différents éléments ne sont pas hors limites
bool checkConsumptions()
{
   float lCurrent = getCurrent();
   float lVoltage = getVoltage();
   
   if (lVoltage < 0.1f)
   {
      // Le capteur n'est vraisemblablement pas connecté
     return true; 
   }
   
   // Batterie 2.2 mAh 25C => 65.5A max
   if (lCurrent > 60.0f)
   {
     Serial.println("OVERCURRENT !");
     printConsumptions();
     return false;
   }

   // Batterie LiPo 3S => 4.2V max par cellule
   if (lVoltage > 12.6f)
   {
      Serial.println("OVERVOLTAGE !");
      // on ne quitte pas pour autant
   }
   
   if (lVoltage < 11.25f)
   {
      Serial.println("UNDERVOLTAGE !");
      printConsumptions();
      return false;
   }
   
   return true;
}

// Envoie les intensités et tensions
void printConsumptions()
{
   float lCurrent = getCurrent();
   float lVoltage = getVoltage();
   Serial.print("A:");
   Serial.print(lCurrent);
   Serial.print(", V:");
   Serial.println(lVoltage);
}
 
void loop() 
{ 
   int lInitValue = 900;
   
   int lIncrement = 5;
   int lValue = 1230;
   bool lStarted = false;
   
   int lLoopCounter = 0;
   
   while (true)
   {
       // On vérifie si une commande a été envoyée par le PC
       int lCmd = Serial.read();
       if (lCmd != -1)
       {
           // Une commande a été envoyée, on la traite
           if (lCmd == 'p')
           {
               lValue += lIncrement;
               printPuissance(lValue);
           }
           else if (lCmd == 'm')
           {
               lValue -= lIncrement;
               printPuissance(lValue);
           }
           else if ((lCmd == 'g') || (lCmd == 'l'))
           {
               if (lCmd == 'g')
                 MaxLoop = 100;
               else
                 MaxLoop = 3000;
                 
               printPuissance(lValue);
               Serial.println("Demarrage dans 5s...");
               delay(5000);
               
               if (Serial.read() != -1)
               {
                  lStarted = false;
                  Serial.println("Demarrage annule.");
               }
               else
               {
                  lStarted = true;
                  lLoopCounter = 0;
                  Serial.println("Demarrage.");
               }
           }
           else if (lCmd == 's')
           {
              Serial.println("Arret sur demande.");
              myservo.writeMicroseconds(lInitValue);
              lStarted = false; 
           }
           else if (lCmd == 'c')
           {
              printConsumptions(); 
           }
       }
       else
       {
          if (lLoopCounter%40 == 0)
          {
             printConsumptions();
          }
       }
       
       // On met à jour l'état du moteur
       if (lStarted == false)
       {
          myservo.write(lInitValue);
       }
       else
       {
          myservo.writeMicroseconds(lValue);
          
          if (lLoopCounter >= MaxLoop)
          {
             Serial.println("Arret.");
             myservo.writeMicroseconds(lInitValue);
             lStarted = false; 
          }
       }
       
       lLoopCounter++;
       
       // Pause de 100 ms durant lesquelles on vérifie les consommations
       for(int i = 0; i < 10; ++i)
       {
          if (checkConsumptions() == false)
          {
             Serial.println("Erreur : HORS LIMITES !.");
             myservo.writeMicroseconds(lInitValue);
             lStarted = false;
             break;
          }
          
          delay(10);
       }
   }
} 
