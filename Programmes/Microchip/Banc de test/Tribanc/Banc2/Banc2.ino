// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h>

#define V_PIN 7
#define I_PIN 4
 
Servo Motor1;
Servo Motor2;
Servo Motor3;

float Current;
float Voltage;

void updateVoltMeasures()
{
   Current = (analogRead(I_PIN)); // * 0.066705f);
   Voltage = (analogRead(V_PIN)); // * 0.020152f);
}
 
void setup() 
{   
  Serial.begin(9600); 
  
  pinMode(V_PIN, INPUT);
  pinMode(I_PIN, INPUT);
  
  // Pull up activée si high
  digitalWrite(V_PIN, LOW);
  digitalWrite(I_PIN, LOW);
  
  // Moteurs
  Motor1.attach(9);
  Motor2.attach(10);
  Motor3.attach(11);
  
  Motor1.writeMicroseconds(900);
  Motor2.writeMicroseconds(900);
  Motor3.writeMicroseconds(900);
  
  delay(7000);
}

void printPuissance(int pPuissance)
{
   Serial.print("@");
   Serial.println(pPuissance);
}

void deadLoop()
{
    Motor1.write(900);
    Motor2.write(900);
    Motor3.write(900);
    
    Serial.println("Arrêt définitif !");
    
    while(true);
}

void loop() 
{ 
   int lInitValue = 900;
   int MaxLoop;
   
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
           if (lCmd == 'k')
           {
               // mode kill !!!
               deadLoop();
           }
           else if (lCmd == 's')
           {
              Serial.println("Arret sur demande.");
              Motor1.writeMicroseconds(lInitValue);
              Motor2.writeMicroseconds(lInitValue);
              Motor3.writeMicroseconds(lInitValue);
              lStarted = false; 
           }
           else if (lCmd == 'v')
           {
              updateVoltMeasures();
              Serial.print("Tension : ");
              Serial.println(Voltage, 1);
              Serial.print("Intensité : ");
              Serial.println(Current, 1);
           }
           else if (lCmd == 'p')
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
                 MaxLoop = 2000;
               else
                 MaxLoop = 4000;
                 
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
       }
       
       // On met à jour l'état du moteur
       if (lStarted == false)
       {
          Motor1.write(lInitValue);
          Motor2.write(lInitValue);
          Motor3.write(lInitValue);
       }
       else
       {
          Motor1.writeMicroseconds(lValue);
          Motor2.writeMicroseconds(lValue);
          Motor3.writeMicroseconds(lValue);
          
          if (lLoopCounter >= MaxLoop)
          {
             Serial.println("Arret.");
             Motor1.writeMicroseconds(lInitValue);
             Motor2.writeMicroseconds(lInitValue);
             Motor3.writeMicroseconds(lInitValue);
             lStarted = false; 
          }
       }
       
       lLoopCounter++;
       delay(1);
   }
} 
