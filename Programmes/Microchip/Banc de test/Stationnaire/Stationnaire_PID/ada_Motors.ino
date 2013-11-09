/**************************************************************************/
/*                              Motorisation                              */
/**************************************************************************/

#include <Servo.h>

/*****************************************************************/
// Moteurs
Servo Motor1;
Servo Motor2;
Servo Motor3;

/*****************************************************************/
bool  g_MotorsState = false; // Autorisation ou non pour les moteurs de tourner.
int16_t g_MotorsTarget[3];     // Commande que les moteurs doivent atteindre (en UCM, unité de commande moteur)
int16_t g_MotorsValue[3];      // Valeur actuelle des commandes moteurs (en UCM).

/*****************************************************************/
bool getMotorState()
{
    return g_MotorsState;
}

/*****************************************************************/
void deadStop()
{
    stopMotors();
    while(true) ;
}

/*****************************************************************/
void setupMotors()
{
    // Moteurs
    Motor1.attach(9);
    Motor2.attach(11);
    Motor3.attach(10);
    
    Motor1.writeMicroseconds(900);
    Motor2.writeMicroseconds(900);
    Motor3.writeMicroseconds(900);
    
    // Delay pour activation des moteurs
    delay(7000);
    
    // Etat initial des moteurs = arrêtés
    stopMotors();
}

/*****************************************************************/
void stopMotors()
{
    // Arrêt total des moteurs
    int16_t lValue = 900;
    
    g_MotorsTarget[0] = (float) lValue;
    g_MotorsTarget[1] = (float) lValue;
    g_MotorsTarget[2] = (float) lValue;
    
    g_MotorsValue[0] = (float) lValue;
    g_MotorsValue[1] = (float) lValue;
    g_MotorsValue[2] = (float) lValue;
    
    Motor1.writeMicroseconds(lValue);
    Motor2.writeMicroseconds(lValue);
    Motor3.writeMicroseconds(lValue);
   
    if (g_MotorsState == true)
        printText("Extinction.\n");
    g_MotorsState = false;
}

/*****************************************************************/
void startMotors()
{
    if (g_MotorsState == false)
        printText("Allumage.\n");
    g_MotorsState = true;
}

/*****************************************************************/
int16_t setMotorsTargets(float* p_Forces)
{
    float lValue;
    int16_t lIntValue;
  
    // D'après courbes empiriques
    // Moteur #1
    lValue = 174.1f * p_Forces[0] + 1226.0f;
    lIntValue = (int16_t) lValue;
    thresholdRangeInt16(&lIntValue, 1230, 1700);
    g_MotorsTarget[0] = lIntValue;
    
    // Moteur #2
    lValue = 115.0f * p_Forces[1] + 1220.0f;
    lIntValue = (int16_t) lValue;
    thresholdRangeInt16(&lIntValue, 1230, 1700);
    g_MotorsTarget[1] = lIntValue;
    
    // Moteur #3
    lValue = 93.7f * p_Forces[2] + 1229.0f;
    lIntValue = (int16_t) lValue;
    thresholdRangeInt16(&lIntValue, 1230, 1700);
    g_MotorsTarget[2] = lIntValue;
    
    return lIntValue;
}

/*****************************************************************/
void updateMotorsCmd()
{
    g_MotorsValue[0] = g_MotorsTarget[0];
    g_MotorsValue[1] = g_MotorsTarget[1];
    g_MotorsValue[2] = g_MotorsTarget[2];
    
    setMotorsCmd(g_MotorsValue);
}

/*****************************************************************/
void setMotorsCmd(int16_t *p_Cmd)
{
    if (g_MotorsState == false)
    {
        stopMotors();
    }
    else
    {
        Motor1.writeMicroseconds((int) p_Cmd[0]);
        Motor2.writeMicroseconds((int) p_Cmd[1]);
        Motor3.writeMicroseconds((int) p_Cmd[2]);
    }
}
