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
   
    g_MotorsState = false;
}

/*****************************************************************/
void startMotors()
{
    g_MotorsState = true;
}

/*****************************************************************/
// force en cN
int16_t getMotorCmdFromForce(int16_t p_Force)
{
    thresholdRangeInt16(&p_Force, 0, 2000);
  
    // D'après courbes empiriques
    int16_t lValue = (16 * p_Force) / 17;
    lValue += 1233;
    thresholdRangeInt16(&lValue, 1230, 1650);
    return lValue;
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
        int16_t lCmd;
        lCmd = getMotorCmdFromForce(p_Cmd[0]);
        Motor1.writeMicroseconds(lCmd);
        
        lCmd = getMotorCmdFromForce(p_Cmd[1]);
        Motor2.writeMicroseconds(lCmd);
        
        lCmd = getMotorCmdFromForce(p_Cmd[2]);
        Motor3.writeMicroseconds(lCmd);
    }
}