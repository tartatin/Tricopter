/**************************************************************************/
/*                                 Servo                                  */
/**************************************************************************/

#include <Servo.h>

/*****************************************************************/
// Servo
Servo Servo1;
float g_ServoAngle; // Angle actuel du servo moteur

/**************************************************************************/
void setupServo()
{
    g_ServoAngle = 0.0f;
    Servo1.attach(5);
    setServoAngle(0.0f);
}

/**************************************************************************/
int getServoValueFromAngle(int pAngle)
{
    int lTemp = pAngle*pAngle - 169*pAngle + 7897;
    lTemp = (int) (lTemp / 126);
    lTemp = max(min(lTemp, 160), 16);
    
    return lTemp;
}

/**************************************************************************/
void setServoAngle(float pAngle)
{
    thresholdRange(&pAngle, -45.0f, +45.0f);
    int lValue = getServoValueFromAngle((int) pAngle);
    Servo1.write(lValue);
    g_ServoAngle = pAngle;
}

float getServoAngle()
{
    return g_ServoAngle;
}
