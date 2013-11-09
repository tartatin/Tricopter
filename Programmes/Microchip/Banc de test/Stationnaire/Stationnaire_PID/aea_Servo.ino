/**************************************************************************/
/*                                 Servo                                  */
/**************************************************************************/

#include <Servo.h>

/*****************************************************************/
// Servo
Servo Servo1;
int16_t g_ServoAngle; // Angle actuel du servo moteur

/**************************************************************************/
void setupServo()
{
    g_ServoAngle = 0.0f;
    Servo1.attach(5);
    setServoAngle(0.0f);
}

/**************************************************************************/
int16_t getServoValueFromAngle(int pAngle)
{
    int16_t lTemp = pAngle*pAngle - 169*pAngle + 7897;
    lTemp = (int16_t) (lTemp / 126);
    lTemp = max(min(lTemp, 160), 16);
    
    return lTemp;
}

/**************************************************************************/
void setServoAngle(int16_t pAngle)
{
    thresholdRangeInt16(&pAngle, -45, +45);
    int16_t lValue = getServoValueFromAngle(pAngle);
    Servo1.write(lValue);
    g_ServoAngle = pAngle;
}

int16_t getServoAngle()
{
    return g_ServoAngle;
}
