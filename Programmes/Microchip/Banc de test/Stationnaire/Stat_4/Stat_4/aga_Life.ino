/**************************************************************************/
/*                              Multimètre                                */
/**************************************************************************/

// Multimètre
#define V_PIN 2
#define I_PIN 3

float g_Current;
float g_Voltage;

/*****************************************************************/
void setupVAMeter()
{
    // Multimètre
    pinMode(V_PIN, INPUT);
    pinMode(I_PIN, INPUT);
    
    // Pull up activée si high
    digitalWrite(V_PIN, LOW);
    digitalWrite(I_PIN, LOW);
}

/*****************************************************************/
void updateVAMeasures()
{
     g_Voltage = (analogRead(V_PIN) * 0.020152f);
     g_Current = (analogRead(I_PIN) * 0.066705f);
}
