/**************************************************************************/
/*                                Control                                 */
/**************************************************************************/

float g_ArmsLengthFull          = 0.30f;   // Longueur des bras (en m).

float g_InvArmsLengthFull       = 1.0f / g_ArmsLengthFull;
float g_InvArmsLengthSin120     = g_InvArmsLengthFull * 1.1547f; // = 1 / (longueur * sin(120))

float g_MeanForce               = 0.0f;    // Puissance moteur moyenne demandée (en N)
float g_MotorsPosition[3][2];          // Position des moteurs, [0][1] = M1.Y, (en m)

float g_PID_Kp = 0.0f;
float g_PID_Ki = 1.0f;

bool g_FirstMeasure = true;
int16_t g_Previous_Rotation[3];
int32_t g_Rotation_Integral[3];

float g_FloatBuffer[3]; // buffer de calcul

/*****************************************************************/
void setMeanForce(float pMeanForce)
{
    g_MeanForce = pMeanForce;
    thresholdRangeFloat(&g_MeanForce, 0.0f, 20.0f);
}

float getMeanForce()
{
    return g_MeanForce;
}

/*****************************************************************/
void setPIDCoeffs(float pKp, float pKi)
{
    g_PID_Kp = pKp;
    g_PID_Ki = pKi;
    
    // seuiller !
}

/*****************************************************************/
void updatePID()
{
    if (g_MotorsState == false)
        g_FirstMeasure = true;
        
    if (g_FirstMeasure == true)
    {
        g_Previous_Rotation[0] = g_Raw_Rotation[0];
        g_Previous_Rotation[1] = g_Raw_Rotation[1];
        g_Previous_Rotation[2] = g_Raw_Rotation[2];
        
        g_Rotation_Integral[0] = g_Raw_Rotation[0];
        g_Rotation_Integral[1] = g_Raw_Rotation[1];
        g_Rotation_Integral[2] = g_Raw_Rotation[2];
        
        g_FirstMeasure = false;
        return;
    }

    // Maj de l'intégrale des erreurs
    g_Rotation_Integral[0] += g_Raw_Rotation[0];
    g_Rotation_Integral[1] += g_Raw_Rotation[1];
    g_Rotation_Integral[2] += g_Raw_Rotation[2];
 
    // Amplification de l'écart constaté   
    g_FloatBuffer[0] = (g_PID_Kp * (float)g_Raw_Rotation[0]) + (g_PID_Kp * (float)g_Rotation_Integral[0]) / g_PID_Ki; // + (g_PID_Kp * g_PID_Kd * g_Delta_Rotation[0]) / (1 + g_PID_Kd * g_Delta_Rotation[0] / g_PID_N );
    g_FloatBuffer[1] = (g_PID_Kp * (float)g_Raw_Rotation[1]) + (g_PID_Kp * (float)g_Rotation_Integral[1]) / g_PID_Ki;
    g_FloatBuffer[2] = 0.0f;
    
    // Détermination de la réaction des moteurs
    const float clCoupleCoeff = 1.0f / 260.0f; // en Nm / unité de rotation
    float OverX = clCoupleCoeff * g_FloatBuffer[0] * g_InvArmsLengthFull;
    float OverY = clCoupleCoeff * g_FloatBuffer[1] * g_InvArmsLengthSin120;
    
    g_FloatBuffer[0] = g_MeanForce - OverX;
    g_FloatBuffer[1] = g_MeanForce + OverX/2 - OverY;
    g_FloatBuffer[2] = g_MeanForce + OverX/2 + OverY;
    
    // Application de la réaction
    setMotorsTargets(g_FloatBuffer);
    
    // Stockage des valeurs actuelles
    g_Previous_Rotation[0] = g_Raw_Rotation[0];
    g_Previous_Rotation[1] = g_Raw_Rotation[1];
    g_Previous_Rotation[2] = g_Raw_Rotation[2];
}

