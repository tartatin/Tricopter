/**************************************************************************/
/*                                Control                                 */
/**************************************************************************/

int16_t g_ArmsLength          = 30;   // Longueur des bras (en cm).
int16_t g_MeanForce           = 0;     // Puissance moteur moyenne demandée (en cN)
int16_t g_MotorsPosition[3][2];        // Position des moteurs, [0][1] = M1.Y, (en cm)

int16_t g_PID_Kp = 0;
int16_t g_PID_Ki = 1000;
int16_t g_PID_Kd = 0;
int16_t g_PID_N = 1;

bool g_FirstMeasure = true;
int16_t g_Previous_Rotation[3];
int32_t g_Rotation_Integral[3];

int16_t g_Rotation_Output[3];

/*****************************************************************/
void setMeanForce(int16_t pMeanForce)
{
    g_MeanForce = pMeanForce;
    thresholdRangeInt16(&g_MeanForce, 0, 2000);
}

int16_t getMeanForce()
{
    return g_MeanForce;
}

/*****************************************************************/
void updateMotorsCmd()
{
    // Application de la commande
    g_MotorsValue[0] = g_MotorsTarget[0];
    g_MotorsValue[1] = g_MotorsTarget[1];
    g_MotorsValue[2] = g_MotorsTarget[2];
    
    // Envoi des commandes
    setMotorsCmd(g_MotorsValue);
}

/*****************************************************************/
void updatePID(int16_t pTimeStep)
{
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
    
    int16_t g_Delta_Rotation[3];
    g_Delta_Rotation[0] = g_Raw_Rotation[0] - g_Previous_Rotation[0];
    g_Delta_Rotation[1] = g_Raw_Rotation[1] - g_Previous_Rotation[1];
    g_Delta_Rotation[2] = g_Raw_Rotation[2] - g_Previous_Rotation[2];
    
    g_Rotation_Output[0] = (g_PID_Kp * g_Raw_Rotation[0]) + (g_PID_Kp * g_Rotation_Integral[0]) / g_PID_Ki + (g_PID_Kp * g_PID_Kd * g_Delta_Rotation[0]) / (1 + g_PID_Kd * g_Delta_Rotation[0] / g_PID_N );
}

/*****************************************************************/
void getMotorsCorrection()
{
    int16_t lCoeff = 10; // 10000 cN.cm / 260 unité d'angle
    int16_t lArmSin120 = (g_ArmsLength * 100) / 115;
    
    int16_t OverX = (g_Raw_Rotation[0] * lCoeff) / g_ArmsLength;
    int16_t OverY = (g_Raw_Rotation[1] * lCoeff) / lArmSin120;
    
    g_MotorsTarget[0] = g_MeanForce - OverX;
    g_MotorsTarget[1] = g_MeanForce + OverX/2 - OverY;
    g_MotorsTarget[2] = g_MeanForce + OverX/2 + OverY;
}

