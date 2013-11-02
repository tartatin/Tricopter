/**************************************************************************/
/*                                Control                                 */
/**************************************************************************/

int g_ArmsLength          = 30;   // Longueur des bras (en cm).
int g_MeanForce           = 0;     // Puissance moteur moyenne demandée (en cN)
int g_MotorsPosition[3][2];        // Position des moteurs, [0][1] = M1.Y, (en cm)

/*****************************************************************/
void setMeanForce(int pMeanForce)
{
    g_MeanForce = pMeanForce;
    thresholdRangeInt(&g_MeanForce, 0, 2000);
}

int getMeanForce()
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
void getMotorsCorrection()
{
    int lCoeff = 38; // 10000 cN.cm / 260 unité d'angle
    int lArmSin120 = (g_ArmsLength * 100) / 115;
    
    int OverX = (g_Raw_Rotation[0] * lCoeff) / g_ArmsLength;
    int OverY = (g_Raw_Rotation[1] * lCoeff) / lArmSin120;
    
    g_MotorsTarget[0] = g_MeanForce - OverX;
    g_MotorsTarget[1] = g_MeanForce + OverX/2 - OverY;
    g_MotorsTarget[2] = g_MeanForce + OverX/2 + OverY;
}

