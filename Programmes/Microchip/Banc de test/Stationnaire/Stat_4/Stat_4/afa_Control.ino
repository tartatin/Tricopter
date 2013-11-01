/**************************************************************************/
/*                                Control                                 */
/**************************************************************************/

float g_ArmsLength          = 0.3f;  // Longueur des bras (en m).
float g_MeanForce           = 0.0f;  // Puissance moteur moyenne demandée (en N)
float g_MinAcceleration     = 0.01f; // Accélération minimale avant action de stabilisation (en m/s²)
float g_MotorsPosition[3][2];        // Position des moteurs, [0][1] = M1.Y, (en m)

/*****************************************************************/
// Données PID
float g_P_Acceleration[3];
float g_D_Acceleration[3];
float g_I_Acceleration[3];

/*****************************************************************/
// Données précédentes
bool  g_HasPreviousData = false;
float g_Prev_Filtered_Acceleration[3];
float g_Prev_Raw_Acceleration[3];

/*****************************************************************/
// Données filtrées
float g_Filtered_RotationRate[3]; // Vitesse angulaire, en rad/s
float g_Filtered_Acceleration[3]; // Accélération instantannée, en unités légales

/*****************************************************************/
// Paramètres PID
float g_P_Coeff = 1.0f;
float g_I_Coeff = 0.1f;
float g_D_Coeff = 0.2;

/*****************************************************************/
// Commandes
float g_Input[3];        // Vecteur issu du PID

float g_RotationAxis[2]; // Axe de rotation instantanné, normé, Z=0
float g_OrthoAxis[2];    // Axe ortho à l'axe de rotation, dans le plan, normé, Z=0
float g_StabCoeffs[3];   // Coefficients de stabilisation pour chacun des moteurs
float g_MotorsCorr[3];   // Correction, en Newton, à apporter à chaque moteur
bool  g_ApplyCorr;       // Indique si des corrections doivent être appliquées
float g_InputLength; // Norme de l'accélération projetée

/*****************************************************************/
void setMeanForce(float pMeanForce)
{
    g_MeanForce = pMeanForce;
    thresholdRange(&g_MeanForce, 0.0f, 20.0f);
}

float getMeanForce()
{
    return g_MeanForce;
}

/*****************************************************************/
void setPIDCoeffs(float pP, float pI, float pD)
{
    g_P_Coeff = pP;
    g_I_Coeff = pI;
    g_D_Coeff = pD;
    
    thresholdRange(&g_P_Coeff, -100.0f, +100.0f);
    thresholdRange(&g_I_Coeff, -100.0f, +100.0f);
    thresholdRange(&g_D_Coeff, -100.0f, +100.0f);
}

void getPIDCoeffs(float *pP, float *pI, float *pD)
{
    *pP = g_P_Coeff;
    *pI = g_I_Coeff;
    *pD = g_D_Coeff;
}

/*****************************************************************/
void filterMeasures()
{
    // Calcul de l'accélération filtrée
    g_Filtered_Acceleration[0] = g_Raw_Acceleration[0]; //(g_Prev_Raw_Acceleration[0] + g_Raw_Acceleration[0]) / 2.0f;
    g_Filtered_Acceleration[1] = g_Raw_Acceleration[1]; //(g_Prev_Raw_Acceleration[1] + g_Raw_Acceleration[1]) / 2.0f;
    g_Filtered_Acceleration[2] = g_Raw_Acceleration[2]; //(g_Prev_Raw_Acceleration[2] + g_Raw_Acceleration[2]) / 2.0f;
    
    // Calcul de la différence avec la commande
    g_Filtered_Acceleration[2] = 0.0f; // TODO : calculer la vraie différence
}

/*****************************************************************/
void computePID(uint32_t pMicroTimeStep)
{
    g_P_Acceleration[0] = g_Filtered_Acceleration[0];
    g_P_Acceleration[1] = g_Filtered_Acceleration[1];
    g_P_Acceleration[2] = g_Filtered_Acceleration[2];
  
    // Calcul de la dérivée instantannée et de l'intégrale
    if (g_HasPreviousData == false)
    {
        g_D_Acceleration[0] = 0.0f;
        g_D_Acceleration[1] = 0.0f;
        g_D_Acceleration[2] = 0.0f;
        
        g_I_Acceleration[0] = 0.0f;
        g_I_Acceleration[1] = 0.0f;
        g_I_Acceleration[2] = 0.0f;
        
        return;
    }
    
    float lTime = (float) pMicroTimeStep * 0.000001f;
    float lInvTime = 1.0f / lTime;
    
    g_D_Acceleration[0]  = (g_Filtered_Acceleration[0] - g_Prev_Filtered_Acceleration[0]) * lInvTime;
    g_D_Acceleration[1]  = (g_Filtered_Acceleration[1] - g_Prev_Filtered_Acceleration[1]) * lInvTime;
    g_D_Acceleration[2]  = (g_Filtered_Acceleration[2] - g_Prev_Filtered_Acceleration[2]) * lInvTime;
    
    g_I_Acceleration[0] += g_P_Acceleration[0] * lTime;
    g_I_Acceleration[1] += g_P_Acceleration[1] * lTime;
    g_I_Acceleration[2] += g_P_Acceleration[2] * lTime;
}

/*****************************************************************/
void keepMeasures()
{
    g_HasPreviousData = true;
  
    // conserve les mesures d'une frame à l'autre
    g_Prev_Raw_Acceleration[0] = g_Raw_Acceleration[0];
    g_Prev_Raw_Acceleration[1] = g_Raw_Acceleration[1];
    g_Prev_Raw_Acceleration[2] = g_Raw_Acceleration[2];
    
    g_Prev_Filtered_Acceleration[0] = g_Filtered_Acceleration[0];
    g_Prev_Filtered_Acceleration[1] = g_Filtered_Acceleration[1];
    g_Prev_Filtered_Acceleration[2] = g_Filtered_Acceleration[2];
}

/*****************************************************************/
void updateMotorsCmd(uint32_t p_TimeStep)
{
    // Application de la commande
    g_MotorsValue[0] = g_MotorsTarget[0];
    g_MotorsValue[1] = g_MotorsTarget[1];
    g_MotorsValue[2] = g_MotorsTarget[2];
    
    // Envoi des commandes
    //setMotorsCmd(g_MotorsValue);
}

/*****************************************************************/
void computeInput()
{
    g_Input[0] = (g_P_Coeff * g_P_Acceleration[0]) + (g_I_Coeff * g_I_Acceleration[0]) + (g_D_Coeff * g_D_Acceleration[0]);
    g_Input[1] = (g_P_Coeff * g_P_Acceleration[1]) + (g_I_Coeff * g_I_Acceleration[1]) + (g_D_Coeff * g_D_Acceleration[1]);
    g_Input[2] = (g_P_Coeff * g_P_Acceleration[2]) + (g_I_Coeff * g_I_Acceleration[2]) + (g_D_Coeff * g_D_Acceleration[2]);
}

/*****************************************************************/
void updateAxis()
{
    // Calcul de l'axe de rotation normalisé.
    // On gère ici le cas d'un vol stable (sans accélération). Ce serait dommage de tout flinguer,
    // sous prétexte de stabiliser l'hélico !
    g_InputLength = sqrt(g_Input[0]*g_Input[0] + g_Input[1]*g_Input[1]);
    
    if (g_InputLength < g_MinAcceleration)
    {
        g_ApplyCorr = false;
        return;
    }
    else
       g_ApplyCorr = true;
       
    float lInv = 1.0f / g_InputLength;
    g_OrthoAxis[0] = g_Input[0] * lInv;
    g_OrthoAxis[1] = g_Input[1] * lInv;
}

/*****************************************************************/
void getMotorsCorrection()
{
    if (g_ApplyCorr == false)
    {
        float lForce = getMotorCmdFromForce(g_MeanForce);
        for(int m = 0; m < 3; ++m)
            g_MotorsTarget[m] = lForce;
        return;
    }
  
    // Calcul des distances de chaque moteur à l'axe ortho normé, et sommation des totaux
    // positifs et négatifs séparément.
    float l_DistFromAxis[3];
    float l_TotalPlus = 0.0f;
    float l_TotalNeg  = 0.0f;
    
    for(int m = 0; m < 3; ++m)
    {
        float lValue = g_MotorsPosition[m][0]*g_OrthoAxis[0] + g_MotorsPosition[m][1]*g_OrthoAxis[1];
        l_DistFromAxis[m] = lValue;
        if (lValue >= 0.0f)
            l_TotalPlus += lValue;
        else
            l_TotalNeg += lValue;
    }
    
    // Calcul des coefficients, correspondant à l'importance de chaque moteur par rapport à son groupe
    // (groupe négatif, ou groupe positif, suivant le sens dans lequel l'hélico doit rotater).
    float l_InvTotalPlus = 1.0f / l_TotalPlus;
    float l_InvTotalNeg  = 1.0f / l_TotalNeg;
    
    for(int m = 0; m < 3; ++m)
    {
        // Calcul du coefficient
        float lValue;
        if (l_DistFromAxis[m] >= 0.0f)
            lValue = - l_DistFromAxis[m] * l_InvTotalPlus;
        else
            lValue = + l_DistFromAxis[m] * l_InvTotalNeg;
        g_StabCoeffs[m] = lValue;
            
        // Calcul de la correction en Newton correspondante
        float lCorr = lValue * g_InputLength;
        g_MotorsCorr[m] = lCorr;
        
        float lForce = max(lCorr + g_MeanForce, 0.0f);
        g_MotorsTarget[m] = getMotorCmdFromForce(lForce);
    }
    
    // On applique une correction au moteur 1, pour prendre en compte son orientation (l'effort calculé doit correspondre à la projetée
    // sur l'axe Z de l'effort fourni par le moteur).
    g_MotorsTarget[0] = g_MotorsTarget[0] / cos(g_ServoAngle * 0.0174532925f);
}

/*****************************************************************/
void setupStabilizer()
{
    const float TwoThirdPi = 2.0943951f;
    const float lCoses[3] = {cos(0.0f), cos(TwoThirdPi), cos(2.0f * TwoThirdPi)};
    const float lSines[3] = {sin(0.0f), sin(TwoThirdPi), sin(2.0f * TwoThirdPi)};
    
    for(int m = 0; m < 3; ++m)
    {
        g_MotorsPosition[m][0] = g_ArmsLength * lCoses[m];
        g_MotorsPosition[m][1] = g_ArmsLength * lSines[m];
    }
    
    g_P_Coeff = 0.0f;
    g_I_Coeff = 0.0f;
    g_D_Coeff = 0.0f;
}


