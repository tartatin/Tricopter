// Vol stationnaire

#include <Wire.h> // I2C library, gyroscope
#include <Servo.h>

/*****************************************************************/
// Multimètre
#define V_PIN 2
#define I_PIN 3

float g_Current;
float g_Voltage;

/*****************************************************************/
// Moteurs
Servo Motor1;
Servo Motor2;
Servo Motor3;

/*****************************************************************/
// Servo
Servo Servo1;

/*****************************************************************/
const uint8_t gEscapeChar  = 0x11;
const uint8_t gStartChar   = 0x21;
const uint8_t gStopChar    = 0x31;

bool gStartFound = false;      // Début du message trouvé ?
bool gIsEscapedChar = false;   // Le prochain caractère est spécial ?

const uint8_t gRedundancyLvl = 2;
const uint8_t gRedundancyCount = gRedundancyLvl*2-1; // = 3
uint8_t gLastChars[gRedundancyCount];
uint8_t gReadChars = 0;

const uint8_t gMsgMaxSize = 32; // taille max d'un message (hors duplication)
uint8_t gMsgBuffer[gMsgMaxSize];
uint8_t gMsgSize = 0;
bool gMsgReceived = false;

uint32_t gDropCount = 0;

/*****************************************************************/
// Messages de service
#define MSG_BASE           0x80
#define CMD_BASE           (MSG_BASE | 0x00)
#define RET_BASE           (MSG_BASE | 0x40)

#define MSG_UNDEFINED      MSG_BASE

/*****************************************************************/
// Commandes
#define MSG_CMD_NOOP       (CMD_BASE | 0x7F)
#define MSG_CMD_PING       (CMD_BASE | 0x01)
#define MSG_SET_THRUST     (CMD_BASE | 0x02)
#define MSG_SET_SERVO      (CMD_BASE | 0x04)
#define MSG_CMD_STOP       (CMD_BASE | 0x06)
#define MSG_SET_MIN_ACC    (CMD_BASE | 0x07)
#define MSG_ENGINE_CMD     (CMD_BASE | 0x08)
#define MSG_PID_CMD        (CMD_BASE | 0x09)
#define MSG_KEEP_ALIVE_CMD (CMD_BASE | 0x0A)

// Retours
#define MSG_VAL_PING       (RET_BASE | 0x01)
#define MSG_VAL_PLOT       (RET_BASE | 0x02)
#define MSG_VAL_VA         (RET_BASE | 0x03)

// Prints
#define MSG_PRINT_FLOAT    (RET_BASE | 0x04)
#define MSG_PRINT_STRING   (RET_BASE | 0x05)

/*****************************************************************/
// Accelerometer ADXL345
#define ACC (0x53)    //ADXL345 ACC address
#define A_TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

// Gyroscope ITG3200 
#define GYRO 0x69 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E

#define G_TO_READ 8 // 2 bytes for each axis x, y, z

// offsets are chip specific. 
int g_offx = 120;
int g_offy = 20;
int g_offz = 93;

/*****************************************************************/
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
// Données instantannées
float g_Raw_RotationRate[3]; // Vitesse angulaire, en rad/s
float g_Raw_Acceleration[3]; // Accélération instantannée, en unités légales

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
bool  g_MotorsState = false; // Autorisation ou non pour les moteurs de tourner.
float g_MotorsTarget[3];     // Commande que les moteurs doivent atteindre (en UCM, unité de commande moteur)
float g_MotorsValue[3];      // Valeur actuelle des commandes moteurs (en UCM).

/*****************************************************************/
float g_ServoAngle; // Angle actuel du servo moteur

/*****************************************************************/
/*                            Main                               */
/*****************************************************************/
void setup() 
{   
    // Communication
    setupCom();
  
    // Capteurs
    setupI2C();
    setupIMU();
    setupVAMeter();
  
    // Moteurs & servo
    setupMotors();
    setupServo();
  
    // Stabilisateur
    setupStabilizer();
  
    // Fin du setup
    printText("Setup ok.\n");
}

/*****************************************************************/
void quickStop()
{
    // Arrêt total des moteurs
    stopMotors();
    
    // Boucle infinie, plus rien ne se passera
    while(true);
}

/*****************************************************************/
void sendPlot(uint8_t pDim, uint8_t pPlotID, float pTime, float *pValues)
{
    const uint8_t lSize = 1+(1+pDim)*sizeof(float);
    uint8_t lValues[lSize];
    float *lFloats = (float*) (&lValues[1]);
    
    lValues[0] = pPlotID;
    lFloats[0] = pTime;
    for(uint8_t i = 0; i < pDim; ++i)
        lFloats[1+i] = pValues[i];
    
    sendCommand(MSG_VAL_PLOT, lValues, lSize);
}

/*****************************************************************/
void send2DPlot(uint8_t pPlotID, float pTime, float pV1, float pV2)
{
    const uint8_t lSize = 1+3*sizeof(float);
    uint8_t lValues[lSize];
    float *lFloats = (float*) (&lValues[1]);
    
    lValues[0] = pPlotID;
    lFloats[0] = pTime;
    lFloats[1] = pV1;
    lFloats[2] = pV2;
    
    sendCommand(MSG_VAL_PLOT, lValues, lSize);
}

/*****************************************************************/
void printStatus()
{
    float l_Now = (float) millis() * 0.001f;
    
    static uint32_t lLast = millis();
    uint32_t lNow = millis();
    
    if (lNow - lLast < 10)
    {
        return;
    }
    lLast = lNow;
  
    //sendPlot(3, 1, l_Now, g_Raw_Acceleration);
  
    sendPlot(3, 1, l_Now, g_P_Acceleration);
    sendPlot(3, 2, l_Now, g_I_Acceleration);
    sendPlot(3, 3, l_Now, g_D_Acceleration);
    
    /*send2DPlot( 4, l_Now, g_MotorsValue[0], g_MotorsTarget[0]);
    send2DPlot( 5, l_Now, g_MotorsValue[1], g_MotorsTarget[1]);
    send2DPlot( 6, l_Now, g_MotorsValue[2], g_MotorsTarget[2]);*/
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
void loop() 
{
    static uint32_t lLast = micros();
    uint32_t lDiff = 0;
    
    // Lecture des commandes et arrêt d'urgence
    while (receiveMsg())
        treatMessage();
    
    // Lecture des variables d'état générales
    //updateVAMeasures();
  
    // Calcul des commandes moteurs :
    // 1. Lecture des valeurs de l'accéléromètre
    // 2. Calculs des différents axes liés à une éventuelle perte d'équilibre
    // 3. Détermination des ratios correctifs pour chaque moteur, et mise à jour des nouveaux objectifs des moteurs
    getAccelerometerData();
    getGyroscopeData();
    
    // Filtrage, dérivation et intégration
    filterMeasures();
    computePID(lDiff);
    keepMeasures();
    
    // Calcul de la commande
    //computeInput();
    //updateAxis();
    //getMotorsCorrection();
    
    // Mise à jour des valeurs moteurs
    //updateMotorsCmd(lDiff);
    
    // Debug
    //printStatus();
    
    // Calcul des intervalles de temps
    uint32_t lNow = micros();
    if (lNow >= lLast)
        lDiff = lNow-lLast;
    else
        lDiff = lNow + (0xFFFFFFFF-lLast);
    lLast = lNow;
    
    // Délai pour un pas de 10 ms
    /*if (lDiff < 2000)
      delayMicroseconds(2000 - lDiff);*/
}

/*****************************************************************/
void updateMotorsCmd(uint32_t p_TimeStep)
{
    // Application de la commande
    g_MotorsValue[0] = g_MotorsTarget[0];
    g_MotorsValue[1] = g_MotorsTarget[1];
    g_MotorsValue[2] = g_MotorsTarget[2];
    
    // Envoi des commandes
    setMotorsCmd(g_MotorsValue);
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
}

/**************************************************************************/
/*                              Motorisation                              */
/**************************************************************************/

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
    int lValue = 900;
    
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
      ; //Serial.println("Security switch is now ON. Motors shut down.");
    g_MotorsState = false;
}

/*****************************************************************/
void startMotors()
{
    if (g_MotorsState == false)
      ; //Serial.println("Security switch is now OFF. Motors may start.");
    g_MotorsState = true;
}

/*****************************************************************/
float getMotorCmdFromForce(float p_Force)
{
    // D'après courbes empiriques
    float lValue = 94.5f * p_Force + 1233.0f;
    return min(max(lValue, 1230.0f), 1650.0f);
}

/*****************************************************************/
void setMotorsCmd(float *p_Cmd)
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

/**************************************************************************/
/*                                 Servo                                  */
/**************************************************************************/

void setupServo()
{
    g_ServoAngle = 0.0f;
    Servo1.attach(5);
    setServoAngle(0.0f);
}

int getServoValueFromAngle(int pAngle)
{
    int lTemp = pAngle*pAngle - 169*pAngle + 7897;
    lTemp = (int) (lTemp / 126);
    lTemp = max(min(lTemp, 160), 16);
    
    return lTemp;
}

void setServoAngle(float pAngle)
{
    int lValue = getServoValueFromAngle((int) pAngle);
    Servo1.write(lValue);
    g_ServoAngle = pAngle;
}

/**************************************************************************/
/*                             Communication                              */
/**************************************************************************/

void setupCom()
{
    Serial.begin(115200);
    Serial.setTimeout(1);
    
    printText("Communication ok.\n");
}

/*****************************************************************/
void writeComSpecial(uint8_t pChar)
{
    for(uint8_t i = 0; i < gRedundancyCount; ++i)
    {
        Serial.write(&pChar, 1);
    }
}

/*****************************************************************/
void writeComData(uint8_t *pData, uint8_t pSize)
{
    for(uint8_t i = 0; i < pSize; ++i)
    {
        uint8_t lChar = *pData;
        
        // Ecriture du caractère d'échappement si nécessaire
        if ((lChar == gEscapeChar) || (lChar == gStartChar) || (lChar == gStopChar))
            writeComSpecial(gEscapeChar);
        
        // Ecriture du caractère
        writeComSpecial(lChar);
        
        // Caractère suivant
        pData++;
    }
}

/*****************************************************************/
void prepareCommand(uint8_t pCmdId)
{
    writeComSpecial(gStartChar);
    writeComData(&pCmdId, 1);
}

/*****************************************************************/
void endCommand()
{
    writeComSpecial(gStopChar);
}

/*****************************************************************/
void appendParams(uint8_t *pArgs, uint8_t pSize)
{
    writeComData(pArgs, pSize);
}

/*****************************************************************/
void sendCommand(uint8_t pCmdId, uint8_t *pArgs, uint8_t pSize)
{
    prepareCommand(pCmdId);
    if (pArgs != NULL)
        appendParams(pArgs, pSize);
    endCommand();
}

/*****************************************************************/
void printFloat(float pValue)
{
    sendCommand(MSG_PRINT_FLOAT, (uint8_t*) &pValue, sizeof(float));
}

/*****************************************************************/
void printText(char *pTxt)
{
    prepareCommand(MSG_PRINT_STRING);
    const uint8_t lSize = min(gMsgMaxSize-1, strlen(pTxt));
    appendParams((uint8_t*) pTxt, lSize);
    endCommand();
}

/*****************************************************************/
void thresholdRange(float *pValue, float pMin, float pMax)
{
    if (*pValue < pMin)
        *pValue = pMin;
    if (*pValue > pMax)
        *pValue = pMax;
}

/*****************************************************************/
#define IS_MSG(y) (gMsgBuffer[0] == y)
#define CHECK_SIZE(s) {if(gMsgSize != (s+1)) {gDropCount++; return false;}}

#define UINT32_PTR(offset) ((uint32_t*) &gMsgBuffer[offset+1])
#define BOOL_PTR(offset) ((bool*) &gMsgBuffer[offset+1])
#define FLOAT_PTR(offset) ((float*) &gMsgBuffer[offset+1])

#define UINT32(offset) *UINT32_PTR(offset)
#define BOOL(offset) *BOOL_PTR(offset)
#define FLOAT(offset) *FLOAT_PTR(offset)

/*****************************************************************/
bool treatMessage()
{
    if (gMsgReceived == false)
        return false;
    gMsgReceived = false;
  
    /************************ Prioritaires **********************/
    // Arrêt d'urgence
    if (IS_MSG(MSG_CMD_STOP))
    {
        CHECK_SIZE(0);
        printText("MSG_CMD_STOP\n");
        //quickStop();
    }
    
    // Commande moteur
    else if (IS_MSG(MSG_ENGINE_CMD))
    {
        CHECK_SIZE(sizeof(uint32_t));
        printText("MSG_ENGINE_CMD\n");
        /*uint32_t *lValue = UINT32_PTR(0);

        if (*lValue == 0x12345678)
        {
            g_MotorsState = true;
            printText("Allumage.\x00");
        }
        else
        {
            g_MotorsState = false;
        }*/
    }

    /*********************** Asservissement**********************/    
    // Poussée
    else if (IS_MSG(MSG_SET_THRUST))
    {
        CHECK_SIZE(sizeof(float));
        printText("MSG_SET_THRUST\n");
        /*g_MeanForce = FLOAT(0);
        thresholdRange(&g_MeanForce, 0.0f, 20.0f);
        printFloat(g_MeanForce);*/
    }
    
    // Paramètre du correcteur PID
    else if (IS_MSG(MSG_PID_CMD))
    {
        CHECK_SIZE(3*sizeof(float));
        printText("MSG_PID_CMD\n");
        
        /*g_P_Coeff = FLOAT(0*sizeof(float));
        g_I_Coeff = FLOAT(1*sizeof(float));
        g_D_Coeff = FLOAT(2*sizeof(float));

        thresholdRange(&g_P_Coeff, -100.0f, +100.0f);
        thresholdRange(&g_I_Coeff, -100.0f, +100.0f);
        thresholdRange(&g_D_Coeff, -100.0f, +100.0f);
        
        printFloat(g_P_Coeff);
        printFloat(g_I_Coeff);
        printFloat(g_D_Coeff);*/
    }
    
    // Accélération minimale
    else if (IS_MSG(MSG_SET_MIN_ACC))
    {
        CHECK_SIZE(sizeof(float));
        printText("MSG_SET_MIN_ACC\n");
        /*g_MinAcceleration = FLOAT(0);
        thresholdRange(&g_MinAcceleration, 0.001f, 0.5f);*/
    }
    
    // Servo
    else if (IS_MSG(MSG_SET_SERVO))
    {
        CHECK_SIZE(sizeof(float));
        printText("MSG_SET_SERVO\n");
        /*float *lAngle = FLOAT_PTR(0);
        thresholdRange(lAngle, -45.0f, +45.0f);
        setServoAngle(*lAngle);*/
    }
    
    /************************** Autres ************************/
    // No-op
    else if (IS_MSG(MSG_CMD_NOOP))
    {
        CHECK_SIZE(0);
        printText("MSG_CMD_NOOP\n");
    }
    
    // Ping
    else if (IS_MSG(MSG_CMD_PING))
    {
        CHECK_SIZE(0);
        printText("MSG_CMD_PING\n");
    }
    
    // Un message reçu
    return true;
}

/*****************************************************************/
bool receiveMsg()
{
    gStartFound = false;
    gMsgReceived = false;
    gIsEscapedChar = false;
    gMsgSize = 0;
    
    /*if (Serial.available() > 0)
    {
        // Délai nécessaire pour que le buffer du serial se remplisse ?!
        delay(50);
    }*/
                
    // On ne travail que si des nouvelles donn?es sont disponibles
    while( true ) //Serial.available() > 0 )
    {
        delay(50);
        
        if (gReadChars < gRedundancyCount)
        {
            int lActuallyRead = Serial.readBytes((char*) &gLastChars[gReadChars], gRedundancyCount-gReadChars);
            if (lActuallyRead <= 0)
                break; // on reviendra plus tard, en attendant les données sont conservées dans gLastChars
            gReadChars += lActuallyRead;
        }
        
        // On doit avoir lu les 3 chars consécutifs
        if (gReadChars < gRedundancyCount)
            break;
        gReadChars = 0;
            
        // Les 3 chars consécutifs doivent être identiques
        //    à 100% si aucun message n'a commencé,
        //    à 66% si un message a commencé
        char lChar = 0x00;
        if (gStartFound)
        {
            bool lFound = false;
            for(uint8_t i = 0; i < gRedundancyLvl; ++i)
            {
                uint8_t lSameCount = 0;
                for(uint8_t j = i+1; j < gRedundancyCount; ++j)
                {
                    if (gLastChars[i] == gLastChars[j])
                        lSameCount++;
                }
                if (lSameCount >= gRedundancyLvl)
                {
                    lChar = gLastChars[i];
                    lFound = true;
                    break;
                }
            }
            
            //printText("receiveMsg - 11\n");
            if (lFound == false)
            {
                // Les données ne sont pas consistantes, donc un morceau des données est mauvais, donc le message est foutu.
                gStartFound = false;
                gMsgReceived = false;
                gIsEscapedChar = false;
                gMsgSize = 0;
                gDropCount++;
                
                // Passage au caractère suivant
                continue;
            }
        }
        else
        {
            bool lAreAllSame = true;
            for(uint8_t i = 1; i < gRedundancyCount; ++i)
            {
                if (gLastChars[0] != gLastChars[i])
                {
                    lAreAllSame = false;
                    break;
                }
            }
            
            if (lAreAllSame)
            {
                lChar = gLastChars[0];
            }
            else
            {
                // aucun message n'a commencé, alors on est simplement désynchro, auquel cas on ne supprime que le 1er octet,
                gReadChars = gRedundancyCount-1;
                for(uint8_t i = 0; i < gRedundancyCount-1; ++i)
                    gLastChars[i] = gLastChars[i+1];
                    
                continue;
            }
        }

            
        // Deux cas : soit on attend le début du message, soit on est dedans
        if (gStartFound)
        {
            // On lit les données petit à petit
            if (gIsEscapedChar)
            {
                // quel que soit le caractère suivant, on l'insère comme un caractère de donnée
                gMsgBuffer[gMsgSize] = lChar;
                gIsEscapedChar = false;
                gMsgSize++;
            }
            else
            {
                //printText("receiveMsg - 17\n");
                if (lChar == gEscapeChar)
                {
                    // Il s'agit d'un caractère d'échappement
                    gIsEscapedChar = true;
                }
                else if (lChar == gStopChar)
                {
                    // Caractère de fin
                    gMsgReceived = true;
                    gStartFound = false;
                }
                else
                {
                    // Caractère normal, on insère
                    gMsgBuffer[gMsgSize] = lChar;
                    gMsgSize++;
                }
            }
        }
        else
        {
            if (lChar == gStartChar)
            {
                gStartFound = true;
                gMsgReceived = false;
                gIsEscapedChar = false;
                gMsgSize = 0;
            }
        }
        
        // A ce stade il est possible qu'un message complet ait été obtenu, si c'est le cas on doit cesser de chercher le message,
        // quitter, et attendre que celui-ci soit traité dans la boucle principale de l'application.
        if (gMsgReceived)
        {
            break;
        }
    }
    
    return gMsgReceived;
}

/**************************************************************************/
/*                                  IMU                                   */
/**************************************************************************/

void setupIMU()
{
  initAcc();
  initGyro();
}

/*****************************************************************/
void initAcc() {
  //Turning on the ADXL345
  i2cWriteTo(ACC, 0x2D, 0);      
  i2cWriteTo(ACC, 0x2D, 16);
  i2cWriteTo(ACC, 0x2D, 8);
  //by default the device is in +-2g range reading
}

/*****************************************************************/
void getAccelerometerData() {
  int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  byte buff[A_TO_READ];
  
  i2cReadFrom(ACC, regAddress, A_TO_READ, buff); //read the acceleration data from the ADXL345
  
  //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  //thus we are converting both bytes in to one int
  int result[3];
  result[0] = (((int)buff[1]) << 8) | buff[0];   
  result[1] = (((int)buff[3]) << 8) | buff[2];
  result[2] = (((int)buff[5]) << 8) | buff[4];
  
  g_Raw_Acceleration[0] = (float) result[0] * 0.04241245; // = / 231.3f * 9.81f;
  g_Raw_Acceleration[1] = (float) result[1] * 0.04241245; // = / 231.3f * 9.81f;
  g_Raw_Acceleration[2] = (float) result[2] * 0.04241245; // = / 231.3f * 9.81f;
}

/*****************************************************************/
//initializes the gyroscope
void initGyro()
{
  /*****************************************
  * ITG 3200
  * power management set to:
  * clock select = internal oscillator
  *     no reset, no sleep mode
  *   no standby mode
  * sample rate to = 125Hz
  * parameter to +/- 2000 degrees/sec
  * low pass filter = 5Hz
  * no interrupt
  ******************************************/
  i2cWriteTo(GYRO, G_PWR_MGM, 0x00);
  i2cWriteTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
  i2cWriteTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  i2cWriteTo(GYRO, G_INT_CFG, 0x00);
}


void getGyroscopeData()
{
    /**************************************
    Gyro ITG-3200 I2C
    registers:
    temp MSB = 1B, temp LSB = 1C
    x axis MSB = 1D, x axis LSB = 1E
    y axis MSB = 1F, y axis LSB = 20
    z axis MSB = 21, z axis LSB = 22
    *************************************/
  
    int regAddress = 0x1B;
    int temp, x, y, z;
    byte buff[G_TO_READ];
    
    i2cReadFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
    
    int result[4];
    result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
    result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
    result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
    result[3] = (buff[0] << 8) | buff[1]; // temperature
    
    g_Raw_RotationRate[0] = (float) result[0] * 0.001065264f; // = 2000 °/s / 32768 * 2pi / 360
    g_Raw_RotationRate[1] = (float) result[1] * 0.001065264f; // = 2000 °/s / 32768 * 2pi / 360
    g_Raw_RotationRate[2] = (float) result[2] * 0.001065264f; // = 2000 °/s / 32768 * 2pi / 360
}

/**************************************************************************/
/*                                  I2C                                   */
/**************************************************************************/

void setupI2C()
{
    Wire.begin();
}

/*****************************************************************/
//Writes val to address register on ACC
void i2cWriteTo(int DEVICE, byte address, byte val) {
   Wire.beginTransmission(DEVICE); //start transmission to ACC 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

/*****************************************************************/
//reads num bytes starting from address register on ACC in to buff array
void i2cReadFrom(int DEVICE, byte address, int num, byte buff[]) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC
  
  int i = 0;
  while(Wire.available())    //ACC may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

/**************************************************************************/
/*                              Multimètre                                */
/**************************************************************************/

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
