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
bool gIsComSynchronized = false;
const uint8_t gComBufferSize = (1 << 6)-1;
uint8_t gComBuffer[gComBufferSize];
uint8_t gComDataStart = 0;
uint8_t gComDataSize = 0;

uint32_t gSyncMagicNumber = 0xC3DEDEC3;

/*****************************************************************/
#define HPAR(x) (uint8_t) ( \
    ((x & 0b00000001) != 0) ^ \
    ((x & 0b00000010) != 0) ^ \
    ((x & 0b00000100) != 0) ^ \
    ((x & 0b00001000) != 0) ^ \
    ((x & 0b00010000) != 0) ^ \
    ((x & 0b00100000) != 0))

#define PAR(x) (uint8_t) ( \
    ((x & 0b00000001) != 0) ^ \
    ((x & 0b00000010) != 0) ^ \
    ((x & 0b00000100) != 0) ^ \
    ((x & 0b00001000) != 0) ^ \
    ((x & 0b00010000) != 0) ^ \
    ((x & 0b00100000) != 0) ^ \
    ((x & 0b01000000) != 0) ^ \
    ((x & 0b10000000) != 0))

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
#define MSG_SET_CORRECTION (CMD_BASE | 0x03)
#define MSG_SET_SERVO      (CMD_BASE | 0x04)
#define MSG_SET_RISE       (CMD_BASE | 0x05)
#define MSG_CMD_STOP       (CMD_BASE | 0x06)
#define MSG_SET_MIN_ACC    (CMD_BASE | 0x07)

// Retours
#define MSG_PING_RET       (RET_BASE | 0x01)
#define MSG_PLOT_RET       (RET_BASE | 0x02)
#define MSG_VA_RET         (RET_BASE | 0x03)

/*****************************************************************/
#define ID_NOOP_CMD       0
#define ID_PLOT_RET       1
#define ID_STOP_CMD       2
#define ID_VA_RET         3
#define ID_THRUST_CMD     4
#define ID_CORRECTION_CMD 5
#define ID_RISE_CMD       6
#define ID_MIN_ACC_CMD    7

/*****************************************************************/
#define CMD(x) {x, (PAR(x) << 6)}
uint8_t g_Commands[][7] = {
  CMD( MSG_CMD_NOOP       ),   // ID_NOOP_CMD
  CMD( MSG_PLOT_RET       ),   // ID_PLOT_RET
  CMD( MSG_CMD_STOP       ),   // ID_STOP_CMD
  CMD( MSG_VA_RET         ),   // ID_VA_RET
  CMD( MSG_SET_THRUST     ),   // ID_THRUST_CMD
  CMD( MSG_SET_CORRECTION ),   // ID_CORRECTION_CMD
  CMD( MSG_SET_RISE       ),   // ID_RISE_CMD
  CMD( MSG_SET_MIN_ACC    )};  // ID_MIN_ACC_CMD

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
float g_CorrectionIntensity = 0.0f;  // Intensité de la correction (en N/(m/s²)).
float g_MeanForce           = 0.0f;  // Puissance moteur moyenne demandée (en N)
float g_MinAcceleration     = 0.01f; // Accélération minimale avant action de stabilisation (en m/s²)
float g_MotorsPosition[3][2];        // Position des moteurs, [0][1] = M1.Y, (en m)

/*****************************************************************/
float g_Acceleration[3]; // Accélération instantannée, en unités légales
float g_RotationAxis[2]; // Axe de rotation instantanné, normé, Z=0
float g_OrthoAxis[2];    // Axe ortho à l'axe de rotation, dans le plan, normé, Z=0
float g_StabCoeffs[3];   // Coefficients de stabilisation pour chacun des moteurs
float g_MotorsCorr[3];   // Correction, en Newton, à apporter à chaque moteur

/*****************************************************************/
bool  g_MotorsState = false; // Autorisation ou non pour les moteurs de tourner.
float g_MotorStep = 100.0f;  // en UCM/s.
float g_MotorsTarget[3];     // Commande que les moteurs doivent atteindre (en UCM, unité de commande moteur)
float g_MotorsValue[3];      // Valeur actuelle des commandes moteurs (en UCM).

/*****************************************************************/
int g_ServoAngle; // Angle actuel du servo moteur

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
    
    sendCommand(ID_PLOT_RET, lValues, lSize);
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
    
    sendCommand(ID_PLOT_RET, lValues, lSize);
}

/*****************************************************************/
void printStatus()
{
    static uint32_t sl_LastTime = millis();
    uint32_t l_Time = millis();
    if (l_Time - sl_LastTime >= 500)
    {
        sendSynchronizeWord();
        sl_LastTime = l_Time;
    }
  
    float l_Now = (float) l_Time * 0.001f;
  
    sendPlot(3, 1, l_Now, g_Acceleration);
    sendPlot(3, 2, l_Now, g_MotorsCorr);
    sendPlot(3, 3, l_Now, g_MotorsTarget);
    
    send2DPlot(4, l_Now, g_MotorsValue[0], g_MotorsTarget[0]);
    send2DPlot(5, l_Now, g_MotorsValue[1], g_MotorsTarget[1]);
    send2DPlot(6, l_Now, g_MotorsValue[2], g_MotorsTarget[2]);
}

/*****************************************************************/
void loop() 
{
    int l_Counter = 0;
    int l_LoopDelay = 0;
    uint32_t lLast = millis();
    while(true)
    {
        // Lecture des commandes et arrêt d'urgence, et lecture des variables d'état générales
        receiveCommands();
        //updateVAMeasures();
      
        // Calcul des commandes moteurs :
        // 1. Lecture des valeurs de l'accéléromètre
        // 2. Calculs des différents axes liés à une éventuelle perte d'équilibre
        // 3. Détermination des ratios correctifs pour chaque moteur, et mise à jour des nouveaux objectifs des moteurs
        getAccelerometerData();
        updateAxis();
        getMotorsCorrection();
        
        // Mise à jour des valeurs moteurs (toutes les N boucles)
        updateMotorsCmd(millis() - lLast);
        
        // Debug
        printStatus();
        
        lLast = millis();
        // Wait
        //delay(l_LoopDelay);
        //++l_Counter;
   }
}

/*****************************************************************/
void updateMotorsCmd(uint32_t p_TimeStep)
{
    // p_TimeStep = Temps entre deux appels à cette fonction.
    float lInc = (float) p_TimeStep * 0.001f * g_MotorStep; // incrémentation pour cet appel
    float lDiffs[2];
    
    for(int m = 0; m < 3; ++m)
    {
        float lDiff = g_MotorsTarget[m] - g_MotorsValue[m];
        if (abs(lDiff) <= lInc)
        {
            g_MotorsValue[m] = g_MotorsTarget[m];
        }
        else
        {
            if (lDiff >= 0.0f)
                g_MotorsValue[m] += lInc;
            else
                g_MotorsValue[m] -= lInc;
        }
    }
    
    // Envoi des commandes
    setMotorsCmd(g_MotorsValue);
}

/*****************************************************************/
void updateAxis()
{
    // Calcul de l'axe de rotation normalisé.
    // On gère ici le cas d'un vol stable (sans accélération). Ce serait dommage de tout flinguer,
    // sous prétexte de stabiliser l'hélico !
    float lLength = sqrt(g_Acceleration[0]*g_Acceleration[0] + g_Acceleration[1]*g_Acceleration[1]);
    
    if (lLength < g_MinAcceleration)
    {
        g_Acceleration[0] = g_MinAcceleration;
        g_Acceleration[1] = 0;
        lLength           = g_MinAcceleration;
    }
    
    float lInv = 1.0f / lLength;
    g_RotationAxis[0] = +(g_Acceleration[1] * lInv);
    g_RotationAxis[1] = -(g_Acceleration[0] * lInv);
    
    // Calcul du vecteur ortho, normalisé.
    // Les signes ont une grande importance (rotation dans un sens, et pas dans l'autre,
    // sinon la correction des efforts se fera dans le mauvais sens !)
    g_OrthoAxis[0] = -g_RotationAxis[1];
    g_OrthoAxis[1] = +g_RotationAxis[0];
}

/*****************************************************************/
void getMotorsCorrection()
{
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
    for(int m = 0; m < 3; ++m)
    {
        // Calcul du coefficient
        float lValue;
        if (l_DistFromAxis[m] >= 0.0f)
            lValue = - l_DistFromAxis[m] / l_TotalPlus;
        else
            lValue = + l_DistFromAxis[m] / l_TotalNeg;
        g_StabCoeffs[m] = lValue;
            
        // Calcul de la correction en Newton correspondante
        float lCorr = g_CorrectionIntensity * lValue;
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
    float TwoThirdPi = 2.0943951f;
    for(int m = 0; m < 3; ++m)
    {
        g_MotorsPosition[m][0] = g_ArmsLength * cos(m*TwoThirdPi);
        g_MotorsPosition[m][1] = g_ArmsLength * sin(m*TwoThirdPi);
    }
}

/**************************************************************************/
/*                              Motorisation                              */
/**************************************************************************/

void setupMotors()
{
    // Moteurs
    Motor1.attach(9);
    Motor2.attach(10);
    Motor3.attach(11);
    
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
    return 94.5f * p_Force + 1233.0f;
}

/*****************************************************************/
void setMotorsCmd(float *p_Cmd)
{
    // Aucune sortie pour l'instant !!
    return;
  
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
    g_ServoAngle = 0;
    Servo1.attach(5);
    setServoAngle(0);
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
    int lValue = getServoValueFromAngle(pAngle);
    Servo1.write(lValue);
    g_ServoAngle = pAngle;
}

/**************************************************************************/
/*                             Communication                              */
/**************************************************************************/

void setupCom()
{
    Serial.begin(9600); 
}

/*****************************************************************/
void sendSynchronizeWord()
{
    Serial.write((uint8_t*) &gSyncMagicNumber, sizeof(gSyncMagicNumber));
}

/*****************************************************************/
void sendCommand(uint8_t pCmdId, uint8_t *pArgs, uint8_t pSize)
{
    uint8_t lRealSize = pSize+1;
    uint8_t lSizeParityByte = (HPAR(lRealSize) << 7);
    uint8_t lHeader         = lSizeParityByte | g_Commands[pCmdId][1] | lRealSize;
    uint8_t lId             = g_Commands[pCmdId][0];
    
    Serial.write(lHeader);
    Serial.write(lId);
    Serial.write((uint8_t*) pArgs, pSize);
}

/*****************************************************************/
void clearComBuffer()
{
    gComDataSize = 0;
    gComDataStart = 0;
}

/*****************************************************************/
void skip(uint8_t pStepBy)
{
    pStepBy = min(pStepBy, gComDataSize);
    gComDataStart = (gComDataStart + pStepBy) % gComBufferSize;
    gComDataSize -= pStepBy;
}

/*****************************************************************/
uint8_t readComData(uint8_t* pOut, uint8_t pSize, uint8_t pStepBy = 0xFF)
{
    if (pSize > gComDataSize)
        pSize = gComDataSize;
        
    // Lecture des donn�es
    uint8_t lAddress = gComDataStart;
    for(uint8_t i = 0; i < pSize; ++i)
    {
        *pOut = gComBuffer[lAddress];
        lAddress = (lAddress+1) % gComBufferSize;
        ++pOut;
    }
    
    // Step
    if (pStepBy == 0)
        return pSize;
    else if (pStepBy == 0xFF)
        skip(pSize);
    else
        skip(pStepBy);
       
    return pSize;
}

/*****************************************************************/
bool readFloat(float *pOut)
{
    return (readComData((uint8_t*) pOut, sizeof(float)) == sizeof(float));
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
#define IS_MSG(x, y) (x == g_Commands[y][0])
void treatCommand(uint8_t pCmd, uint8_t pSize)
{
    /************************ Prioritaires **********************/
    // Arrêt d'urgence
    if (IS_MSG(pCmd, ID_STOP_CMD))
    {
        quickStop();
    }

    /*********************** Asservissement**********************/    
    // Poussée
    else if (IS_MSG(pCmd, ID_THRUST_CMD))
    {
        readFloat(&g_MeanForce);
        thresholdRange(&g_MeanForce, 0.0f, 20.0f);
    }
    
    // Correction
    else if (IS_MSG(pCmd, ID_CORRECTION_CMD))
    {
        readFloat(&g_CorrectionIntensity);
        thresholdRange(&g_CorrectionIntensity, 0.0f, 2.0f);
    }
    
    // Montée
    else if (IS_MSG(pCmd, ID_RISE_CMD))
    {
        readFloat(&g_MotorStep);
        thresholdRange(&g_MotorStep, 1.0f, 1000.0f);
    }
    
    // Accélération minimale
    else if (IS_MSG(pCmd, ID_MIN_ACC_CMD))
    {
        readFloat(&g_MinAcceleration);
        thresholdRange(&g_MinAcceleration, 0.001f, 0.5f);
    }
    
    /************************** Autres ************************/
    // No-op
    else if (IS_MSG(pCmd, ID_NOOP_CMD))
    {
    }
}

/*****************************************************************/
bool searchForSynchronizeWord()
{
    while (gComDataSize >= sizeof(gSyncMagicNumber))
    {
        uint32_t lInt;
        if (readComData((uint8_t*) &lInt, sizeof(lInt), 1) < sizeof(lInt)) // lecture de 4 octets, step de 1 octet
            return false;

        if (lInt == gSyncMagicNumber)
        {
            skip(sizeof(lInt)-1); // on passe les 3 octets, pour faire dispara�tre la fin du sync word
            gIsComSynchronized = true;
            return true;
        }
    }
}

/*****************************************************************/
uint8_t receiveData()
{
    // On ne travail que si des nouvelles donn�es sont disponibles
    uint8_t lAvailable = Serial.available();
    if (lAvailable == 0)
        return 0;
    
    // On doit lire les donn�es disponibles sans d�passer le buffer.
    // La dimension du buffer est calcul�e pour ne pas d�passer la dimension max d'un message (2^6-1)
    uint8_t lToRead = min(lAvailable, gComBufferSize-gComDataSize);
    for(uint8_t i = 0; i < lToRead; ++i)
    {
        uint8_t lAddress = (gComDataStart+gComDataSize+i)%gComBufferSize;
        gComBuffer[lAddress] = Serial.read();
    }
    gComDataSize += lToRead;
    
    return lToRead;
}

/*****************************************************************/
void receiveCommands()
{
    // On ne travail que si des nouvelles donn�es sont disponibles
    if (receiveData() == 0)
        return;
    
    // Synchronisation
    if (gIsComSynchronized == false)
    {
        if (searchForSynchronizeWord() == false)
            return;
    }
    
    // Analyse des donn�es re�ues
    while (gComDataSize >= 2)
    {
        // Lecture des 2 1ers octets sans avancer dans le flux. L'avance sera faite manuellement si elle est permise, plus tard.
        uint8_t lTwoBytes[2];
        if (readComData(lTwoBytes, 2, 0) < 2)
            break;
        
        uint8_t lByte1 = lTwoBytes[0];
        uint8_t lSizeParity = (lByte1 >> 7) & 0x01;
        uint8_t lMsgParity  = (lByte1 >> 6) & 0x01;
        uint8_t lSize       = (lByte1 & 0b00111111);

        
        // V�rification des parit�s et r�cup�ration de la commande
        if (HPAR(lSize) != lSizeParity)
        {
            if (gComDataSize >= sizeof(gSyncMagicNumber))
            {
                // Lecture du message de synchro sans avancer. L'avance ne se fait que s'il s'agit effectivement du message de synchro.
                uint32_t lFourBytes;
                readComData((uint8_t*) &lFourBytes, sizeof(lFourBytes), 0);
                if (lFourBytes == gSyncMagicNumber)
                {
                    // on peut avancer en sautant les 4 octets correspondant au message de synchro.
                    skip(4);
                    continue;
                }
                else
                {
                    clearComBuffer();
                    gIsComSynchronized = false;
                    break;
                }
            }
            else
            {
                break;
            }
        }
        
        // 2�me octet : le message
        uint8_t lMsg = lTwoBytes[1];
            
        // Test du bit de parit�
        if (PAR(lMsg) != lMsgParity)
        {            
            clearComBuffer();
            gIsComSynchronized = false;
            return;
        }
        
        // Analyse du message, si on a assez de donn�es
        uint8_t lRealSize = lSize+1;
        if (gComDataSize >= lRealSize)
        {
            skip(2); // on peut passer les 2 1ers octets d�j� lus
            treatCommand(lMsg, lSize);
        }
        else
        {
            // On n'a pas assez de donn�es, on quitte et on recommencera l'analyse
            // � la prochaine fourn�e de donn�es.
            break;
        }
     }
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
  
  g_Acceleration[0] = (float) result[0] * 0.04241245; // = / 231.3f * 9.81f;
  g_Acceleration[1] = (float) result[1] * 0.04241245; // = / 231.3f * 9.81f;
  g_Acceleration[2] = (float) result[2] * 0.04241245; // = / 231.3f * 9.81f;
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


void getGyroscopeData(int * result)
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
  
  result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
  result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
  result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
  result[3] = (buff[0] << 8) | buff[1]; // temperature
  
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
