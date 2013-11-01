// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

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
int g_PingIndex;  // Indice du dernier ping envoyé
int g_PongIndex;  // Indice du dernier ping reçu

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
    
    Serial.println("Quick stop requested.");
    
    // Boucle infinie, plus rien ne se passera
    while(true);
}

/*****************************************************************/
void printStatus()
{
    /*if (g_Voltage < 11.0f)
        Serial.println("Warning : Voltage < 11V.");
    if (g_Current > 40.0f)
        Serial.println("Warning : Current > 40A.");*/
  
    static int sl_Counter = 0;
    
    ++sl_Counter;
    if (sl_Counter % 100 != 0)
        return;
    
    Serial.print("Acc  = ");
    Serial.print(g_Acceleration[0]);
    Serial.print(", ");
    Serial.print(g_Acceleration[1]);
    Serial.print(", ");
    Serial.print(g_Acceleration[2]);
    Serial.println(".");
    
    Serial.print("Corr = ");
    Serial.print(g_MotorsCorr[0]);
    Serial.print(", ");
    Serial.print(g_MotorsCorr[1]);
    Serial.print(", ");
    Serial.print(g_MotorsCorr[2]);
    Serial.println(".");
    
    Serial.print("Cmds = ");
    Serial.print(g_MotorsTarget[0]);
    Serial.print(", ");
    Serial.print(g_MotorsTarget[1]);
    Serial.print(", ");
    Serial.print(g_MotorsTarget[2]);
    Serial.println(".");
}

/*****************************************************************/
void loop() 
{
    int l_Counter = 0;
    int l_LoopDelay = 10;
    while(true)
    {
        // Lecture des commandes et arrêt d'urgence, et lecture des variables d'état générales
        readCommands();
        updateVAMeasures();
      
        // Calcul des commandes moteurs :
        // 1. Lecture des valeurs de l'accéléromètre
        // 2. Calculs des différents axes liés à une éventuelle perte d'équilibre
        // 3. Détermination des ratios correctifs pour chaque moteur, et mise à jour des nouveaux objectifs des moteurs
        getAccelerometerData();
        updateAxis();
        getMotorsCorrection();
        
        // Mise à jour des valeurs moteurs (toutes les N boucles)
        updateMotorsCmd((float) l_LoopDelay / 1000.0f);
        
        // Debug
        printStatus();
        
        // Wait
        delay(l_LoopDelay);
        ++l_Counter;
   }
}

/*****************************************************************/
void updateMotorsCmd(float p_TimeStep)
{
    // p_TimeStep = Temps entre deux appels à cette fonction.
    float lInc = p_TimeStep * g_MotorStep; // incrémentation pour cet appel
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
    
    g_RotationAxis[0] = +(g_Acceleration[1] / lLength);
    g_RotationAxis[1] = -(g_Acceleration[0] / lLength);
    
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
      Serial.println("Security switch is now ON. Motors shut down.");
    g_MotorsState = false;
}

/*****************************************************************/
void startMotors()
{
    if (g_MotorsState == false)
      Serial.println("Security switch is now OFF. Motors may start.");
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

void readCommands()
{
    int lCmd = Serial.read();
    
    // Arrêt d'urgence
    if (lCmd == '0')
    {
        quickStop(); 
    }
    
    // Contrôle de la puissance moyenne
    else if ((lCmd == 'a') || (lCmd == 'q'))
    {
        if (lCmd == 'a')
            g_MeanForce += 0.1f;
        else
            g_MeanForce -= 0.1f;
            
        Serial.print("Force = ");
        Serial.print(g_MeanForce);
        Serial.println(" N.");
    }
    
    // Contrôle de la vitesse d'évolution de la commande moteur
    else if ((lCmd == 'z') || (lCmd == 's'))
    {
        if (lCmd == 'z')
            g_MotorStep += 1.0f;
        else
            g_MotorStep -= 1.0f;
            
        Serial.print("Step = ");
        Serial.print(g_MotorStep);
        Serial.println(" UCM/s.");
    }
    
    // Commande de l'allumage ou non du bousin
    else if ((lCmd == 'p') || (lCmd == 'm'))
    {
        if (lCmd == 'p')
        {
            Serial.println("Motors will start in 5 seconds. Press any key to cancel.");
            delay(5000);
            
            if (Serial.read() == -1)
                startMotors();
        }
        else
        {
            stopMotors();
        }
    }
    
    // Contrôle de l'intensité de la correction
    else if ((lCmd == 'e') || (lCmd == 'd'))
    {
        if (lCmd == 'e')
            g_CorrectionIntensity += 0.1f;
        else
            g_CorrectionIntensity -= 0.1f;
            
        Serial.print("Intensity = ");
        Serial.print(g_CorrectionIntensity);
        Serial.println(" N/(m/s2).");
    }
    
    // Contrôle de l'accélération minimale avant réaction
    else if ((lCmd == 'r') || (lCmd == 'f'))
    {
        if (lCmd == 'r')
          g_MinAcceleration += 0.02;
        else
          g_MinAcceleration -= 0.02;
            
        Serial.print("Epsilon = ");
        Serial.print(g_MinAcceleration);
        Serial.println(" m/s2.");
    }
    
    // Commande du servomoteur
    else if ((lCmd == 't') || (lCmd == 'g'))
    {
        if (lCmd == 't')
            setServoAngle(g_ServoAngle + 1);
        else
            setServoAngle(g_ServoAngle - 1);
            
        Serial.print("Servo = ");
        Serial.print(g_ServoAngle);
        Serial.println(" deg.");
    }
    
    // Affichage de la tension
    else if (lCmd == 'v')
    {
        Serial.print("Voltage = ");
        Serial.print(g_Voltage);
        Serial.println(" V.");
        
        Serial.print("Current = ");
        Serial.print(g_Current);
        Serial.println(" A.");
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
