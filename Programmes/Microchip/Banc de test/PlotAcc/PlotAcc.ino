// Communication avec le PC.

/*****************************************************************/
#include <Wire.h> // I2C library, gyroscope
#include <Servo.h>

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
float g_Acceleration[3]; // Accélération instantannée, en unités légales

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
#define MSG_NOOP_CMD       (CMD_BASE | 0x7F)
#define MSG_PING_CMD       (CMD_BASE | 0x01)
#define MSG_THRUST_CMD     (CMD_BASE | 0x02)
#define MSG_CORRECTION_CMD (CMD_BASE | 0x03)
#define MSG_SERVO_CMD      (CMD_BASE | 0x04)
#define MSG_RISE_CMD       (CMD_BASE | 0x05)
#define MSG_STOP_CMD       (CMD_BASE | 0x06)

// Retours
#define MSG_PING_RET       (RET_BASE | 0x01)
#define MSG_PLOT_RET       (RET_BASE | 0x02)

/*****************************************************************/
#define ID_NOOP_CMD 0
#define ID_PLOT_RET 1
#define ID_STOP_CMD 2

/*****************************************************************/
#define CMD(x) {x, (PAR(x) << 6)}
uint8_t gCommands[][3] = {
  CMD( MSG_NOOP_CMD ),   // ID_NOOP_CMD
  CMD( MSG_PLOT_RET ),   // ID_PLOT_RET
  CMD( MSG_STOP_CMD )};  // ID_STOP_CMD

/*****************************************************************/
void blinkLed(uint8_t pCount, uint8_t pDelay, uint8_t pDelay2)
{
    delay(pDelay2);
    for(uint8_t i = 0; i < pCount; ++i)
    {
        digitalWrite(13, HIGH); 
        delay(pDelay);          
        digitalWrite(13, LOW);  
        delay(pDelay);          
    }
    delay(pDelay2);
}

void dbg()
{
    delay(3000);          
    digitalWrite(13, HIGH); 
    delay(1000);          
    digitalWrite(13, LOW);  
    delay(3000);          
}

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
  
  // Debug
  pinMode(13, OUTPUT);   
}

/*****************************************************************/
void loop() 
{
    Serial.write((uint8_t*) &gSyncMagicNumber, sizeof(gSyncMagicNumber));
  
    int lCounter = 0;
    float lTime = 0.0f;
    while(true)
    {
        // Communication
        receiveCommands();
        
        // Accélération
        getAccelerometerData();
        
        // Envoi des infos
        const uint8_t lSize = 1+4*sizeof(float);
        uint8_t lArgs[lSize];
        float *lFloats = (float*) &lArgs[1];
        lArgs[0]   = 0x01;
        lFloats[0] = lTime;
        lFloats[1] = g_Acceleration[0];        
        lFloats[2] = g_Acceleration[1];
        lFloats[3] = g_Acceleration[2];
        
        sendCommand(ID_PLOT_RET, lArgs, lSize);

        delay(50);
        lTime += 0.05f;
        
        lCounter++;
        if (lCounter == 16)
        {
            lCounter = 0;
            Serial.write((uint8_t*) &gSyncMagicNumber, sizeof(gSyncMagicNumber));
        }
   }
}

/**************************************************************************/
/*                             Communication                              */
/**************************************************************************/

void setupCom()
{
    Serial.begin(9600); 
}

void sendCommand(uint8_t pCmdId, uint8_t *pArgs, uint8_t pSize)
{
    uint8_t lRealSize = pSize+1;
    uint8_t lSizeParityByte = (HPAR(lRealSize) << 7);
    uint8_t lHeader         = lSizeParityByte | gCommands[pCmdId][1] | lRealSize;
    uint8_t lId             = gCommands[pCmdId][0];
    
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
void treatCommand(uint8_t pCmd, uint8_t pSize)
{
    if (pCmd == gCommands[ID_NOOP_CMD][0])
    {
        Serial.write((uint8_t*) &gSyncMagicNumber, sizeof(gSyncMagicNumber));
        
        const uint8_t lSize = 1+6*sizeof(float);
        uint8_t lValues[lSize];
        float *lFloats = (float*) (&lValues[1]);
        
        lValues[0] = 1;
        lFloats[0] = 10.0f;
        lFloats[1] = 15.0f;
        lFloats[2] = 10.0f;
        lFloats[3] = 20.0f;
        
        sendCommand(ID_PLOT_RET, lValues, lSize);
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











