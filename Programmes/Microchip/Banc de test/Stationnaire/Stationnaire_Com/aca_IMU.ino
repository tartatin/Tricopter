/**************************************************************************/
/*                                  IMU                                   */
/**************************************************************************/
// Données instantannées
int16_t g_Raw_Rotation[3]; // Orientations (en unité d'angle)
int16_t g_Raw_Acceleration[3]; // Accélération instantannée, en unités légales


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

#define G_TO_READ 8 // 2 bytes for each axis x, y, z + 1 pour la temperature

// offsets are chip specific. 
int16_t g_offx = 120;
int16_t g_offy = 20;
int16_t g_offz = 93;

/*****************************************************************/
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
  g_Raw_Acceleration[0] = (((int16_t)buff[1]) << 8) | (int16_t)buff[0];   
  g_Raw_Acceleration[1] = (((int16_t)buff[3]) << 8) | (int16_t)buff[2];
  g_Raw_Acceleration[2] = (((int16_t)buff[5]) << 8) | (int16_t)buff[4];
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
    
    g_Raw_Rotation[0] = ((int16_t)buff[3] << 8) | (int16_t)buff[2];
    g_Raw_Rotation[1] = ((int16_t)buff[5] << 8) | (int16_t)buff[4];
    g_Raw_Rotation[2] = ((int16_t)buff[7] << 8) | (int16_t)buff[6];
}
