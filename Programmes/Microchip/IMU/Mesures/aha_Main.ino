/*****************************************************************/
/*                            Main                               */
/*****************************************************************/

char str[128]; 

/*****************************************************************/
void setup() 
{   
    Serial.begin(38400);

    setupI2C();
    setupIMU();
    setupVAMeter();
}

void bitN(int ptr, char* out, byte N)
{
    char* cur = out + N;
    for(byte i = 0; i < N; ++i)
    {
        --cur;
        *cur = '0' + (byte)(ptr & 0x0001);
        ptr >>= 1;
    }
}
char sa[17];
char sb[17];

/*****************************************************************/
void loop() 
{
    getAccelerometerData();
    getGyroscopeData();
    updateVAMeasures();
    
    int acc[3];
    acc[0] = (int)(g_Raw_Acceleration[0] * 1000.0f);
    acc[1] = (int)(g_Raw_Acceleration[1] * 1000.0f);
    acc[2] = (int)(g_Raw_Acceleration[2] * 1000.0f);
    
    /*int gyro[3];
    gyro[0] = (int)(g_Raw_RotationRate[0]);
    gyro[1] = (int)(g_Raw_RotationRate[1]);
    gyro[2] = (int)(g_Raw_RotationRate[2]);*/
    
    int va[2];
    va[0] = (int)(g_Voltage * 1000.0f);
    va[1] = (int)(g_Current * 1000.0f);
    
    
    
    /*bitN(tmp_gyro[0], sa, 8);
    bitN(tmp_gyro[1], sa+8, 8);
    sa[16] = 0;
    bitN(tmp_gyro[2], sb, 16);
    sb[16] = 0;
    sprintf(str, "%s - %s\n", sa, sb);*/
    
    //sprintf(str, "%d | %d | %d\n", g_Raw_Rotation[0], g_Raw_Rotation[1], g_Raw_Rotation[2]);
    
    int ArmLength = 100; // longueur des bras
    int ArmLengthCos120 = 50;
    int ArmLengthSin120 = 87;
    
    int Intensity = 100;
    
    int Thrusts[3]; // poussée que chaque moteur doit fournir pour rétablir l'équilibre
    
    int OverX = ((g_Raw_Rotation[0] * Intensity) / ArmLength);
    int OverY = ((g_Raw_Rotation[1] * Intensity) / ArmLengthSin120);
    
    Thrusts[0] = -OverX;
    Thrusts[1] = +OverX/2 - OverY;
    Thrusts[2] = +OverX/2 + OverY;
    
    sprintf(str, "%d | %d | %d\n", Thrusts[0], Thrusts[1], Thrusts[2]);
    
    //sprintf(str, "acc: %d,%d,%d | gyro: %ud,%ud,%ud | va: %d,%d\n", acc[0], acc[1], acc[2], tmp_gyro[0], tmp_gyro[1], tmp_gyro[2], va[0], va[1]);  
    Serial.println(str);
    
    delay(100);
}

