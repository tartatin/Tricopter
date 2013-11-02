/*****************************************************************/
/*                            Main                               */
/*****************************************************************/

/*****************************************************************/
void setup() 
{   
    // Communication
    Serial.begin(38400);
    
    // Capteurs
    setupI2C();
    setupIMU();
    //setupVAMeter();
  
    // Moteurs & servo
    setupServo();
    setupMotors();
    
    startMotors();
}

/*****************************************************************/
void quickStop()
{
    // Arrêt total des moteurs
    stopMotors();
    
    Serial.println("Stop.");
    
    // Boucle infinie, plus rien ne se passera
    while(true);
}

/*****************************************************************/
void loop() 
{
    static uint32_t lLast = millis();
    uint32_t lNow = millis();
    if (lNow - lLast > 10000)
        quickStop();
  
    getAccelerometerData();
    getGyroscopeData();
    getMotorsCorrection();
    updateMotorsCmd();
    
    char tmp[128];
    sprintf(tmp, "%d | %d | %d\n", g_MotorsTarget[0], g_MotorsTarget[1], g_MotorsTarget[2]);
    Serial.println(tmp);
    
    delay(100);
}

