/*****************************************************************/
/*                            Main                               */
/*****************************************************************/

/*****************************************************************/
void setup() 
{   
    // Communication
    setupCom();
  
    printText("Initialisation...");
    
    // Capteurs
    printText("+ capteurs...");
    setupI2C();
    setupIMU();
    setupVAMeter();
  
    // Moteurs & servo
    printText("+ servo...");
    setupServo();
    printText("+ moteurs...");
    setupMotors();
  
    // Fin du setup
    printText("Prêt à décoller.");
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
void loop() 
{    
    // Lecture des commandes et arrêt d'urgence
    while (receiveMsg())
        treatMessage();
        
    static uint32_t lLast = millis();
    uint32_t lNow = millis();
    uint32_t lDelta = lNow - lLast;
    if (lDelta < 100)
        return;
    lLast = lNow;
    
    static byte slLoopCount = 0;
    if (slLoopCount % 20 == 0)
    {
        updateVAMeasures();
        printTextf("Voltage (mV): %d", (int16_t)(g_Voltage * 1000.0f));
    }
    ++slLoopCount;
  
    // Calcul des commandes moteurs :
    // 1. Lecture des valeurs de l'accéléromètre
    // 2. Calculs des différents axes liés à une éventuelle perte d'équilibre
    // 3. Détermination des ratios correctifs pour chaque moteur, et mise à jour des nouveaux objectifs des moteurs
    getAccelerometerData();
    getGyroscopeData();
    
    updatePID();
    updateMotorsCmd();
    
    int16_t lIntValues[3];
    lIntValues[0] = (int16_t)(g_FloatBuffer[0] * 1.0f);
    lIntValues[1] = (int16_t)(g_FloatBuffer[1] * 1.0f);
    lIntValues[2] = (int16_t)(g_FloatBuffer[2] * 1.0f);
    
    // DEBUG /////////////////////////////////////////////////////////////////////////////
    static float slTmpFloat[3];
    
    slTmpFloat[0] = (float)g_Raw_Rotation[0];
    slTmpFloat[1] = (float)g_Raw_Rotation[1];
    slTmpFloat[2] = (float)g_Raw_Rotation[2];
    sendPlot(3, 2, (float)lNow * 0.001f, slTmpFloat);
    
    slTmpFloat[0] = (float)g_MotorsTarget[0];
    slTmpFloat[1] = (float)g_MotorsTarget[1];
    slTmpFloat[2] = (float)g_MotorsTarget[2];
    sendPlot(3, 3, (float)lNow * 0.001f, slTmpFloat);
    
    // printTextf("%d | %d | %d", g_MotorsTarget[0], g_MotorsTarget[1], g_MotorsTarget[2]);
}

