/*****************************************************************/
/*                            Main                               */
/*****************************************************************/

// Vol stationnaire

uint8_t gBuffer[32];
uint8_t gCount = 0;

/*****************************************************************/
void setup() 
{   
    // Communication
    setupCom();
  
    printText("Initialisation...");
    
    // Capteurs
    printText("+ capteurs...\n");
    setupI2C();
    setupIMU();
    setupVAMeter();
  
    // Moteurs & servo
    printText("+ moteurs...\n");
    setupMotors();
    setupServo();
  
    // Stabilisateur
    printText("+ contrôle de vol...\n");
    setupStabilizer();
  
    // Fin du setup
    printText("Prêt à décoller.\n");
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
  
    sendPlot(3, 1, l_Now, g_Raw_Acceleration);
    sendPlot(3, 2, l_Now, g_MotorsValue);
  
    /*sendPlot(3, 1, l_Now, g_P_Acceleration);
    sendPlot(3, 2, l_Now, g_I_Acceleration);
    sendPlot(3, 3, l_Now, g_D_Acceleration);*/
    
    /*send2DPlot( 4, l_Now, g_MotorsValue[0], g_MotorsTarget[0]);
    send2DPlot( 5, l_Now, g_MotorsValue[1], g_MotorsTarget[1]);
    send2DPlot( 6, l_Now, g_MotorsValue[2], g_MotorsTarget[2]);*/
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
    computeInput();
    updateAxis();
    getMotorsCorrection();
    
    // Mise à jour des valeurs moteurs
    updateMotorsCmd(lDiff);
    
    // Debug
    printStatus();
    
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

