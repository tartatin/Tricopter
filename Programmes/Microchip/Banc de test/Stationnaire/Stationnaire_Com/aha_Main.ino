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
    printText("+ capteurs...\n");
    setupI2C();
    setupIMU();
    setupVAMeter();
  
    // Moteurs & servo
    printText("+ servo...\n");
    setupServo();
    printText("+ moteurs...\n");
    setupMotors();
  
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
void loop() 
{    
    static uint32_t lLast = millis();
    uint32_t lNow = millis();
    //if (lNow - lLast > 10000)
    //    quickStop();
  
    // Lecture des commandes et arrêt d'urgence
    while (receiveMsg())
        treatMessage();
  
    // Calcul des commandes moteurs :
    // 1. Lecture des valeurs de l'accéléromètre
    // 2. Calculs des différents axes liés à une éventuelle perte d'équilibre
    // 3. Détermination des ratios correctifs pour chaque moteur, et mise à jour des nouveaux objectifs des moteurs
    getAccelerometerData();
    getGyroscopeData();
    
    getMotorsCorrection();
    updateMotorsCmd();
}

