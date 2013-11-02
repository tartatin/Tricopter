/**************************************************************************/
/*                             Communication                              */
/**************************************************************************/

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
// Commandes
#define NOOP_MSGID             0x00
#define PING_MSGID             0x01
#define SET_THRUST_MSGID       0x02
#define SET_SERVO_MSGID        0x03
#define SET_ENGINE_MSGID       0x04
#define SET_PID_MSGID          0x05
#define PRINT_FLOAT_MSGID      0x06
#define PRINT_STRING_MSGID     0x07
#define STOP_ENGINE_MSGID      0x08
#define RETURN_PLOT_MSGID      0x09
#define PRINT_INT_MSGID        0x0A

/*****************************************************************/
void setupCom()
{
    Serial.begin(38400);
    Serial.setTimeout(1);
    
    printText("Communication... done.\n");
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
void printInt16(int16_t pValue)
{
    sendCommand(PRINT_INT_MSGID, (uint8_t*) &pValue, sizeof(int16_t));
}

void printUInt16(uint16_t pValue)
{
    sendCommand(PRINT_INT_MSGID, (uint8_t*) &pValue, sizeof(uint16_t));
}

/*****************************************************************/
void printText(char *pTxt)
{
    const uint8_t lStrLen = strlen(pTxt);
    if (lStrLen > gMsgMaxSize-1)
        printText((char*) (pTxt+(gMsgMaxSize-1)));
        
    pTxt[lStrLen] = 0x00;
    
    prepareCommand(PRINT_STRING_MSGID);
    appendParams((uint8_t*) pTxt, lStrLen);
    endCommand();
}

/*****************************************************************/
void printTextf(char *fmt, ... ){
    char tmp[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(tmp, 128, fmt, args);
    va_end (args);
    printText(tmp);
}

/*****************************************************************/
void sendPlot(uint8_t pDim, uint8_t pPlotID, float pTime, float *pValues)
{
    prepareCommand(RETURN_PLOT_MSGID);
    appendParams((uint8_t*) &pPlotID, sizeof(pPlotID));
    appendParams((uint8_t*) &pTime,   sizeof(pTime));
    appendParams((uint8_t*) pValues,  pDim*sizeof(float));
    endCommand();
}

/*****************************************************************/
void sendPlot2D(uint8_t pPlotID, float pTime, float pV1, float pV2)
{
    float lValues[2] = {pV1, pV2};
    sendPlot(2, pPlotID, pTime, lValues);
}

/*****************************************************************/
void sendPlot3D(uint8_t pPlotID, float pTime, float pV1, float pV2, float pV3)
{
    float lValues[3] = {pV1, pV2, pV3};
    sendPlot(3, pPlotID, pTime, lValues);
}

/*****************************************************************/
#define IS_MSG(y) (gMsgBuffer[0] == y)
#define CHECK_SIZE(s) {if(gMsgSize != (s+1)) {gDropCount++; return false;}}

#define UINT32_PTR(offset) ((uint32_t*) &gMsgBuffer[offset+1])
#define INT32_PTR(offset) ((int32_t*) &gMsgBuffer[offset+1])
#define UINT16_PTR(offset) ((uint16_t*) &gMsgBuffer[offset+1])
#define INT16_PTR(offset) ((int16_t*) &gMsgBuffer[offset+1])
#define UINT8_PTR(offset) ((uint8_t*) &gMsgBuffer[offset+1])
#define INT8_PTR(offset) ((int8_t*) &gMsgBuffer[offset+1])
#define BOOL_PTR(offset) ((bool*) &gMsgBuffer[offset+1])

#define UINT32(offset) *UINT32_PTR(offset)
#define INT32(offset) *INT32_PTR(offset)
#define UINT16(offset) *UINT16_PTR(offset)
#define INT16(offset) *INT16_PTR(offset)
#define UINT8(offset) *UINT8_PTR(offset)
#define INT8(offset) *INT8_PTR(offset)
#define BOOL(offset) *BOOL_PTR(offset)

/*****************************************************************/
bool treatMessage()
{
    if (gMsgReceived == false)
        return false;
    gMsgReceived = false;
  
    /************************ Prioritaires **********************/
    // Arrêt d'urgence
    if (IS_MSG(STOP_ENGINE_MSGID))
    {
        deadStop();
    }
    
    // Commande moteur
    else if (IS_MSG(SET_ENGINE_MSGID))
    {
        CHECK_SIZE(sizeof(uint8_t));
        uint8_t *lValue = UINT8_PTR(0);

        if (*lValue == 0xAA)
            startMotors();
        else
            stopMotors();
    }

    /*********************** Asservissement**********************/    
    // Poussée
    else if (IS_MSG(SET_THRUST_MSGID))
    {
        CHECK_SIZE(sizeof(uint16_t));
        
        // Set de la valeur
        uint16_t lValue = UINT16(0);
        thresholdRangeUInt16(&lValue, 0, 2000);
        setMeanForce(lValue);
        
        // Get de la valeur pour retour
        lValue = getMeanForce();
        printTextf("Thrust: %d\n", lValue);
    }
    
    // Servo
    else if (IS_MSG(SET_SERVO_MSGID))
    {
        CHECK_SIZE(sizeof(int16_t));
        setServoAngle(INT16(0));
        
        int16_t lValue = getServoAngle();
        printTextf("Servo: %d\n", lValue);
    }
    
    /************************** Autres ************************/
    // No-op
    else if (IS_MSG(NOOP_MSGID))
    {
        printText("NOOP_MSGID\n");
    }
    
    // Ping
    else if (IS_MSG(PING_MSGID))
    {
        printText("PING_MSGID\n");
    }
    
    // Un message reçu
    return true;
}

/*****************************************************************/
void clearMsg()
{
    gStartFound = false;
    gMsgReceived = false;
    gIsEscapedChar = false;
    gMsgSize = 0;
}

/*****************************************************************/
bool resolveChar(uint8_t *pChar, bool pAllowErrors)
{
    // Les 3 chars consécutifs doivent être identiques
    //    à 100% si aucun message n'a commencé,
    //    à 66% si un message a commencé
    
    *pChar = 0x00;
    if (pAllowErrors)
    {
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
                *pChar = gLastChars[i];
                return true;
            }
        }
        
        // Les données ne sont pas consistantes, donc un morceau des données est mauvais, donc le message est foutu.
        return false;
    }
    else
    {
        for(uint8_t i = 1; i < gRedundancyCount; ++i)
        {
            if (gLastChars[0] != gLastChars[i])
                return false;
        }
        
        *pChar = gLastChars[0];
        return true;
    }
}

/*****************************************************************/
bool receiveMsg()
{
    // On supprime le message stocké, si jamais y'en a un.
    // S'il n'y en a pas, il ne faut surtout pas supprimer car le message est en cours de réception.
    if (gMsgReceived)
        clearMsg();
                
    // On ne travail que si des nouvelles donn?es sont disponibles
    while( true ) //Serial.available() > 0 )
    {
        // Lecture des nouvelles données, si nécessaire
        if (gReadChars < gRedundancyCount)
        {
            int lActuallyRead = Serial.readBytes((char*) &gLastChars[gReadChars], gRedundancyCount-gReadChars);
            gReadChars += lActuallyRead;
        }
        
        // On doit avoir lu les 3 chars consécutifs
        if (gReadChars < gRedundancyCount)
        {
            break;
        }
        
        //printText("receiveMsg - 3\n");
        
        // Détermination du caractère
        uint8_t lChar;
        if (resolveChar(&lChar, gStartFound) == false)
        {
            //printText("receiveMsg - 4\n");
            // On supprime le message en cours
            clearMsg();
            
            // On conserve les 2 derniers caractères
            gReadChars = gRedundancyCount-1;
            for(uint8_t i = 0; i < gRedundancyCount-1; ++i)
                gLastChars[i] = gLastChars[i+1];
            continue;
        }
        else
        {
            //printText("receiveMsg - 5\n");
            // On supprime le buffer, puisqu'on a pu lire le caractère
            gReadChars = 0;
        }
        
        // Deux cas : soit on attend le début du message, soit on est dedans
        if (gStartFound)
        {
            //printText("receiveMsg - 6\n");
            
            // On lit les données petit à petit
            if (gIsEscapedChar)
            {
                //printText("receiveMsg - 7\n");
                
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
            //printText("receiveMsg - 8\n");
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
            break;
    }
    
    return gMsgReceived;
}
