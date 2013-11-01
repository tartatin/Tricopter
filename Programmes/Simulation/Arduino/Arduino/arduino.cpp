#include "arduino.h"

/*****************************************************************/
/*                            Main                               */
/*****************************************************************/
void setup() 
{   
  pinMode(13, OUTPUT);     
  
  // Communication
  setupCom();
}

/*****************************************************************/
void loop() 
{
    // Lecture des commandes et arrêt d'urgence
    while (receiveMsg())
        treatMessage();
}

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
#define MSG_SET_SERVO      (CMD_BASE | 0x04)
#define MSG_CMD_STOP       (CMD_BASE | 0x06)
#define MSG_SET_MIN_ACC    (CMD_BASE | 0x07)
#define MSG_ENGINE_CMD     (CMD_BASE | 0x08)
#define MSG_PID_CMD        (CMD_BASE | 0x09)
#define MSG_KEEP_ALIVE_CMD (CMD_BASE | 0x0A)

// Retours
#define MSG_VAL_PING       (RET_BASE | 0x01)
#define MSG_VAL_PLOT       (RET_BASE | 0x02)
#define MSG_VAL_VA         (RET_BASE | 0x03)

// Prints
#define MSG_PRINT_FLOAT    (RET_BASE | 0x04)
#define MSG_PRINT_STRING   (RET_BASE | 0x05)

/*****************************************************************/
void setupCom()
{
    Serial.begin(115200);
    Serial.setTimeout(1);
    
    printText("Communication ok.\n");
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
void printFloat(float pValue)
{
    sendCommand(MSG_PRINT_FLOAT, (uint8_t*) &pValue, sizeof(float));
}

/*****************************************************************/
void printText(char *pTxt)
{
    prepareCommand(MSG_PRINT_STRING);
    const uint8_t lSize = min(gMsgMaxSize-1, strlen(pTxt));
    appendParams((uint8_t*) pTxt, lSize);
    endCommand();
}

/*****************************************************************/
void sendPlot(uint8_t pDim, uint8_t pPlotID, float pTime, float *pValues)
{

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
    
    sendCommand(MSG_VAL_PLOT, lValues, lSize);
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
#define IS_MSG(y) (gMsgBuffer[0] == y)
#define CHECK_SIZE(s) {if(gMsgSize != (s+1)) {gDropCount++; return false;}}

#define UINT32_PTR(offset) ((uint32_t*) &gMsgBuffer[offset+1])
#define BOOL_PTR(offset) ((bool*) &gMsgBuffer[offset+1])
#define FLOAT_PTR(offset) ((float*) &gMsgBuffer[offset+1])

#define UINT32(offset) *UINT32_PTR(offset)
#define BOOL(offset) *BOOL_PTR(offset)
#define FLOAT(offset) *FLOAT_PTR(offset)

/*****************************************************************/
bool treatMessage()
{
    if (gMsgReceived == false)
        return false;
    gMsgReceived = false;
  
    /************************ Prioritaires **********************/
    // Arrêt d'urgence
    if (IS_MSG(MSG_CMD_STOP))
    {
        CHECK_SIZE(0);
        printText("MSG_CMD_STOP\n");
        //quickStop();
    }
    
    // Commande moteur
    else if (IS_MSG(MSG_ENGINE_CMD))
    {
        CHECK_SIZE(sizeof(uint32_t));
        printText("MSG_ENGINE_CMD\n");
        /*uint32_t *lValue = UINT32_PTR(0);

        if (*lValue == 0x12345678)
        {
            g_MotorsState = true;
            printText("Allumage.\x00");
        }
        else
        {
            g_MotorsState = false;
        }*/
    }

    /*********************** Asservissement**********************/    
    // Poussée
    else if (IS_MSG(MSG_SET_THRUST))
    {
        CHECK_SIZE(sizeof(float));
        printText("MSG_SET_THRUST\n");
        /*g_MeanForce = FLOAT(0);
        thresholdRange(&g_MeanForce, 0.0f, 20.0f);
        printFloat(g_MeanForce);*/
    }
    
    // Paramètre du correcteur PID
    else if (IS_MSG(MSG_PID_CMD))
    {
        CHECK_SIZE(3*sizeof(float));
        printText("MSG_PID_CMD\n");
        
        /*g_P_Coeff = FLOAT(0*sizeof(float));
        g_I_Coeff = FLOAT(1*sizeof(float));
        g_D_Coeff = FLOAT(2*sizeof(float));

        thresholdRange(&g_P_Coeff, -100.0f, +100.0f);
        thresholdRange(&g_I_Coeff, -100.0f, +100.0f);
        thresholdRange(&g_D_Coeff, -100.0f, +100.0f);
        
        printFloat(g_P_Coeff);
        printFloat(g_I_Coeff);
        printFloat(g_D_Coeff);*/
    }
    
    // Accélération minimale
    else if (IS_MSG(MSG_SET_MIN_ACC))
    {
        CHECK_SIZE(sizeof(float));
        printText("MSG_SET_MIN_ACC\n");
        /*g_MinAcceleration = FLOAT(0);
        thresholdRange(&g_MinAcceleration, 0.001f, 0.5f);*/
    }
    
    // Servo
    else if (IS_MSG(MSG_SET_SERVO))
    {
        CHECK_SIZE(sizeof(float));
        printText("MSG_SET_SERVO\n");
        /*float *lAngle = FLOAT_PTR(0);
        thresholdRange(lAngle, -45.0f, +45.0f);
        setServoAngle(*lAngle);*/
    }
    
    /************************** Autres ************************/
    // No-op
    else if (IS_MSG(MSG_CMD_NOOP))
    {
        CHECK_SIZE(0);
        printText("MSG_CMD_NOOP\n");
    }
    
    // Ping
    else if (IS_MSG(MSG_CMD_PING))
    {
        CHECK_SIZE(0);
        printText("MSG_CMD_PING\n");
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
bool resolveChar(uint8_t *pChar)
{
    // Les 3 chars consécutifs doivent être identiques
    //    à 100% si aucun message n'a commencé,
    //    à 66% si un message a commencé
    
    *pChar = 0x00;
    char lChar = 0x00;
    if (gStartFound)
    {
        bool lFound = false;
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
        bool lAreAllSame = true;
        for(uint8_t i = 1; i < gRedundancyCount; ++i)
        {
            if (gLastChars[0] != gLastChars[i])
            {
                lAreAllSame = false;
                break;
            }
        }
        
        if (lAreAllSame)
        {
            *pChar = gLastChars[0];
            return true;
        }
        else
        {
            // aucun message n'a commencé, alors on est simplement désynchro
            return false;
        }
    }
}

/*****************************************************************/
bool receiveMsg()
{
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
            break;
        
        // Détermination du caractère
        uint8_t lChar;
        if (resolveChar(&lChar) == false)
        {
            // On conserve les 2 derniers caractères
            gReadChars = gRedundancyCount-1;
            for(uint8_t i = 0; i < gRedundancyCount-1; ++i)
                gLastChars[i] = gLastChars[i+1];
            continue;
        }
        else
        {
            // On supprime le buffer
            gReadChars = 0;
        }
        
        // Deux cas : soit on attend le début du message, soit on est dedans
        if (gStartFound)
        {
            // On lit les données petit à petit
            if (gIsEscapedChar)
            {
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