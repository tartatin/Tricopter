// Essai de communication
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

bool doblink = false;

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
    receiveMsg();
    treatCommand();
}

/**************************************************************************/
/*                             Communication                              */
/**************************************************************************/

/*****************************************************************/
void setupCom()
{
    Serial.begin(9600); 
}

/*****************************************************************/
void writeComSpecial(uint8_t pChar)
{
    for(uint8_t i = 0; i < gRedundancyCount; ++i)
        Serial.write(&pChar, 1);
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
void printText(char *pTxt)
{
    prepareCommand(MSG_PRINT_STRING);
    const uint8_t lSize = min(gMsgMaxSize-1, strlen(pTxt));
    appendParams((uint8_t*) pTxt, lSize);
    endCommand();
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
bool treatCommand()
{
    if (gMsgReceived == false)
        return false;
    gMsgReceived = false;
    
    /************************** Autres ************************/
    // No-op
    if (IS_MSG(MSG_CMD_PING))
    {
        CHECK_SIZE(0);
        printText("Hello world !");
    }
    
    // Un message traité
    return true;
}

/*****************************************************************/
bool receiveMsg()
{
    // Si un message a deja ete recu, il doit d'abord etre traite.
    if (gMsgReceived)
        return true;
    
    uint8_t lAvailable;

    // On ne travail que si des nouvelles donn?es sont disponibles
    while( (lAvailable = Serial.available()) > 0 )
    {
        const uint8_t clToRead = min(gRedundancyCount, lAvailable)-gReadChars;
        for(uint8_t i = 0; i < clToRead; ++i)
            gLastChars[gReadChars+i] = Serial.read();
        gReadChars += clToRead;
        
        // On doit avoir lu les 3 chars consécutifs
        if (gReadChars < gRedundancyCount)
            break;
        gReadChars = 0;
            
        // Les 3 chars consécutifs doivent être identiques
        //    à 100% si aucun message n'a commencé,
        //    à 66% si un message a commencé
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
                    lChar = gLastChars[i];
                    lFound = true;
                    break;
                }
            }
            
            if (lFound == false)
            {
                // Les données ne sont pas consistantes, donc un morceau des données est mauvais, donc le message est foutu.
                gStartFound = false;
                gMsgReceived = false;
                gIsEscapedChar = false;
                gMsgSize = 0;
                gDropCount++;
            }
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
                lChar = gLastChars[0];
            else
            {
                // aucun message n'a commencé, alors on est simplement désynchro, auquel cas on ne supprime que le 1er octet,
                gReadChars = gRedundancyCount-1;
                for(uint8_t i = 0; i < gRedundancyCount-1; ++i)
                    gLastChars[i] = gLastChars[i+1];
                continue;
            }
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
