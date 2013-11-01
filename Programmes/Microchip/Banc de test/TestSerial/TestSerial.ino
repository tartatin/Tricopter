void setup() 
{   
    // Communication
    setupCom();
}

/*****************************************************************/
uint8_t gBuffer[32];
uint8_t gCount = 0;

void loop() 
{
    while (Serial.available() > 0)
    {
        gBuffer[gCount] = Serial.read();
        gCount++;
        if (gCount == 32)
        {
            Serial.write((uint8_t*) gBuffer, gCount);
            gCount = 0;
        }
    }
}

/*****************************************************************/
void setupCom()
{
    Serial.begin(115200);
    Serial.setTimeout(1);
}
