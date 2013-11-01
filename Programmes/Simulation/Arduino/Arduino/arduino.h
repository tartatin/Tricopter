#pragma once

#include "helper.h"

void setupCom();
void writeComSpecial(uint8_t pChar);
void writeComData(uint8_t *pData, uint8_t pSize);
void prepareCommand(uint8_t pCmdId);
void endCommand();
void appendParams(uint8_t *pArgs, uint8_t pSize);
void sendCommand(uint8_t pCmdId, uint8_t *pArgs, uint8_t pSize);
void thresholdRange(float *pValue, float pMin, float pMax);
bool treatMessage();
bool receiveMsg();
void printFloat(float pValue);
void printText(char *pTxt);