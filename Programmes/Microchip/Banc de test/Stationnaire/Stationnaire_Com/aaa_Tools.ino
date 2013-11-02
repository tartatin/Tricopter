/**************************************************************************/
/*                                 Outils                                 */
/**************************************************************************/

/*****************************************************************/
void thresholdRangeInt16(int16_t *pValue, int16_t pMin, int16_t pMax)
{
    if (*pValue < pMin)
        *pValue = pMin;
    if (*pValue > pMax)
        *pValue = pMax;
}

void thresholdRangeUInt16(uint16_t *pValue, uint16_t pMin, uint16_t pMax)
{
    if (*pValue < pMin)
        *pValue = pMin;
    if (*pValue > pMax)
        *pValue = pMax;
}
