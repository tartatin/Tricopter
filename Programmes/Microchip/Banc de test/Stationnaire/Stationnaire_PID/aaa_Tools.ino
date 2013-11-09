/**************************************************************************/
/*                                 Outils                                 */
/**************************************************************************/

class int16_ratio {
public:
    int16_ratio(int16_t pNum, int16_t pDenom)
    {
        num = pNum;
        denom = pDenom;
    }
    
    int16_t operator*(const int16_t value)
    {
        return (value * num) / denom;
    }
  
    int16_t num;
    int16_t denom;
};

/*****************************************************************/
inline void thresholdRangeFloat(float *pValue, float pMin, float pMax)
{
    if (*pValue < pMin)
        *pValue = pMin;
    if (*pValue > pMax)
        *pValue = pMax;
}

inline void thresholdRangeInt16(int16_t *pValue, int16_t pMin, int16_t pMax)
{
    if (*pValue < pMin)
        *pValue = pMin;
    if (*pValue > pMax)
        *pValue = pMax;
}

inline void thresholdRangeUInt16(uint16_t *pValue, uint16_t pMin, uint16_t pMax)
{
    if (*pValue < pMin)
        *pValue = pMin;
    if (*pValue > pMax)
        *pValue = pMax;
}
