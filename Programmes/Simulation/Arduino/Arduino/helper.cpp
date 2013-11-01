#include "helper.h"

CSerial Serial;

void pinMode(unsigned char, unsigned char)
{
}

void digitalWrite(unsigned char, unsigned char)
{
}

void delay(uint32_t)
{
}

uint32_t min(uint32_t a, uint32_t b)
{
	return std::min(a, b);
}

/*********************************/

void CSerial::begin(uint32_t)
{
}

void CSerial::write(uint8_t*, unsigned int)
{
}

uint32_t CSerial::available()
{
	return mData.size();
}

char CSerial::read()
{
	char a = (char) mData.at(0);
	mData.erase(mData.begin());
	return a;
}

void CSerial::__write(uint8_t* ptr, unsigned int count)
{
	mData.insert(mData.end(), ptr, (ptr+count));
}

void CSerial::setTimeout(int pTime)
{

}

int CSerial::readBytes(char *pOut, int pSize)
{
	int count = min(pSize, mData.size());
	for(int i = 0; i < count; ++i)
	{
		*pOut = read();
		pOut++;
	}
	return count;
}