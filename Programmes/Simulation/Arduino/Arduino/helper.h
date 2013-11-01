#pragma once

#include <algorithm>
#include <vector>
#undef min

#define OUTPUT 0
#define INPUT  1
#define HIGH 1
#define LOW  0

typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;

void loop();
void setup();

void pinMode(unsigned char, unsigned char);
void digitalWrite(unsigned char, unsigned char);
void delay(uint32_t);
uint32_t min(uint32_t a, uint32_t b);

/*********************************/

class CSerial {
private:
	std::vector<uint8_t> mData;
public:
	void begin(uint32_t);
	void write(uint8_t*, unsigned int);
	uint32_t available();
	char read();
	int readBytes(char *pOut, int pSize);
	void setTimeout(int pTime);

	void __write(uint8_t*,unsigned int);
};

extern CSerial Serial;

