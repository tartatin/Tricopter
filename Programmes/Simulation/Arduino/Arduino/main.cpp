#include <Windows.h>
#include "arduino.h"

#define T(x) x, x, x

int main(int argc, char **argv)
{
	char data[] = {T(0x21),
				   T(0x11),
				   T(0x31),
				   T(0x81),
	               T(0x31),

				   0x65,
	
				   T(0x21),
				   T(0x81),
	               T(0x31),
	
				   0x21, 0x21};

	setup();
	Serial.__write((uint8_t*)data, _countof(data));
	while (true)
		loop();
	return 0;
}