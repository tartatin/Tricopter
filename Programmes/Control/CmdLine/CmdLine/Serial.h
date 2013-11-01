#pragma once

#include <thread>

enum SerialPort {
	COM1,
	COM2,
	COM3,
	COM4
};

enum SerialBaudRate {
	Baud9600,
	Baud19200,
	Baud38400,
	Baud57600,
	Baud115200
};

class CSerial {
public:
	CSerial();
	~CSerial();

	void open(SerialPort pPort, SerialBaudRate pRate);
};

