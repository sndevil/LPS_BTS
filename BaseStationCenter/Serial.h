/*
 * Serial.h
 *
 * Created: 3/8/2015 12:07:40 PM
 *  Author: Mobarezi
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_

class Serial
{
	private:
		char rxData;
		bool rxFlag;
		bool txFlag;
public:
	void init(int baudRate);
	void putChar(char data);
	void putChar2(char data);	
	void put16Bit(int data);
	unsigned char getChar();
	void putStr(const char *s);
	void sendPacket(char* pkt, int length);
	bool isSerialReceived();
	bool isSerialTransmitted();
	bool isTXReady();
	char getChar2();
	void uartRXINT();
	void uartTXINT();
	bool isSerialTxEmpty();
	bool isSerialRxFull();
};



#endif /* SERIAL_H_ */