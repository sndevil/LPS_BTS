/*
 * Message.cpp
 *
 * Created: 3/8/2015 10:42:03 AM
 *  Author: Mobarezi
 */ 

#include "Message.h"
#include "Serial.h"
#include "GlobalVariables.h"

//int Message::calculateCRC(unsigned char *ptr, int count)
//{
	//int crc;
	//char i;
//
	//crc = 0;
//
	//while (--count >= 0)
	//{
		//crc = crc ^ (int) *ptr++ << 8;
		//i = 8;
//
		//do
		//{
			//if (crc & 0x8000)
			//crc = crc << 1 ^ 0x1021;
			//else
			//crc = crc << 1;
		//} while(--i);
	//}
	//return (crc);
//}

void Message::constructBaseGetStatusPacket()
{
	baseGetStatusPacket.frameHeader = 0x7e;
	baseGetStatusPacket.messageType = 1;
	baseGetStatusPacket.frameTrailer = 0x81;
	
	unsigned char* packetPointer = (unsigned char*)&baseGetStatusPacket;
	int len = sizeof(BaseGetStatusPacket)-4;

	packetPointer++;
	int crc = calculateCRC(packetPointer, len);

	baseGetStatusPacket.checksum[0] = crc & 0xFF;
	baseGetStatusPacket.checksum[1] = (crc>>8) & 0xFF;
}

void Message::sendBaseGetStatusPacket()
{
	constructBaseGetStatusPacket();
	
	serial.sendPacket((char*)&baseGetStatusPacket, sizeof(struct BaseGetStatusPacket));
}