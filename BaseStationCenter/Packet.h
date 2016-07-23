/* 
* Packet.h
*
* Created: 8/1/2015 5:34:46 PM
* Author: CIT_15
*/


#ifndef __PACKET_H__
#define __PACKET_H__

#include "definition.h"



class Packet
{
//variables
public:
protected:
private:
	int packetNumber;
	int crcPacket;
	char header[PACKET_HEADER_NUM];//,0x22,0x33,0x44,0x55,0x66};
	//char header[PACKET_HEADER_NUM]={0x00,0x00,0x00,0x00,0x00,0x00};
	char preamble[PREAMBLE_LENGTH];


//functions
public:
	Packet();
	~Packet(); 
	void createPacket(char *dataString ,int dataLength,char *packetString);
	bool extractData(char *packetString ,int &dataLength);
	
protected:
private:
	Packet( const Packet &c );
	Packet& operator=( const Packet &c );
	bool isHeaderDetected(char *string);
	int calculatedCRC(char *string , int strSize);
}; //Packet

#endif //__PACKET_H__
