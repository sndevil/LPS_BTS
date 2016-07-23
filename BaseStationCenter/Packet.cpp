/*
* Packet.cpp
*
* Created: 8/1/2015 5:34:46 PM
* Author: CIT_15
*/


#include "Packet.h"
#include "definition.h"
#include "GlobalVariables.h"

// default constructor
Packet::Packet()
{
	header[0]=0x11;
	preamble[0]=0x12;
	preamble[1]=0x34;
	preamble[2]=0x56;
	//header = {0x11,0x22,0x33,0x44,0x55,0x66};
	//preamble ={0x12,0x34,0x56};
} //Packet

// default destructor
Packet::~Packet()
{
} //~Packet

void Packet::createPacket( char *dataString ,int dataLength,char *packetString )
{
	unsigned char i;
	crcPacket =0;
	if (dataLength > DATA_LENGTH )
		dataLength = DATA_LENGTH;
	if (dataLength != 0 )
	{
		//for (i=0; i< PREAMBLE_LENGTH ;i++)
		//{
			//packetString[i]=preamble[i];
		//}
		for (i=0; i < PACKET_HEADER_NUM ; i++)
		{
			packetString[i] = header[i];
		}
		
		packetString[PACKET_HEADER_NUM ] = (dataLength >> 8);
		packetString[PACKET_HEADER_NUM  + 1] = (dataLength & 0xFF);
		
		for (i=0; i < dataLength ; i++)
		{
			crcPacket += dataString[i];
			packetString[i + PACKET_HEADER_NUM  + 2 ]= dataString[i];
		}
		//packetString[i + PACKET_HEADER_NUM + PREAMBLE_LENGTH  ]= '\r';
		//packetString[i + PACKET_HEADER_NUM + PREAMBLE_LENGTH + 1 ]= '\n';
		
		for (i=(PACKET_HEADER_NUM + dataLength +2 ) ; i < (PACKET_LENGTH - 2) ; i++)
		{
			packetString[ i ]= 0x00;
		}
		
		packetString[PACKET_LENGTH  - 2 ] = (crcPacket >> 8) ;
		packetString[PACKET_LENGTH - 1 ] = (crcPacket & 0xFF );
		//packetString[21] =0xFF;
	}
	else
	for (i=0 ; i<PACKET_LENGTH ; i++ )
	{
		packetString[i]= 0x00 ;
	}
	
	
}

bool Packet::extractData(char *packetString ,int &dataLength)
{
	unsigned char i;
	int crcRecived=0, crcClaculated, tempDataLength =0;
	
	//serialDataBuffer.writeByte(packetString[18]);
	//for(int i=0 ; i<6 ; i++)
		//serial.putChar(packetString[i]);
		//serialTxDataBuffer.writeString(packetString , 6);
		
	if (isHeaderDetected(packetString))
	{		
		tempDataLength |= (packetString[PACKET_HEADER_NUM] & 0xFF);
		tempDataLength <<=8;
		tempDataLength |= (packetString[PACKET_HEADER_NUM + 1] & 0xFF );
		
		if (tempDataLength > DATA_LENGTH )
		{
			//serial.putChar(2);
			return false;
		}
		
		dataLength = tempDataLength;
		
		crcClaculated = calculatedCRC(packetString+PACKET_HEADER_NUM+2 , tempDataLength);
		
		//serialTxDataBuffer.writeByte(crcClaculated);
		//serialTxDataBuffer.writeByte(crcClaculated >> 8);
		
		for (i=0; i< tempDataLength ; i++)
		packetString[i] = packetString[i + PACKET_HEADER_NUM + 2 ];

		crcRecived |= (packetString[PACKET_LENGTH  - 2 ] & 0xFF) ;
		crcRecived <<= 8;
		crcRecived |=(packetString[PACKET_LENGTH  - 1 ]  & 0xFF);
		
		if (crcRecived == crcClaculated)
		{
			//serial.putChar(1);
			return true;
		}
	}
	//serial.putChar(0);
	return false;
}

bool Packet::isHeaderDetected( char *string )
{
	unsigned char i;
	for (i=0; i < PACKET_HEADER_NUM ; i++)
	{
		if (string[i] != header[i])
		{
			return false;
		}
	}
	return true;
}

int Packet::calculatedCRC( char *string , int strSize )
{
	unsigned char i;
	int calcCRC=0;
	
	for (i=0; i < strSize ; i++)
	{
		calcCRC += string[i];
	}
	return calcCRC;
}

