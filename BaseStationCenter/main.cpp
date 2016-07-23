/*
* BaseStationCenter.cpp
*
* Created: 3/8/2015 9:35:52 AM
*  Author: Zamani
*/

#define F_CPU 8000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "GlobalVariables.h"
#include "definition.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "functions.h"
#include "FIFO.h"
#include "Packet.h"
#include "StationInfo.h"
#include <avr/wdt.h>



char receivedPacket[PACKET_LENGTH +1],command[PACKET_LENGTH+ 1] ,commandPacket[PACKET_LENGTH + 1] ;

int main(void)
{
	char i=0 , intTemp = 0 , temp ;
	
	int dataLength , baseStationNumber,timeoutCounter;
	unsigned int rssivalue;
	int ret;

	//rssivalue = 0x3BF;
	//ret = transceiver.calculateRSSI(rssivalue);
	
	loadBS();
	
	serial.init(19200);
	
	micro_Initialize();
	
	resetGlobalIntrrupt();
	
	transceiver.setMode(TRANSCEIVER_MODE_RECEIVER);

	for (i=0;i<3 ;i++)
	{
		setLED(0);
		_delay_ms(50);
		resetLED(0);
		_delay_ms(50);
	}
	resetLED(0);

	if( (PIND & 0x04) >> 2)
	setLED(0);
	_delay_ms(500);
	setLED(0);
	
	setGlobalIntrrupt();


	i=0;
	
	baseStationNumber =0;
	//setLED(1);
	//transceiver.calculateRSSI(0x73D);
	while (1)
	{
		processSerialReceivedBytes();
		
		if ((serialTxDataBuffer.getFifoFullLength() > 0 ) )
		if (serial.isSerialTxEmpty())
		{
			if(serialTxDataBuffer.readByte(temp))
				serial.putChar(temp);
		}
			
			
		ToggleLED(0);
		if (transceiver.isReceiver())
		{
			
			if (transceiver.isPreambleDetected())
			{
				rssivalue = transceiver.adfReadback(RSSI_READBACK);
				stationInfo.stationInfoStructure.rssiBaseStation = rssivalue;
				transceiver.calculateRSSI(rssivalue);
			}
			
			if (transceiver.getFifoFullLength() > PACKET_LENGTH - 1  )
			{
				resetLED(2);
				transceiver.readReceivedPacket(receivedPacket);
				if (packetProcessor.extractData(receivedPacket , dataLength))
					receivedDataProcess(receivedPacket , dataLength);
				
			}
			
		}
		else
		{
			setLED(2);			
			if (transceiver.getFifoFullLength() == 0)
				transceiver.changeMode();
				
		}
		
	};
}

//***********************************************************************


