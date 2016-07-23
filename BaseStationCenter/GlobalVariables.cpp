/*
 * GlobalVariables.cpp
 *
 * Created: 3/8/2015 12:47:10 PM
 *  Author: Mobarezi
 */ 

#include "GlobalVariables.h"
#include "Serial.h"
#include "Transceiver.h"
#include "ADF7020.h"

Serial serial;
//Message message;
//Station stations[10];
Packet packetProcessor;

//ADF7020 Board0 ;
Transceiver transceiver;

FIFO serialTxDataBuffer,serialRxDataBuffer;
StationInfo stationInfo;

int test_index;
char intrruptFlag;
bool ltrTurnOnFlag;