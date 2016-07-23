/*
 * GlobalVariables.h
 *
 * Created: 3/8/2015 12:47:01 PM
 *  Author: Mobarezi
 */ 


#ifndef GLOBALVARIABLES_H_
#define GLOBALVARIABLES_H_

#include "Serial.h"
#include "Message.h"
#include "ADF7020.h"
#include "definition.h"
#include "FIFO.h"
#include "Transceiver.h"
#include "StationInfo.h"


extern Serial serial;
//extern Message message;
//extern Station stations[10];
extern Packet packetProcessor;

//extern ADF7020 Board0;
extern Transceiver transceiver;
extern FIFO serialTxDataBuffer,serialRxDataBuffer;
extern StationInfo stationInfo;
extern int test_index;

extern char intrruptFlag;
extern bool ltrTurnOnFlag;

#endif /* GLOBALVARIABLES_H_ */