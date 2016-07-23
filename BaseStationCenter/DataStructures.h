/*
 * DataStructures.h
 *
 * Created: 3/8/2015 10:19:32 AM
 *  Author: Mobarezi
 */ 


#ifndef DATASTRUCTURES_H_
#define DATASTRUCTURES_H_

#include "definition.h"

struct BaseGetStatusPacket
{
	char frameHeader;
	char messageType;
	char checksum[2];
	char frameTrailer;
};

struct Station
{
	char stationNumber;
	char SNR;
	int tow;
	short int weekNumber;
	float latitude;
	float longitude;
	float altitude;
	float lastLocked;
	float stability;
	float voltage;
	float current;
	float temperature;
	float battery;
};


#endif /* DATASTRUCTURES_H_ */