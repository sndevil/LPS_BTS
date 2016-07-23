/* 
* StationInfo.cpp
*
* Created: 9/20/2015 11:59:16 AM
* Author: CIT_15
*/


#include "StationInfo.h"

// default constructor
StationInfo::StationInfo()
{
	structStartPointer = (char*)&stationInfoStructure;
	stationInfoStructure.stationNumber = -1;
} //StationInfo

// default destructor
StationInfo::~StationInfo()
{
} //~StationInfo

int16_t StationInfo::calculateCRC()
{
	int16_t calcCRC=0;
	char i;
	
	for (i=0; i < sizeof(stationInfoStructure) ; i++)
	{
		calcCRC += structStartPointer[i];
	}
	return calcCRC;
}
