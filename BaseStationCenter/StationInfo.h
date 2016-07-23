/* 
* StationInfo.h
*
* Created: 9/20/2015 11:59:16 AM
* Author: CIT_15
*/


#ifndef __STATIONINFO_H__
#define __STATIONINFO_H__

#include <stdint.h>

struct StationInfoStructure
{
	char startByte;
	char commandCode;
	char stationNumber;
	char voltage;
	char current;
	char stability;
	char temperature;
	char humidity;
	char battery;
	char lastLock;
	char statusFlag;
	char macNumber;
	long int xCordinate;
	long int yCordinate;
	long int zCordinate;
	long int time;
	long int assignFlyingObject;
	long int readyFlyingObject;
	int16_t rssiBaseStation;
	int16_t rssiCenterStation;
	char snrValue[16];
	char LTRHealth;
	int16_t crcByte;
	char treailer;
};

class StationInfo
{
//variables
public:
	StationInfoStructure stationInfoStructure;
	char* structStartPointer;
protected:
private:
	
//functions
public:
	StationInfo();
	~StationInfo();
	int16_t calculateCRC();
protected:
private:
	StationInfo( const StationInfo &c );
	StationInfo& operator=( const StationInfo &c );
	

}; //StationInfo

#endif //__STATIONINFO_H__
