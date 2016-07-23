/* 
* ADF7020.h
*
* Created: 7/28/2015 8:52:42 AM
* Author: CIT_15
*/


#ifndef __ADF7020_H__
#define __ADF7020_H__

#include <stdint.h>

enum TranceiverMode
{
	TRANSCEIVER_MODE_RECEIVER,
	TRANSCEIVER_MODE_TRANSMITTER	
};

enum RFPowerState
{
	RF_POWER_STATE_HIGH,
	RF_POWER_STATE_LOW,
	RF_POWER_STATE_MEDIUM
};

class ADF7020
{
//variables
public:
protected:
private:
	 TranceiverMode mode;
	 RFPowerState rfPowerState;
	char sendBuffer;
	char sendBufferShadow;
	char receivedByteShadow;
	char receivedByte;
	char sendBitIndex;
	char receiveBitIndex;
	char oldTransmitBit;
	bool transmitBufferEmpty;
	bool receiveDataFlag;
	bool receivePreambleDetect;
	char timerCounter;

//functions
public:
	ADF7020();
	~ADF7020();
	void setMode(TranceiverMode mode);
	void ADF_Program_Check(void);
	unsigned int adfReadback(char readBackMode );
	void receiveInt1ISR();
	void receiveInt2ISR();
	void transmitTimerISR();
	void adf7020_ChangeMode(TranceiverMode mode);
	
	void clearPreaambleDetect();
	void sendByte(char data);
	char receiveByte();
	void changeRfPower(RFPowerState state);
	void calculateRSSI(uint16_t rssi);
	
	bool isReceiver(); 
	bool isNewDataAvailable();
	bool isTransmitterReady();
	bool isPreambleDetected();
	
	
protected:
private:
	ADF7020( const ADF7020 &c );
	ADF7020& operator=( const ADF7020 &c );
	void adf7020_Initialize();
	unsigned int ADF_ReadBack_Format(char readback_format);
	void Set_Register(unsigned long Data);
	void transmitBit();
	void receiveBit();
	
}; //ADF7020

#endif //__ADF7020_H__
