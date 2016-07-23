/* 
* FIFO.h
*
* Created: 8/1/2015 7:38:17 PM
* Author: CIT_15
*/


#ifndef __FIFO_H__
#define __FIFO_H__

#include "definition.h"

enum FIFOSTATUS 
{
	FIFO_EMPTY,
	FIFO_HALF_EMPTY,
	FIFO_FULL	
};

class FIFO
{
//variables
public:
protected:
private:
	char fifoBuffer[MAX_FIFO_LENGTH+1];
	int readIndex;
	int writeIndex;

//functions
public:
	FIFO();
	~FIFO();
	bool readByte(char &data);
	void writeByte(char data);
	int readString(char *str,int strSize);
	void writeString(char *str,int strSize);
	int getFifoFullLength();
	int getFifoFreeLength();
	void clearFifo();
protected:
	
private:
	FIFO( const FIFO &c );
	FIFO& operator=( const FIFO &c );

}; //FIFO

#endif //__FIFO_H__
