/* 
* FIFO.cpp
*
* Created: 8/1/2015 7:38:17 PM
* Author: CIT_15
*/


#include "FIFO.h"
#include "definition.h"
#include "functions.h"
#include <avr/interrupt.h>
// default constructor
FIFO::FIFO()
{
	readIndex = 0;
	writeIndex = 0;
} //FIFO

// default destructor
FIFO::~FIFO()
{
} //~FIFO

bool FIFO::readByte(char &data)
{
	//char temp = 0;
	if(getFifoFullLength() > 0 )
	{
		data = fifoBuffer[readIndex];
		readIndex ++ ;
		if ( readIndex > MAX_FIFO_LENGTH )
			readIndex =0;
		return true;
	}
	return false;
}

void FIFO::writeByte( char data )
{
	//if (getFifoFullLength() < MAX_FIFO_LENGTH )
	{
		fifoBuffer [writeIndex] = data;
		writeIndex ++ ;
		if ( writeIndex > MAX_FIFO_LENGTH )
			writeIndex =0;
	}
	
}

int FIFO::readString( char *str,int strSize )
{
	char temp;
	int length = 0;
	
	if ( strSize > MAX_FIFO_LENGTH )
	{
		strSize = MAX_FIFO_LENGTH ;
	}
	
	while (strSize)
	{
		if(!readByte(temp))
			return length;
		length ++ ;
		*str = temp;
		str++;
		strSize --;
	};
	return length;
}

void FIFO::writeString( char *str,int strSize )
{
	if ( strSize > MAX_FIFO_LENGTH )
	{
		strSize = MAX_FIFO_LENGTH ;
	}
	
	while (strSize)
	{
		writeByte(*str);
		str++;
		strSize --;
	};
}



int FIFO::getFifoFullLength()
{
	
	int length=0;
	char interruptFlag ;
	
	interruptFlag = resetAndStoreIntrruptFlag();
	
	if (writeIndex >= readIndex)
		length =	writeIndex - readIndex;
	else
		length = MAX_FIFO_LENGTH + 1 - (readIndex - writeIndex);
	
	restoreIntrrupt(interruptFlag);
	
	return length;
}

int FIFO::getFifoFreeLength()
{
	int length;
	//cli();
	length = (MAX_FIFO_LENGTH + 1 - getFifoFullLength());
	//sei();
	return length;
}

void FIFO::clearFifo()
{
	writeIndex=readIndex=0;
}
