/* 
* Receiver.cpp
*
* Created: 7/27/2015 6:18:37 PM
* Author: CIT_15
*/


#include "Receiver.h"
#include <avr/interrupt.h>

// default constructor
Receiver::Receiver()
{
} //Receiver

// default destructor
Receiver::~Receiver()
{
} //~Receiver

void Receiver::isr()
{
	
	if (Datain_flag ==1)
	{
		//cli();
		if (Bit == 8)
		{
			//serial.putChar(char(Data));
			Led2_WR(1-Led2_RD());
			Bit ==0;
			
			Received_Data[ByteCounter]=Data;
			Data=0;
			ByteCounter++;
			if (ByteCounter > PACKET_LENTH )
			{
				Datain_flag=0;
				ByteCounter=0;
				Packet_count++;
			}
		}
		
		Data <<= 1;
		if (Data_IO)
		Data += 1;
		Bit ++;
		//Led1_WR(1-Led1_RD());
		
	}
	
	//sei();	
	
}

//////////////////////////////////////////////////////////////////////////
// External Interrupt 1 service routine
ISR(INT1_vect)
{
	receiver0.isr();
}


// External Interrupt 2 service routine
ISR(INT2_vect)
{
	
}



