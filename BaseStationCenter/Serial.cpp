/*
 * Serial.cpp
 *
 * Created: 3/8/2015 12:09:25 PM
 *  Author: Mobarezi
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

#include "Serial.h"
#include "GlobalVariables.h"
#include "definition.h"
#include "functions.h"

/*#ifndef F_CPU
	#define F_CPU 8000000l
#endif*/

void Serial::init(int baudRate)
{
	//// USART initialization
	//// Communication Parameters: 8 Data, 1 Stop, No Parity
	//// USART Receiver: On
	//// USART Transmitter: On
	//// USART Mode: Asynchronous
	//// USART Baud Rate: 4800
	//UCSRA=0x00;
	//UCSRB=0x98;
	//UCSRC=0x86;
	//UBRRH=0x00;
	//UBRRL=0x67;
	switch (baudRate)
	{
		case 9600 :
			// USART initialization
			// Communication Parameters: 8 Data, 1 Stop, No Parity
			// USART Receiver: On
			// USART Transmitter: On
			// USART Mode: Asynchronous
			// USART Baud Rate: 9600
			UCSRA=0x00;
			UCSRB=0x18;
			UCSRC=0x86;
			UBRRH=0x00;
			UBRRL=0x33;
			break;
			
		case 19200:
			
			// USART initialization
			// Communication Parameters: 8 Data, 1 Stop, No Parity
			// USART Receiver: On
			// USART Transmitter: On
			// USART Mode: Asynchronous
			// USART Baud Rate: 19200
			UCSRA=0x00;
			UCSRB=0x18;
			UCSRC=0x86;
			UBRRH=0x00;
			UBRRL=0x19;
			break;
		default:
			// USART initialization
			// Communication Parameters: 8 Data, 1 Stop, No Parity
			// USART Receiver: On
			// USART Transmitter: On
			// USART Mode: Asynchronous
			// USART Baud Rate: 9600
			UCSRA=0x00;
			UCSRB=0x18;
			UCSRC=0x86;
			UBRRH=0x00;
			UBRRL=0x33;
			break;
		
	}
	
	
	//// USART initialization
	//// Communication Parameters: 8 Data, 1 Stop, No Parity
	//// USART Receiver: On
	//// USART Transmitter: On
	//// USART Mode: Asynchronous
	//// USART Baud Rate: 14400
	//UCSRA=0x00;
	//UCSRB=0x98;
	//UCSRC=0x86;
	//UBRRH=0x00;
	//UBRRL=0x22;
	//// USART initialization
	//// Communication Parameters: 8 Data, 1 Stop, No Parity
	//// USART Receiver: Off
	//// USART Transmitter: On
	//// USART Mode: Asynchronous
	//// USART Baud Rate: 38400
	//UCSRA=0x00;
	//UCSRB=0x08;
	//UCSRC=0x86;
	//UBRRH=0x00;
	//UBRRL=0x0C;
	//// USART initialization
	//// Communication Parameters: 8 Data, 1 Stop, No Parity
	//// USART Receiver: On
	//// USART Transmitter: On
	//// USART Mode: Asynchronous
	//// USART Baud Rate: 19200
	//UCSRA=0x00;
	//UCSRB=0x98;
	//UCSRC=0x86;
	//UBRRH=0x00;
	//UBRRL=0x19;


}

void Serial::putChar(char data)
{
	// Wait for empty transmit buffer
	while ( !(UCSRA & (_BV(UDRE))) );
	// Start transmission
	UDR = data;
}

bool Serial::isSerialTxEmpty()
{
	return (UCSRA & (_BV(UDRE)));
}

void Serial::put16Bit(int data)
{
	putChar((data >>8) & 0xFF);
	putChar(data & 0xFF);
	
}

unsigned char Serial::getChar()
{
	// Wait for incoming data
	while ( !(UCSRA & (_BV(RXC))) );
	// Return the data
	return UDR;
}

void Serial::putStr(const char *s)
{
	// loop through entire string
	while (*s) {
		putChar(*s);
		s++;
	}
}

void Serial::sendPacket(char* pkt, int length)
{
	for (int i=0; i<length; i++)
		putChar(pkt[i]);
}

bool Serial::isSerialRxFull()
{
	return (UCSRA & (_BV(RXC)));
}


