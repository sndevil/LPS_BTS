/* 
* Transmitter.h
*
* Created: 7/27/2015 6:16:45 PM
* Author: CIT_15
*/


#ifndef __TRANSMITTER_H__
#define __TRANSMITTER_H__

#define PACKET_LENTH	30		//Byte
#define	PREAMBLE_LENTH	8
#define Max_Rand  200
#define Min_Rand  100
#define Delay 3000     //delay after sending Packet_num packets
#define Packet_Num 10  //number of data packets that must be sned after locking GPS.

class Transmitter
{
//variables

public:

protected:
private:

//functions
public:
	Transmitter();
	~Transmitter();
protected:
private:
	Transmitter( const Transmitter &c );
	Transmitter& operator=( const Transmitter &c );

}; //Transmitter

#endif //__TRANSMITTER_H__
