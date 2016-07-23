/* 
* Receiver.h
*
* Created: 7/27/2015 6:18:38 PM
* Author: CIT_15
*/


#ifndef __RECEIVER_H__
#define __RECEIVER_H__

#define RXB8 1
#define TXB8 0
#define UPE 2
#define OVR 3
#define FE 4
#define UDRE 5
#define RXC 7

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)

class Receiver
{
//variables

public:
protected:
private:
	char ByteCounter=0,Bit=0,Datain_flag=0,Received_Data[PACKET_LENTH+2],Data,i=0,Packet_flag=0,hh=0,ss=0,mm=0;
	char data_valid=0,Packet_count=0,check_flag=0,Gps_counter=0,clock_flag=0,gps_update=0;
	int check_sum=0,sum=0;
	char update_user, Menu_initialize,fflag=0;

//functions
public:
	Receiver();
	~Receiver();
	void isr();
protected:
private:
	Receiver( const Receiver &c );
	Receiver& operator=( const Receiver &c );

}; //Receiver


#endif //__RECEIVER_H__
