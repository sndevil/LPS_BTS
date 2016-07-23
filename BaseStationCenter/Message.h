/*
 * Message.h
 *
 * Created: 3/8/2015 10:36:11 AM
 *  Author: Mobarezi
 */ 


#ifndef MESSAGE_H_
#define MESSAGE_H_

#include "DataStructures.h"

class Message
{
private:
	BaseGetStatusPacket baseGetStatusPacket;
	int calculateCRC(unsigned char *ptr, int count);
	void constructBaseGetStatusPacket();
public:
	void sendBaseGetStatusPacket();
};



#endif /* MESSAGE_H_ */