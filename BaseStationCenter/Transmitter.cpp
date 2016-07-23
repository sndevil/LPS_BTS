/* 
* Transmitter.cpp
*
* Created: 7/27/2015 6:16:44 PM
* Author: CIT_15
*/


#include "Transmitter.h"

// default constructor
Transmitter::Transmitter()
{
} //Transmitter

// default destructor
Transmitter::~Transmitter()
{
} //~Transmitter

ISR(TIMER1_COMPA_vect)
// Timer 1 output compare A interrupt service routine
//interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
	//
	//
	TCNT1H=0x00;
	TCNT1L=68;

	//Led2_WR(1-Led2_RD());
	
	//_delay_us(10);
	//Data_IO_WR(1-Data_IO_RD());
	//    Led1 = 1-Led1;


	//char i=0;
	count ++;
	if(count ==0)
	time_count ++;
	chek_sum=0;
	if (send_flag ==1)
	{
		Led2_WR(1-Led2_RD());
		if(RxData[ByteCounter] != NULL)
		{

			
			if (Bit<8 )
			{
				test_data <<= 1;
				Data_IO_WR((Temp & 0x80) == 0x80);
				test_data +=(Temp & 0x80) == 0x80;
				
				chek_sum +=((Temp & 0x80) == 0x80);
				Temp <<= 1;
				
				Bit++;
			}
			if (Bit==8)
			{
				//serial.putChar((char)Bit);
				Bit=0;
				//cli();
				//serial.putChar( (char)Temp);
				//serial.putChar( (char)test_data);
				//sei();
				Temp = RxData[ByteCounter];
				
				
				test_data =0;
				ByteCounter++;
				//serial.putChar((char)ByteCounter);
			}
			
		}
		else
		{
			ByteCounter =0;
			//_delay_ms(200);
			send_flag =0;
			//Bit=0;
		}
		
		// serial.putChar((char)chek_sum);
	}
	//Data_IO_WR((chek_sum & 0x01) == 0x01);
	
	//serial.putChar((char)send_flag);
	//if(send_flag==1)
	//{
	//if (ByteCounter<PACKET_LENTH && check_flag==0)
	//{
	////serial.putStr("Sending!");
	//Data_IO_WR((Temp & 0x80) == 0x80);
	//if(ByteCounter>7)
	//chek_sum += Data_IO_RD();
	//Temp<<=1;
	//Bit++;
	//if (Bit==8)
	//{
	//Bit = 0 ;
	//ByteCounter++;
	//if (ByteCounter == PACKET_LENTH)
	//{
	//ByteCounter=0;
	////              send_flag=0;
	//check_flag=2;
	//Packet_counter++;
	//}
	//Temp = RxData[ByteCounter];
	//serial.putChar(Temp);
	//}
	//}
	//else
	//{
	////Data_IO_WR((chek_sum & 0x8000) == 0x8000);
	//Data_IO_WR(1);
	//chek_sum<<=1;
	//Bit++;
	//if (Bit==8)
	//{
	//Bit = 0 ;
	//check_flag--;
	//if (check_flag==0)
	//{
	//if(Packet_counter>Packet_Num)
	//{
	////                send_flag=0;
	//Gps_flag=0;
	//Packet_counter=0;
	//delay_flag=1;
	////                Led1=1-Led1;
	//PA_enable_WR(0);
	////                _delay_ms(3000);
	//}
	//
	//chek_sum=0;
	//send_flag=0;
	////					PA_enable=0;
	//}
	//}
	//}
	//}
	//else
	//{
	//Data_IO_WR(1-Data_IO_RD());
	////Data_IO_WR(1);
	//}/**/
}
