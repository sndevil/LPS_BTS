/* 
* ADF7020.cpp
*
* Created: 7/28/2015 8:52:42 AM
* Author: CIT_15
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>

#include "ADF7020.h"
#include "definition.h"
#include "ADF_Structures.h"
#include "GlobalVariables.h"
#include "functions.h"

// default constructor
ADF7020::ADF7020()
{
	 sendBuffer=0;
	receivedByteShadow=0;
	receivedByte=0;
	sendBitIndex=0;
	receiveBitIndex=0;
	oldTransmitBit=0;
	transmitBufferEmpty=0;
	receiveDataFlag=0;
	receivePreambleDetect=0;
	rfPowerState = RF_POWER_STATE_HIGH ;
} //ADF7020

// default destructor
ADF7020::~ADF7020()
{
} //~ADF7020

void ADF7020::Set_Register(unsigned long regData)
{
	char ii;
	Sl_En_WR(0); //While Sending SData SLE must be 0
	S_clk_WR(0); // SData Must be Valid at Clock Rising Edge
	for (ii = 0 ; ii<32 ; ii++)
	{
		S_Data_WR((regData & 0x80000000) == 0x80000000 ); // Must send MSB
		S_clk_WR(1);
		_delay_us(5);
		regData <<= 1;
		S_clk_WR(0); // SData Must be Valid at Clock Rising Edge
		_delay_us(5);
	}
	
	Sl_En_WR(1); // After each 32 bits SLE must become 1
	_delay_us(5);
	Sl_En_WR(0); // For Protecting from Unwanted Data Programming SLE = 0
}

//////////////////////////////////////////////////////////////////////////



void ADF7020::adf7020_Initialize()
{
	unsigned long  i=0, data_reg;
	//unsigned long Reg_Data;
	//Cheap Enable
	CE_WR(0);
	_delay_us(1);
	CE_WR(1);
	//delay_ms(100);
	//R0 Setting
	if (mode == TRANSCEIVER_MODE_RECEIVER)
	{
		//set data and clock pins as input.
		DDRD=0x82; //0x3A
		
		PA_ENABLE_WR(0);
		_delay_us(20);

		//RF3023 Setting: RX: V1=1,V2=0 ---- Tx V1=0,V2=1;
		TR_SWV1_WR(1);
		TR_SWV2_WR(0);
		
		//HMC646 Setting: 1 = Tx Enable
		TR_SW_WR(0);
	   
		ADF_Regs.R0_N.bit1.Fractional_N_DivideRatio =15845;//31130;//4096;
		ADF_Regs.R0_N.bit1.Integer_N_DivideRatio = 43;//106;
		ADF_Regs.R0_N.bit1.Tx_Rx_bit = 1;
	}
	else
	{
		//set data and clock port as aoutput.
		DDRD=0xBA;
		//RF3023 Setting: RX: V1=1,V2=0 ---- Tx V1=0,V2=1;
		TR_SWV1_WR(1);
		TR_SWV2_WR(0);
		
		//HMC646 Setting: 1 = Tx Enable
		TR_SW_WR(1);
		
		_delay_us(1);
		PA_ENABLE_WR(1);
		
		ADF_Regs.R0_N.bit1.Fractional_N_DivideRatio =16500;//16500;//4096;//22938;
		ADF_Regs.R0_N.bit1.Integer_N_DivideRatio =43;//107//22 in rx
		ADF_Regs.R0_N.bit1.Tx_Rx_bit = 0;
	}
	ADF_Regs.R0_N.bit1.PLL_enablebit =1;
	ADF_Regs.R0_N.bit1.MUXOUT =3;
	
	R0_Generate(ADF_Regs.R0_N.bit1,data_reg);
	Set_Register(data_reg);
	//Set_Register(0x76b80000);
	
	//_delay_ms(1);
	//Led0=PIND.2;

	//R1 Setting
	//    Led1=0;
	ADF_Regs.R1_OSC_Filter.bit1.Rcounter_DivideRatio =2;///1 in rx
	ADF_Regs.R1_OSC_Filter.bit1.XTAL_Doublerbit =0;
	ADF_Regs.R1_OSC_Filter.bit1.CLKOUT_DivideRatio =0;
	ADF_Regs.R1_OSC_Filter.bit1.XOSC_enablebit =0;
	ADF_Regs.R1_OSC_Filter.bit1.VCO_Band =0;
	ADF_Regs.R1_OSC_Filter.bit1.CP_Current =2;
	ADF_Regs.R1_OSC_Filter.bit1.VCO_Bias =5;//3
	ADF_Regs.R1_OSC_Filter.bit1.VCO_Adjust =3;
	ADF_Regs.R1_OSC_Filter.bit1.IF_FilterBW =0;
	R1_Generate(ADF_Regs.R1_OSC_Filter.bit1,data_reg);
	//serial.put16Bit(0xAAAB);
	//serial.putChar(0x01);
	//serial.put16Bit(data_reg >>16);
	//serial.put16Bit(data_reg );
	//serial.put16Bit(0xBBBB);
	Set_Register(data_reg);
	//Set_Register(0xAA051);
	//Set_Register(0x4ABA21);
	//_delay_ms(1);
	//    Led1=1;

	//R2 Setting for FSK mode
	//    Led1=0;
	//    ADF_Regs.R2_TransmitModulation_FSK.bit1.Add = 0;
	ADF_Regs.R2_TransmitModulation_FSK.bit1.PA_enablebit = 1;
	ADF_Regs.R2_TransmitModulation_FSK.bit1.MutePA_until_Lock = 0;
	ADF_Regs.R2_TransmitModulation_FSK.bit1.Modulation_Scheme = 0;
	ADF_Regs.R2_TransmitModulation_FSK.bit1.PA_OutLevel = 63;
	ADF_Regs.R2_TransmitModulation_FSK.bit1.fDeviation =33; //20 K deviation
	ADF_Regs.R2_TransmitModulation_FSK.bit1.NotUsed = 0;
	ADF_Regs.R2_TransmitModulation_FSK.bit1.TxDataInvertbit = 0;//0;
	ADF_Regs.R2_TransmitModulation_FSK.bit1.PA_Bias = 3;
	R2_Generate_FSK(ADF_Regs.R2_TransmitModulation_FSK.bit1, data_reg);
	Set_Register(data_reg);
	//Set_Register(0x8010FE12);
	//Set_Register(0x80527E12);
	//_delay_ms(1);
	//    Led1=1;

	//R3 Setting
	ADF_Regs.R3_ReceiverCLK.bit1.BBOS_CLK_Divide =2;		//2  for 9.6k bitrate and 8  for 14.71k
	ADF_Regs.R3_ReceiverCLK.bit1.DEMOD_CLK_Divide =2;
	

	ADF_Regs.R3_ReceiverCLK.bit1.CDR_CLK_Divide =33;		//26 for 9.6k bitrate and 17 for 14.71k

	ADF_Regs.R3_ReceiverCLK.bit1.SEQ_CLK_Divide =200;
	R3_Generate(ADF_Regs.R3_ReceiverCLK.bit1, data_reg);
	Set_Register(data_reg);

	//_delay_ms(1);


	//R4 Setting
	ADF_Regs.R4_Demod_Setup.bit1.Demod_Select =1;
	
	ADF_Regs.R4_Demod_Setup.bit1.PostDemod_BW =5; //5;
	ADF_Regs.R4_Demod_Setup.bit1.Demod_Lock_Timeout =0;
	ADF_Regs.R4_Demod_Setup.bit1.Sync_Mode =1;
	R4_Generate(ADF_Regs.R4_Demod_Setup.bit1, data_reg);
	Set_Register(data_reg);
	
	//Set_Register(0x1000154);
	//Set_Register(0x00214);

	//_delay_ms(1);

	//R5 Setting
	ADF_Regs.R5_Sync_Word.bit1.Sync_Length =3;
	ADF_Regs.R5_Sync_Word.bit1.Match_Tolerance =0;
	ADF_Regs.R5_Sync_Word.bit1.Sync_Word_Byte =0x94D2AD;//same as 0x123456
	R5_Generate(ADF_Regs.R5_Sync_Word.bit1, data_reg);
	Set_Register(data_reg);
	//Set_Register(0xAAAA15);
	//Set_Register(0x12345635);

	//_delay_ms(1);
	//R6 Setting
	ADF_Regs.R6_Correlator_Demod.bit1.Discrimin_BW =125;//0x0FA;	//110 for 19.2K deviation and 0x46 for 29K
	ADF_Regs.R6_Correlator_Demod.bit1.Dot_Product =0;		//0  for 19.2K deviation and 1    for 29K
	ADF_Regs.R6_Correlator_Demod.bit1.LNA_Mode =0;
	ADF_Regs.R6_Correlator_Demod.bit1.LNA_Current =0;
	ADF_Regs.R6_Correlator_Demod.bit1.Mixer_Linearity =0;
	ADF_Regs.R6_Correlator_Demod.bit1.IF_FilterCal =0;
	ADF_Regs.R6_Correlator_Demod.bit1.IF_FilterDivider =0x190;
	ADF_Regs.R6_Correlator_Demod.bit1.RxData_InvertBit =1;
	ADF_Regs.R6_Correlator_Demod.bit1.Demod_ResetBit =0;
	ADF_Regs.R6_Correlator_Demod.bit1.Cdr_ResetBit =0;
	R6_Generate(ADF_Regs.R6_Correlator_Demod.bit1, data_reg);
	Set_Register(data_reg);
	//Set_Register(0x39000FA6);
	//Set_Register(0x19004586);
	//_delay_ms(1);
////R8 Setting
//ADF_Regs.R8_PowerDown.bit1.VCO_Synth_enablebits=0;
//ADF_Regs.R8_PowerDown.bit1.LNA_Mixer_enablebit=0;
//ADF_Regs.R8_PowerDown.bit1.Filter_enablebit=0;
//ADF_Regs.R8_PowerDown.bit1.ADC_enablebit =1;
//ADF_Regs.R8_PowerDown.bit1.Demod_Enablebit =0;
//ADF_Regs.R8_PowerDown.bit1.RSSI_Mode=0;
//ADF_Regs.R8_PowerDown.bit1.Tx_Rx_Switch=0;
//ADF_Regs.R8_PowerDown.bit1.PA_RxMode=0;
//
//R8_Generate(ADF_Regs.R8_PowerDown.bit1, data_reg);
//Set_Register(data_reg);
//Set_Register(0x39000FA6);
//Set_Register(0x19004586);

	//R11 Setting
	ADF_Regs.R11_AFC.bit1.AFC_ScaleCoefficient =0x01a3;
	ADF_Regs.R11_AFC.bit1.AFC_enableBit =1;
	R11_Generate(ADF_Regs.R11_AFC.bit1, data_reg);
	Set_Register(data_reg);
	//Set_Register(0x101a3b);
	//    Set_Register(0x00102F7B);
	//_delay_ms(1);
	//R9 Setting
	ADF_Regs.R9_AGC.bit1.AGC_LowThreshold =0x1e;
	ADF_Regs.R9_AGC.bit1.AGC_HighThreshold =0x46;
	ADF_Regs.R9_AGC.bit1.AGC_Search =0;
	ADF_Regs.R9_AGC.bit1.Gain_Control =0;
	ADF_Regs.R9_AGC.bit1.LNA_Gain =3;
	ADF_Regs.R9_AGC.bit1.Filter_Gain =2;
	ADF_Regs.R9_AGC.bit1.Filter_Current =0;
	R9_Generate(ADF_Regs.R9_AGC.bit1, data_reg);
	Set_Register(data_reg);
	//Set_Register(0xb231e9);
	//Reg_Data = 0xB231E9;
	//_delay_ms(1);
}

//////////////////////////////////////////////////////////////////////////
void ADF7020::ADF_Program_Check(void)
{
	int i;
	unsigned long  data_reg;
	
	for (i=1;i<=5;i++)
	{
		ADF_Regs.R0_N.bit1.MUXOUT =1;
		R0_Generate(ADF_Regs.R0_N.bit1,data_reg);
		Set_Register(data_reg);
		_delay_ms(1);
		if( (PIND & 0x04) >> 2)
			ToggleLED(0);
		
		_delay_ms(300);
		
		ADF_Regs.R0_N.bit1.MUXOUT =0;
		R0_Generate(ADF_Regs.R0_N.bit1,data_reg);
		Set_Register(data_reg);
		if( (PIND & 0x04) >> 2)
			ToggleLED(0);;
		
		_delay_ms(300);
	};
	resetLED(0);
	ADF_Regs.R0_N.bit1.MUXOUT =3;
	R0_Generate(ADF_Regs.R0_N.bit1,data_reg);
	Set_Register(data_reg);
	_delay_ms(1);
	if( (PIND & 0x04) >> 2)
		setLED(0);
	//ADF_ReadBack_Format(SILICON_REV);
}
//////////////////////////////////////////////////////////////////////////
unsigned int ADF7020::adfReadback(char readbackMode)
{
	char i;
	uint16_t regData=0;
	
	switch (readbackMode)
	{
		case AFC_READBACK:
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_Mode =0;
		ADF_Regs.R7_ReadbackSetup.bit1.ADC_Mode =0;
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_enablebit =1;
		break;
		case RSSI_READBACK:
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_Mode = 1;
		ADF_Regs.R7_ReadbackSetup.bit1.ADC_Mode = 0;
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_enablebit = 1;
		break;
		case BATTERY_VLOTAGE_READBACK:
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_Mode = 1;
		ADF_Regs.R7_ReadbackSetup.bit1.ADC_Mode = 1;
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_enablebit = 1;
		break;
		case TEMPERATURE_SENSOR_READBACK:
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_Mode = 1;
		ADF_Regs.R7_ReadbackSetup.bit1.ADC_Mode = 2;
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_enablebit = 1;
		break;
		case ADCIN_VOLTAGE_READBACK:
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_Mode = 1;
		ADF_Regs.R7_ReadbackSetup.bit1.ADC_Mode = 3;
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_enablebit = 1;
		break;
		case SILICON_REVISION_READBACK:
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_Mode = 3;
		ADF_Regs.R7_ReadbackSetup.bit1.ADC_Mode = 0;
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_enablebit = 1;
		break;
		case FILTER_CALIBRETION_READBACK:
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_Mode = 2;
		ADF_Regs.R7_ReadbackSetup.bit1.ADC_Mode = 0;
		ADF_Regs.R7_ReadbackSetup.bit1.Readback_enablebit = 1;
		break;
	}
	
	R7_Generate(ADF_Regs.R7_ReadbackSetup.bit1 , regData);
	Set_Register(regData);
	
	Sl_En_WR(1); //While Sending SData SLE must be 0
	
	for (i = 0 ; i<16 ; i++)
	{
		S_clk_WR(1);
		_delay_us(1);
		regData += S_Read(); //Read MSB first
		regData <<= 1;
		S_clk_WR(0); // SData Must be Valid at Clock Rising Edge
		_delay_us(1);
	}
	
	Sl_En_WR(1); // After each 32 bits SLE must become 1
	_delay_us(1);
	Sl_En_WR(0); // For Protecting from Unwanted Data Programming SLE = 0
	
	return regData;
}
//////////////////////////////////////////////////////////////////////////
void ADF7020::setMode(TranceiverMode mode)
{
	this->mode = mode;
	if (isReceiver())
	{
		// External Interrupt(s) initialization
		// INT0: Off
		// INT1: On
		// INT1 Mode: Rising Edge
		// INT2: On
		// INT2 Mode: Rising Edge
		GICR|=0xA0;
		MCUCR=0x0C;
		MCUCSR=0x40;
		GIFR=0xA0;
		//// External Interrupt(s) initialization
		//// INT0: Off
		//// INT1: Off
		//// INT2: On
		//// INT2 Mode: Rising Edge
		//GICR|=0x20;
		//MCUCR=0x00;
		//MCUCSR=0x40;
		//GIFR=0x20;
		//serial.init(19200);
////
	}
	else
	{
		// External Interrupt(s) initialization
		// INT0: Off
		// INT1: Off
		// INT2: Off
		GICR &=0x0F;
		GIFR=0x00;
		MCUCR=0x00;
		MCUCSR=0x00;
		//serial.init(19200);

	}
	
	// Timer(s)/Counter(s) Interrupt(s) initialization
	//if (isReceiver())
		//TIMSK=0x00;
	//else
		TIMSK=0x10;

	// Analog Comparator initialization
	// Analog Comparator: Off
	// Analog Comparator Input Capture by Timer/Counter 1: Off
	//if (isReceiver())
	//
	//{
		//ACSR=0x00;
		//SFIOR=0x00;
	//}
	//else
	//{
		//ACSR=0x80;
		//SFIOR=0x00;
	//}
	
		
	adf7020_Initialize();
	
	
}

///////////////////////////////////////////////////
void ADF7020::receiveInt1ISR()
{
	if (mode == TRANSCEIVER_MODE_RECEIVER)
		{
			if ( receivePreambleDetect )
			{
				receiveBit();
			} 
			else
			 return ;
		}
	else
		return;
	
}
//////////////////////////////////////////////////////////////////////////
void ADF7020::receiveInt2ISR()
{
	if (mode == TRANSCEIVER_MODE_RECEIVER)
	{
		receivePreambleDetect = true;
		//ToggleLED(1);
	}
	else
		return ;
}

//////////////////////////////////////////////////////////////////////////
void ADF7020::transmitTimerISR()
{
	//Data_Clk_WR(!Data_Clk_RD());
	//if (mode == TRANSCEIVER_MODE_RECEIVER)
	//{
		//return ;
	//} 
	//else
	{
		
		Data_IO_WR (oldTransmitBit);
		 // Data_IO_WR (1);

		//TCNT1H=0x00;  
		//TCNT1L=66;		//for baudrate  9600/7 Kbps
		//timerCounter ++;		//timerCounter>6
		//if (timerCounter > 6) 
		{
		//	timerCounter =0;
		if ( sendBitIndex == 8 )
			oldTransmitBit = 1 - oldTransmitBit ;
		else
			oldTransmitBit = ((sendBufferShadow >> (7-sendBitIndex) ) & 1 );
		
		sendBitIndex ++;
		
			if ( sendBitIndex == 9 )
			{
				sendBufferShadow = sendBuffer;
				sendBuffer =0 ;
				sendBitIndex = 0;
				transmitBufferEmpty =true;
				
			}
		}
		
	}
	
}

//////////////////////////////////////////////////////////////////////////
void ADF7020::sendByte(char data)
{
		sendBuffer = data;
		transmitBufferEmpty= false;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
bool ADF7020::isTransmitterReady()
{
	return transmitBufferEmpty;
}

//////////////////////////////////////////////////////////////////////////
void ADF7020::receiveBit()
{
	if( receiveBitIndex < 8)
	{
		receivedByteShadow <<= 1;
		if (Data_IO_RD())
			receivedByteShadow |= 1;
	}
	
	//ToggleLED(2);
	//if (receivedByteShadow == 0xFF )
	//{
		//setLED(2);
	//}
	
		//receivedByteShadow |= (1 << (7-receiveBitIndex) );
	receiveBitIndex ++;
	if (receiveBitIndex == 9)
	{
		receiveBitIndex = 0;
		receiveDataFlag = true;
		receivedByte = receivedByteShadow;
		receivedByteShadow = 0;
	}
}
//////////////////////////////////////////////////////////////////////////
char ADF7020::receiveByte()
{
	receiveDataFlag = false;
	return receivedByte;
}

//////////////////////////////////////////////////////////////////////////
bool ADF7020::isNewDataAvailable()
{
	//receiveDataFlag=1;
	return (receiveDataFlag && receivePreambleDetect ) ;
}
//////////////////////////////////////////////////////////////////////////
bool ADF7020::isReceiver()
{
	if (mode == TRANSCEIVER_MODE_RECEIVER)
	{
		return true;
	}
	else
		return false;
}
//////////////////////////////////////////////////////////////////////////
void ADF7020::clearPreaambleDetect()
{
	char tempInt;
	//tempInt = resetAndStoreIntrruptFlag();
	receivePreambleDetect = false;
	
	//restoreIntrrupt(tempInt);
}

void ADF7020::adf7020_ChangeMode( TranceiverMode mode )
{
	
	long int data_reg;
	
	this->mode = mode;
	if (isReceiver())
	{
		// External Interrupt(s) initialization
		// INT0: Off
		// INT1: On
		// INT1 Mode: Rising Edge
		// INT2: On
		// INT2 Mode: Rising Edge
		GICR|=0xA0;
		MCUCR=0x0C;
		MCUCSR=0x40;
		GIFR=0xA0;
		//// External Interrupt(s) initialization
		//// INT0: Off
		//// INT1: Off
		//// INT2: On
		//// INT2 Mode: Rising Edge
		//GICR|=0x20;
		//MCUCR=0x00;
		//MCUCSR=0x40;
		//GIFR=0x20;
		//serial.init(19200);
		////
	}
	else
	{
		// External Interrupt(s) initialization
		// INT0: Off
		// INT1: Off
		// INT2: Off
		GICR &=0x0F;
		GIFR=0x00;
		MCUCR=0x00;
		MCUCSR=0x00;
		//serial.init(19200);

	}
	
	// Timer(s)/Counter(s) Interrupt(s) initialization
	//if (isReceiver())
	//TIMSK=0x00;
	//else
	TIMSK=0x10;

	
	
	
	CE_WR(0);
	_delay_us(1);
	CE_WR(1);
	//delay_ms(100);
	//R0 Setting
	if (mode == TRANSCEIVER_MODE_RECEIVER)
	{
		//set data and clock pins as input.
		DDRD=0x82; //0x3A
		PA_ENABLE_WR(0);
		_delay_us(20);
		
		//RF3023 Setting: RX: V1=1,V2=0 ---- Tx V1=0,V2=1;
		TR_SWV1_WR(1);
		TR_SWV2_WR(0);
		
		//HMC646 Setting: 1 = Tx Enable
		TR_SW_WR(0);
		

		ADF_Regs.R0_N.bit1.Fractional_N_DivideRatio =15845;//31130;//4096;
		ADF_Regs.R0_N.bit1.Integer_N_DivideRatio = 43;//106;
		ADF_Regs.R0_N.bit1.Tx_Rx_bit = 1;
	}
	else
	{
		//set data and clock port as output.
		DDRD=0xBA;
		//RF3023 Setting: RX: V1=1,V2=0 ---- Tx V1=0,V2=1;
		TR_SWV1_WR(0);
		TR_SWV2_WR(1);
		
		//HMC646 Setting: 1 = Tx Enable
		TR_SW_WR(1);
		_delay_us(1);
		PA_ENABLE_WR(1);
		
		ADF_Regs.R0_N.bit1.Fractional_N_DivideRatio =16500;//16500;//4096;//22938;
		ADF_Regs.R0_N.bit1.Integer_N_DivideRatio =43;//107//22 in rx
		ADF_Regs.R0_N.bit1.Tx_Rx_bit = 0;
	}
	ADF_Regs.R0_N.bit1.PLL_enablebit =1;
	ADF_Regs.R0_N.bit1.MUXOUT =3;
	
	R0_Generate(ADF_Regs.R0_N.bit1,data_reg);
	Set_Register(data_reg);
}


//////////////////////////////////////////////////////////////////////////
bool ADF7020::isPreambleDetected()
{
	return receivePreambleDetect;
}

//////////////////////////////////////////////////////////////////////////
void ADF7020::changeRfPower(RFPowerState state)
{
	unsigned long data_reg;
	if (state != rfPowerState)
	{
		ADF_Regs.R2_TransmitModulation_FSK.bit1.PA_enablebit = 1;
		ADF_Regs.R2_TransmitModulation_FSK.bit1.MutePA_until_Lock = 0;
		ADF_Regs.R2_TransmitModulation_FSK.bit1.Modulation_Scheme = 0;
		switch (state)
		{
			case RF_POWER_STATE_HIGH:
				ADF_Regs.R2_TransmitModulation_FSK.bit1.PA_OutLevel = 63;
				rfPowerState = RF_POWER_STATE_HIGH;
			break;
			case RF_POWER_STATE_MEDIUM:
				ADF_Regs.R2_TransmitModulation_FSK.bit1.PA_OutLevel = 14;
				rfPowerState = RF_POWER_STATE_MEDIUM;
			break;
			case RF_POWER_STATE_LOW:
				ADF_Regs.R2_TransmitModulation_FSK.bit1.PA_OutLevel = 3;
				rfPowerState = RF_POWER_STATE_LOW;
			break;
		}
		
		ADF_Regs.R2_TransmitModulation_FSK.bit1.fDeviation =33; //20 K deviation
		ADF_Regs.R2_TransmitModulation_FSK.bit1.NotUsed = 0;
		ADF_Regs.R2_TransmitModulation_FSK.bit1.TxDataInvertbit = 0;//0;
		ADF_Regs.R2_TransmitModulation_FSK.bit1.PA_Bias = 2;
		R2_Generate_FSK(ADF_Regs.R2_TransmitModulation_FSK.bit1, data_reg);
		Set_Register(data_reg);
	}
	
}

void ADF7020::calculateRSSI(uint16_t rssi)
{
	switch (rssi & 0x780)
	{
		case 0x700:
			if ((rssi & 0x7F) < 20 )								//power <-110 dbm
				changeRfPower(RF_POWER_STATE_HIGH); 
			else if ((rssi & 0x7F) > 20  && (rssi & 0x7F) < 60)		 // -110 <RFpower < -90
				changeRfPower(RF_POWER_STATE_MEDIUM);
			else
				changeRfPower(RF_POWER_STATE_LOW);
		break;
		case 0x500:
			if ((rssi & 0x7F) < 5 )								//power <-110 dbm
				changeRfPower(RF_POWER_STATE_HIGH);
			else if ((rssi & 0x7F) > 5  && (rssi & 0x7F) < 20 )		 // -110 <RFpower < -90
				changeRfPower(RF_POWER_STATE_MEDIUM);
			else
				changeRfPower(RF_POWER_STATE_LOW);
		break;
		case 0x480:
			if ((rssi & 0x7F) < 20 )								//-98 dbm <power <-88 dbm
				changeRfPower(RF_POWER_STATE_MEDIUM);
			else 		 //	RFpower >-88
				changeRfPower(RF_POWER_STATE_LOW);
		break;
		case 0x400:
			if ((rssi & 0x7F) < 20 )								// -90 <power <-80 dbm
				changeRfPower(RF_POWER_STATE_MEDIUM);
			else
				changeRfPower(RF_POWER_STATE_LOW);
		break;
		case 0x200:
			changeRfPower(RF_POWER_STATE_LOW);						// -75 <power
		break;
		case 0:														
			changeRfPower(RF_POWER_STATE_LOW);					   //  -70 dbm  < power
		break;
		default:
			changeRfPower(RF_POWER_STATE_HIGH);
		break;
	}
}