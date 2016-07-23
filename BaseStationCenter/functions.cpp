/*
* functions.cpp
*
* Created: 7/28/2015 9:20:16 AM
*  Author: CIT_15
*/
#include "functions.h"
#include "definition.h"
#include "GlobalVariables.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

char packetData[PACKET_LENGTH + 1];
uint8_t baseStationNumber EEMEM ;
uint8_t baseStationMACAddress EEMEM;


void micro_Initialize()
{
	PORTA=0x00;
	DDRA=0x0F;

	PORTB=0x10;
	DDRB=0x18;

	PORTC=0x00;
	DDRC=0x1D;

	PORTD=0x00;
	DDRD=0x82;

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0 output: Disconnected
TCCR0=0x00;
TCNT0=0x00;
OCR0=0x00;
	
	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 8000.000 kHz
	// Mode: Normal top=0xFFFF
	// OC1A output: Discon.
	// OC1B output: Discon.
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: On
	// Compare B Match Interrupt: Off
	TCCR1A=0x00;
	TCCR1B=0x01;
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x03;
	OCR1AL=0x41;
	OCR1BH=0x00;
	OCR1BL=0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer 2 Stopped
	// Mode: Normal top=FFh
	// OC2 output: Disconnected
	ASSR=0x00;
	TCCR2=0x00;
	TCNT2=0x00;
	OCR2=0x00;
	
	//if (transceiver1.isReceiver())
	//{
	//// External Interrupt(s) initialization
	//// INT0: Off
	//// INT1: On
	//// INT1 Mode: Rising Edge
	//// INT2: On
	//// INT2 Mode: Rising Edge
	//GICR|=0xA0;
	//MCUCR=0x0C;
	//MCUCSR=0x40;
	//GIFR=0xA0;
	//
	//}
	//else
	//{
	//// External Interrupt(s) initialization
	//// INT0: Off
	//// INT1: Off
	//// INT2: Off
	//MCUCR=0x00;
	//MCUCSR=0x00;
	//
	//}
	//
	//// Timer(s)/Counter(s) Interrupt(s) initialization
	//if (transceiver1.isReceiver())
	//TIMSK=0x00;
	//else
	//TIMSK=0x10;
	//
	//// Analog Comparator initialization
	//// Analog Comparator: Off
	//// Analog Comparator Input Capture by Timer/Counter 1: Off
	//if (transceiver1.isReceiver())
	//
	//{
	//ACSR=0x00;
	//SFIOR=0x00;
	//}
	//else
	//{
	ACSR=0x80;
	SFIOR=0x00;
	//}
	//
	//// Watchdog Timer initialization
	//// Watchdog Timer Prescaler: OSC/2048k
	//#pragma optsize-
	//WDTCR=0x1F;
	//WDTCR=0x0F;
	//#ifdef _OPTIMIZE_SIZE_
	//#pragma optsize+
	//#endif
}

//////////////////////////////////////////////////////////////////////////
void ToggleLED(char led_num)
{
	switch(led_num)
	{
		case 0:
		Led0_WR(!Led0_RD());
		break;
		case 1:
		Led1_WR(!Led1_RD());
		break;
		case 2:
		Led2_WR(!Led2_RD());
		break;
		case 3:
		Led3_WR(!Led3_RD());
		break;
		case 4:
		Led4_WR(!Led4_RD());
		break;
		//default :
		//Led0_WR(!Led0_RD());
		//break;
	}
	
	return ;
}
//////////////////////////////////////////////////////////////////////////
void setLED(char led_num)
{
	switch(led_num)
	{
		case 0:
		Led0_WR(1);
		break;
		case 1:
		Led1_WR(1);
		break;
		case 2:
		Led2_WR(1);
		break;
		case 3:
		Led3_WR(1);
		break;
		case 4:
		Led4_WR(1);
		break;
		default :
		Led0_WR(1);
		break;
	}
	
	return ;
}
//////////////////////////////////////////////////////////////////////////
void resetLED(char led_num)
{
	switch(led_num)
	{
		case 0:
		Led0_WR(0);
		break;
		case 1:
		Led1_WR(0);
		break;
		case 2:
		Led2_WR(0);
		break;
		case 3:
		Led3_WR(0);
		break;
		case 4:
		Led4_WR(0);
		break;
		default :
		Led0_WR(0);
		break;
	}
	
	return ;
}

//////////////////////////////////////////////////////////////////////////





void loadBS()
{
	
	 eeprom_update_byte(&baseStationNumber , 2);
	 eeprom_update_byte(&baseStationMACAddress , 2);
	stationInfo.stationInfoStructure.startByte = '~';
	stationInfo.stationInfoStructure.commandCode =8;
	stationInfo.stationInfoStructure.stationNumber = eeprom_read_byte(&baseStationNumber);
	stationInfo.stationInfoStructure.voltage = 120 ;
	stationInfo.stationInfoStructure.current = 60;
	stationInfo.stationInfoStructure.stability =48;
	stationInfo.stationInfoStructure.temperature = 75;
	stationInfo.stationInfoStructure.humidity = 55;
	stationInfo.stationInfoStructure.battery = 80 ;
	stationInfo.stationInfoStructure.lastLock =240;
	stationInfo.stationInfoStructure.macNumber = eeprom_read_byte(&baseStationMACAddress);
	stationInfo.stationInfoStructure.xCordinate = 154870;
	stationInfo.stationInfoStructure.yCordinate = 1542150;
	stationInfo.stationInfoStructure.zCordinate = 2501540;
	stationInfo.stationInfoStructure.time = 21581010;
	stationInfo.stationInfoStructure.statusFlag = 50;
	stationInfo.stationInfoStructure.assignFlyingObject = 4294949819;
	stationInfo.stationInfoStructure.readyFlyingObject = 0xfffffb01;
	stationInfo.stationInfoStructure.snrValue[0] = 70;
	stationInfo.stationInfoStructure.snrValue[1] = 35;
	stationInfo.stationInfoStructure.snrValue[2] = 35;
	stationInfo.stationInfoStructure.snrValue[3] = 55;
	stationInfo.stationInfoStructure.snrValue[4] = 45;
	stationInfo.stationInfoStructure.snrValue[5] = 25;
	stationInfo.stationInfoStructure.snrValue[14] = 35;
	stationInfo.stationInfoStructure.snrValue[15] = 70;
	stationInfo.stationInfoStructure.LTRHealth = 0 ;
	stationInfo.stationInfoStructure.crcByte = stationInfo.calculateCRC();
}


void receivedDataProcess(char* receivedPacket, int dataLength)
{
	char commandCode=0, LTRcommandLength=0 , intTemp;
	static unsigned char i;
		if(receivedPacket[0] == START_PACKET_BYTE)
		{
			commandCode = receivedPacket[1];
			if (commandCode == GET_STATUS_BASED_ON_MAC_CMD )
			{
				if (receivedPacket[2] == stationInfo.stationInfoStructure.macNumber)
				{
					intTemp = resetAndStoreIntrruptFlag();
					transceiver.changeMode();
					packetProcessor.createPacket(stationInfo.structStartPointer , sizeof(stationInfo.stationInfoStructure) , packetData );
					transceiver.writePacket(packetData);
					GIFR &= 0x1F;
					restoreIntrrupt(intTemp);
				}
				return ;
				
			}
			
			if(receivedPacket[2] == stationInfo.stationInfoStructure.stationNumber)
			{
				switch(commandCode)
				{
					case BASE_STATION_GET_STATUS_CMD:
						intTemp = resetAndStoreIntrruptFlag();
						transceiver.changeMode();
						packetProcessor.createPacket(stationInfo.structStartPointer , sizeof(stationInfo.stationInfoStructure) , packetData );
						transceiver.writePacket(packetData);
						GIFR &= 0x1F;
						restoreIntrrupt(intTemp);
						serialTxDataBuffer.writeString("$CMD",4);
						serialTxDataBuffer.writeString(receivedPacket + 1 , baseStationGetStatusCommandCount + 2);
						break;
					
					case LTR_CMD:
						LTRcommandLength=receivedPacket[3];
						serialTxDataBuffer.writeString(receivedPacket + 4 , LTRcommandLength);
						break;
					case BASE_STATION_RESET_LTR_CMD:
						if (receivedPacket[2] == stationInfo.stationInfoStructure.stationNumber )
						{
							REMOTE_ON_OFF(0);
							_delay_ms(200);
							REMOTE_ON_OFF(1);
							sendAckSignal();
						}	 
					break;
					case SAVE_SETTING_CMD:
						serialTxDataBuffer.writeString("$CMD",4);
						serialTxDataBuffer.writeString(receivedPacket + 1 , saveSettingsCommandCount + 2);
						sendAckSignal();
					break;
					case BASE_STATION_SET_POS_CMD:
						serialTxDataBuffer.writeString("$CMD",4);
						serialTxDataBuffer.writeString(receivedPacket + 1 , baseStationSetPositionCommandCount + 2);
						sendAckSignal();
					break;
					case CHANGE_BASE_STATION_POSITIONING_MODE_CMD:
						serialTxDataBuffer.writeString("$CMD",4);
						serialTxDataBuffer.writeString(receivedPacket + 1 , changeBseStationPositioningModeCommandCount + 2);
						sendAckSignal();
					break;
					case SET_RANGE_OFFSET_CMD:
						serialTxDataBuffer.writeString("$CMD",4);
						serialTxDataBuffer.writeString(receivedPacket + 1 , setRangeOffsetCommandCount + 2);
						sendAckSignal();
					break;
					case BASE_STATION_SET_TIME_CMD:
						serialTxDataBuffer.writeString("$CMD",4);
						serialTxDataBuffer.writeString(receivedPacket + 1 , baseStationSetTimeCommandCount + 2);
						sendAckSignal();
					break;
					case CHANGE_BASE_STATION_NUMBER_CMD:
						serialTxDataBuffer.writeString("$CMD",4);
						serialTxDataBuffer.writeString(receivedPacket + 1 , changeBaseStationNumberCommandCount + 2);
						sendAckSignal();
					break;
					case SET_LTR_TX_POWER_LEVEL_CMD:
						serialTxDataBuffer.writeString("$CMD",4);
						serialTxDataBuffer.writeString(receivedPacket + 1 , setLTRTxPowerLevelCommandCount + 2);
						sendAckSignal();
					break;
					case TURN_ON_OFF_LTR_CMD:
						if (receivedPacket[2] == stationInfo.stationInfoStructure.stationNumber )
						{
							if(receivedPacket[3])
							{
								REMOTE_ON_OFF(1);
							}
							else
							{
								REMOTE_ON_OFF(0);
								stationInfo.stationInfoStructure.LTRHealth = 0;
							}
							sendAckSignal();
						}
					break;					
					default:
					break;
				}
				
			}

		}
	
	return;
}
	


void setGlobalIntrrupt()
{
	intrruptFlag =1;
	sei();
}

void resetGlobalIntrrupt()
{
	intrruptFlag =0;
	cli();
}

char resetAndStoreIntrruptFlag()
{
	char temp=intrruptFlag;
	intrruptFlag = 0 ;
	cli();
	return temp;
}

void restoreIntrrupt(char intFlag)
{
	if (intFlag)
	{
		sei();
		intrruptFlag = 1 ;
	}
	else
	{
		cli();
		intrruptFlag = 0 ;
	}
}


//////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////
void processSerialReceivedBytes()
{
	static unsigned char receivedData[MAXIMUM_NUMBER_OF_DATA+4];
	static int counter=0;
	char temp , *structInfoPointer;
	
	if (serialRxDataBuffer.getFifoFullLength() == 0 )
		return;
	serialRxDataBuffer.readByte(temp);
	
	receivedData[counter] = temp;
	//serial.putChar(counter);

	if (counter == 0 && receivedData[0] != '~')
		counter = -1;
	//if (counter == 1 && receivedData[1] != 8)
	//counter = -1;
	//if (counter == 2 && receivedData[2] != 'M')
	//counter = -1;
	//if (counter == 3 && receivedData[3] != 'D')
	//counter = -1;

	if(counter>0)
	{
		switch(receivedData[1])
		{
			
			/*			case START_CMD:
			if (counter > startCommandCount + 4)
			{
			DEBUGMESSAGE("Start command received\r\n");
			for (int i=0; i<nBASESTATIONS; ++i)
			baseStations[i].deassignFlag = true;


			manageTrackingModules();
			if (receivedData[0] == 'H')
			{
			}
			if (receivedData[0] == 'W')
			{
			}
			if (receivedData[0] == 'C')
			{
			CartesianCoordinate position;
			position.set(0, 0, 0);
			for (int i=0; i<nBASESTATIONS; ++i)
			baseStations[i].initialize(i, position, 0);


			}
			counter = -1;
			}
			break;

			case SERIAL_SETTING_CMD:
			if (counter > serialSettingsCommandCount + 4)
			{
			int serialPort = receivedData[0] - '1';
			int baudRate = receivedData[1];
			baudRate = (baudRate<<8) + receivedData[2];
			baudRate = (baudRate<<8) + receivedData[3];
			serial[serialPort]->setBaudRate(baudRate);
			settings.messageType[serialPort] = receivedData[4];
			settings.messageType[serialPort] <<= 8;
			settings.messageType[serialPort] += receivedData[5];
			DEBUGMESSAGE("Serial settings command received, baudRate: %d, Port: %d, messageType: %d\r\n", baudRate, serialPort, settings.messageType[serialPort]);
			counter = -1;
			}
			break;

			case decimationRateCommand:
			if (counter > decimationRateCommandCount + 4)
			{
			DEBUGMESSAGE("Decimation rate command received\r\n");
			settings.decimationRate = 50 / receivedData[0];
			
			counter = -1;
			}
			break;

			case DEASSIGN_SATELLITES_CMD:
			if (counter > deassignCommandCount + 4)
			{
			DEBUGMESSAGE("Deassign command received\r\n");
			long long int satelliteList;
			satelliteList = receivedData[0];
			satelliteList <<= 8;
			satelliteList |= receivedData[1];
			satelliteList <<= 8;
			satelliteList |= receivedData[2];
			satelliteList <<= 8;
			satelliteList |= receivedData[3];
			DEBUGMESSAGE("GPS satellite to be deassigned: %#lld\r\n", satelliteList);
			for (int i=0; i<nBASESTATIONS; ++i)
			if ((satelliteList & (1<<i)) == (1<<i))
			baseStations[i].deassignFlag = true;

			satelliteList = receivedData[4];
			satelliteList <<= 8;
			satelliteList |= receivedData[5];
			satelliteList <<= 8;
			satelliteList |= receivedData[6];
			satelliteList <<= 8;
			satelliteList |= receivedData[7];
			DEBUGMESSAGE("GLONASS satellite to be deassigned: %lld\r\n", satelliteList);


			manageTrackingModules();
			
			counter = -1;
			}
			break;

			case SEARCH_TYPE_CMD:
			if (counter > searchTypeCommandCount + 4)
			{
			DEBUGMESSAGE("Search type command received: No action\r\n");
			counter = -1;
			}
			break;

			case POSITIONING_TYPE_CMD:
			if (counter > positioningTypeCommandCount + 4)
			{
			DEBUGMESSAGE("Position type command received: No action\r\n");
			counter = -1;
			}
			break;

			case baseStationNumberCommand:
			if (counter > baseStationNumberCommandCount + 4)
			{
			DEBUGMESSAGE("BaseStation number command received\r\n");
			settings.numberOfBaseStationToSearch = receivedData[0];
			int nAll = 0;
			for (int i=0; i<nBASESTATIONS; ++i)
			{
			if (baseStations[i].isAssigned())
			++nAll;
			if (nAll > settings.numberOfBaseStationToSearch)
			baseStations[i].deassignFlag = true;
			}



			manageTrackingModules();
			counter = -1;
			}
			break;

			case PDOP_THRESHOLD_CMD:
			if (counter > gdopThresholdCommandCount + 4)
			{
			settings.pdopThreshold = receivedData[0];
			settings.pdopThreshold = settings.pdopThreshold*256 + receivedData[1];
			DEBUGMESSAGE("PDOP threshold command received: %f\r\n", settings.pdopThreshold);
			counter = -1;
			}
			break;

			case distanceErrorThresholdCommand:
			if (counter > distanceErrorThresholdCommandCount + 4)
			{
			settings.distanceErrorThreshold = receivedData[0];
			settings.distanceErrorThreshold = settings.distanceErrorThreshold*256 + receivedData[1];
			settings.distanceErrorThreshold /= 10.0;
			DEBUGMESSAGE("BaseStation distance error threshold command received: %f\r\n", settings.distanceErrorThreshold);
			counter = -1;
			}
			break;

			case deassignThresholdCommand:
			if (counter > deassignThresholdCommandCount + 4)
			{
			settings.trackingDeassignThreshold = receivedData[0]/4.0;
			DEBUGMESSAGE("GPS deassign threshold command received: %f\r\n", settings.trackingDeassignThreshold);
			counter = -1;
			}
			break;

			case GLONASS_DEASSIGN_THRESHOLD_CMD:
			if (counter > glonassDeassignThresholdCommandCount + 4)
			{
			DEBUGMESSAGE("GLONASS deassign threshold command received: No action\r\n");
			counter = -1;
			}
			break;

			case useThresholdCommand:
			if (counter > useThresholdCommandCount + 4)
			{
			settings.trackingUseThreshold = receivedData[0]/4.0;
			DEBUGMESSAGE("GPS used threshold command received: %f\r\n", settings.trackingUseThreshold);
			counter = -1;
			}
			break;

			case GPS_USE_THRESHOLD_CMD:
			if (counter > glonassUseThresholdCommandCount + 4)
			{
			DEBUGMESSAGE("GLONASS used threshold command received: No action\r\n");
			counter = -1;
			}
			break;

			case RELIABILITY_DEASSIGN_THRESHOLD_CMD:
			if (counter > reliabilityDeassignThresholdCommandCount + 4)
			{
			settings.trackingReliabilityDeassignThreshold = receivedData[0]/100.0;
			DEBUGMESSAGE("Reliability deassign threshold command received: %f\r\n", settings.trackingReliabilityDeassignThreshold);
			counter = -1;
			}
			break;

			case TROPOSPHORIC_CORRECTION_CMD:
			if (counter > useTroposphericCorrectionCommandCount + 4)
			{
			settings.useTroposphericCorrection = receivedData[0];
			DEBUGMESSAGE("Tropospheric correction command received: %d\r\n", settings.useTroposphericCorrection);
			counter = -1;
			}
			break;

			case IONOSPHORIC_CORRECTION_CMD:
			if (counter > useIonosphericCorrectionCommandCount + 4)
			{
			settings.useIonoposphericCorrection = receivedData[0];
			DEBUGMESSAGE("Ionospheric correction command received: %d\r\n", settings.useIonoposphericCorrection);
			counter = -1;
			}
			break;

			case MAX_SPEED_CMD:
			if (counter > maxSpeedCommandCount + 4)
			{
			int speed=0;
			speed = receivedData[0];
			speed <<= 8;
			speed |= receivedData[1];
			settings.maxSpeed = speed;
			DEBUGMESSAGE("maximum speed command received: %f\r\n", settings.maxSpeed);
			counter = -1;
			}
			break;

			case MAX_ACCELERATION_CMD:
			if (counter > maxAccelerationCommandCount + 4)
			{
			int acceleration=0;
			acceleration = receivedData[0];
			acceleration <<= 8;
			acceleration |= receivedData[1];
			settings.maxAcceleration = acceleration/10.0;
			DEBUGMESSAGE("maximum acceleration command received: %f\r\n", settings.maxAcceleration);
			counter = -1;
			}
			break;

			case GREEN_SAT_TYP_CMD:
			if (counter > greenTypeCommandCount + 4)
			{
			settings.greenBaseStationType = receivedData[0] == 0 ? UsedBaseStation : ReadyBaseStation;
			DEBUGMESSAGE("Green satellite type command received: %d\r\n", settings.greenBaseStationType);
			counter = -1;
			}
			break;

			case MASK_ANGLE_CMD:
			if (counter > maskAngleCommandCount + 4)
			{
			settings.elevationMask = (char)receivedData[0];
			DEBUGMESSAGE("Mask angle command received: %f\r\n", settings.elevationMask);
			counter = -1;
			}
			break;

			case AUTO_MAX_ANGLE_ATTITUDE_CMD:
			if (counter > automaticmaskAngleAtHighAttitudeCommandCount + 4)
			{
			int automaticMaskAngle = receivedData[0];
			DEBUGMESSAGE("Automatic mask angle command received: %d\r\n", automaticMaskAngle);
			counter = -1;
			}
			break;

			case SAVE_SETTING_CMD:
			if (counter > saveSettingsCommandCount + 4)
			{
			saveSettingFlag = 1;
			DEBUGMESSAGE("Save settings command received: %d\r\n", saveSettingFlag);
			counter = -1;
			}
			break;

			case READ_SETTING_CMD:
			if (counter > readSettingsCommandCount + 4)
			{
			//TODO: implement this function
			int readSettingFlag = 1;
			DEBUGMESSAGE("Read settings command received: %d\r\n", readSettingFlag);
			counter = -1;
			}
			break;/**/
			case BASE_STATION_INFO_MASSAGE_ID:
				 
				
				if (counter >= baseStationGetStatusInfoCount - 1)
				{
					ToggleLED(1);
					structInfoPointer =  stationInfo.structStartPointer;
					
					if (stationInfo.stationInfoStructure.stationNumber != receivedData[2])
					{
						eeprom_update_byte(&baseStationNumber , receivedData[2]);
					}
					structInfoPointer[2] = receivedData[2];
										
					for (int i=0 ; i < sizeof(stationInfo.stationInfoStructure)-3 ; i++)	 //
					{
						structInfoPointer[i + 3] = 	receivedData[i + 3] ;
					}
					stationInfo.stationInfoStructure.macNumber = eeprom_read_byte(&baseStationMACAddress);
					stationInfo.stationInfoStructure.crcByte = stationInfo.calculateCRC();
					counter = -1 ;
				}
			break;
			default:
				counter = -1;
			break;
			

			}

		}

		if (counter>=MAXIMUM_NUMBER_OF_DATA+2)
			counter = -1;
		counter++;
}

void sendAckSignal()
{
	char intTemp;
	char ackData[13];
	ackData [0] = START_PACKET_BYTE;
	ackData[1] = ACK_CODE;
	ackData[2] = stationInfo.stationInfoStructure.stationNumber;
	ackData[3] = 1;
	ackData[4] = 'A';
	ackData[5] = 'C';
	ackData[6] = 'K';
	ackData[7] = 0;
	ackData[8] = 0;
	ackData[9] = 0;
	ackData[10] = 0;
	ackData[11] = 0;
	ackData[12] = 0;
	
	
	
	intTemp = resetAndStoreIntrruptFlag();
	
	transceiver.changeMode();
	packetProcessor.createPacket(ackData,13 , packetData );
	transceiver.writePacket(packetData);
	GIFR &= 0x1F;
	restoreIntrrupt(intTemp);
}
void sendNackSignal()
{
	char intTemp;
	char ackData[8];
	ackData [0] = START_PACKET_BYTE;
	ackData[1] = ACK_CODE;
	ackData[2] = stationInfo.stationInfoStructure.stationNumber;
	ackData[3] = 0;
	ackData[4] = 'N';
	ackData[5] = 'A';
	ackData[6] = 'C';
	ackData[7] = 'K';
	
	
	intTemp = resetAndStoreIntrruptFlag();
	
	transceiver.changeMode();
	packetProcessor.createPacket(ackData,sizeof(ackData), packetData );
	transceiver.writePacket(packetData);
	GIFR &= 0x1F;
	restoreIntrrupt(intTemp);
}
