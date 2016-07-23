#ifndef definition_H
#define definition_H


//#define    PA_enable_WR(x)		(PORTD = (PORTD & ~_BV(PD1)) | ((x) << PD1))			//PORTD.1
//#define    MaxOut    PORTD.2
#define    MaxOut_RD()			((PINC & _BV(PC0)) >> PC0)     //PINC.0
#define    MaxOut_WR(x)			(PORTC = (PORTC & ~_BV(PC0)) | ((x) << PC0))
#define    Data_Clk_RD()		((PIND & _BV(PD3)) >> PD3)								//PORTD.3
#define    Data_Clk_WR(x)		(PORTD = (PORTD & ~_BV(PD3)) | ((x) << PD3))			//PORTD.3
#define    Data_IO_RD()			((PIND & _BV(PD4)) >> PD4)								//PORTD.4
#define    Data_IO_WR(x)		(PORTD = (PORTD & ~_BV(PD4)) | ((x) << PD4))			//PORTD.4
//#define    INT_Lock   PORTD.5 

#define    TR_SW_WR(x)			(PORTD = (PORTD & ~_BV(PD7)) | ((x) << PD7))			//PORTD.7

#define    REMOTE_ON_OFF(x)		(PORTB = (PORTB & ~_BV(PB4)) | ((x) << PB4))			//PORTB.4


#define    S_clk_WR(x)			(PORTC = (PORTC & ~_BV(PC0)) | ((x) << PC0))			//PORTC.0
#define    S_Read()				((PINC & _BV(PC1)) >> PC1)								//PORTC.1
#define    S_Data_WR(x)			(PORTC = (PORTC & ~_BV(PC2)) | ((x) << PC2))			//PORTC.2
#define    Sl_En_WR(x)			(PORTC = (PORTC & ~_BV(PC3)) | ((x) << PC3))			//PORTC.3
#define    CE_WR(x)				(PORTC = (PORTC & ~_BV(PC4)) | ((x) << PC4))			//PORTC.4
#define    TR_SWV2_WR(x)		(PORTC = (PORTC & ~_BV(PC5)) | ((x) << PC5))			//PORTC.5
#define    TR_SWV1_WR(x)		(PORTC = (PORTC & ~_BV(PC6)) | ((x) << PC6))			//PORTC.6
#define    PA_ENABLE_WR(x)		(PORTB = (PORTC & ~_BV(PC7)) | ((x) << PC7))			//PORTC.4


#define    Led0_RD(x)			((PINA & _BV(PA1)) >> PA1)				//PORTA.1
#define    Led0_WR(x)			(PORTA = (PORTA & ~_BV(PA1)) | ((x) << PA1))			//PORTA.1
#define    Led1_RD(x)			((PINA & _BV(PA2)) >> PA2)				//PORTA.2
#define    Led1_WR(x)			(PORTA = (PORTA & ~_BV(PA2)) | ((x) << PA2))			//PORTA.2
#define    Led2_RD()			((PINA & _BV(PA3)) >> PA3)								//PORTA.3
#define    Led2_WR(x)			(PORTA = (PORTA & ~_BV(PA3)) | ((x) << PA3))			//PORTA.3
#define    Led3_RD(x)			((PINA & _BV(PA4)) >> PA4)				//PORTA.4
#define    Led3_WR(x)			(PORTA = (PORTA & ~_BV(PA4)) | ((x) << PA4))			//PORTA.4
#define    Led4_RD(x)			((PINA & _BV(PA7)) >> PA7)				//PORTA.7
#define    Led4_WR(x)			(PORTA = (PORTA & ~_BV(PA7)) | ((x) << PA7))			//PORTA.7



#define PACKET_LENGTH (DATA_LENGTH  + PACKET_HEADER_NUM + DATA_LENGTH_FIELD + CRC_FIELD_LENGTH )  //53
#define DATA_LENGTH ((sizeof(stationInfo.stationInfoStructure)) + 1 )
#define PREAMBLE_LENGTH 3
#define PACKET_HEADER_NUM 1
#define DATA_LENGTH_FIELD 2
#define CRC_FIELD_LENGTH  2

#define MAX_FIFO_LENGTH 112 //(PACKET_LENGTH+PREAMBLE_LENGTH)*4

//#define CTRL_STATION_NUMBER  1
#define START_PACKET_BYTE  '~'
#define SEND_STATUS  'S'
#define  LTR_CMD   'C'
//#define RESET_LTR  'R'
#define ACK_CODE 10


#define baseStationGetStatusInfoCount (sizeof(stationInfo.stationInfoStructure))

#define  MAXIMUM_NUMBER_OF_DATA (sizeof(stationInfo.stationInfoStructure))

#define BASE_STATION_INFO_MASSAGE_ID 8

#define  START_CMD 									0
#define  SERIAL_SETTING_CMD 						1
#define  REFRESH_RATE_CMD 							2
#define  DEASSIGN_SATELLITES_CMD 					3
#define  SEARCH_TYPE_CMD 							4
#define  SAT_NUMBER_CMD 							5
#define  PDOP_THRESHOLD_CMD 						6
#define  SAT_DISTANCE_ERROR_THRESHOLD_CMD 			7
#define  GPS_DEASSIGN_THRESHOLD_CMD 				8
#define  GLONASS_DEASSIGN_THRESHOLD_CMD 			9
#define  RELIABILITY_DEASSIGN_THRESHOLD_CMD 		10
#define  GPS_USE_THRESHOLD_CMD 						11
#define  GLONASS_USE_THRESHOLD_CMD 					12
#define  TROPOSPHORIC_CORRECTION_CMD 				13
#define  MAX_SPEED_CMD								14
#define  MAX_ACCELERATION_CMD 						15
#define  GREEN_SAT_TYP_CMD 							16
#define  SAVE_SETTING_CMD 							17
#define  READ_SETTING_CMD 							18
#define  MASK_ANGLE_CMD 							19
#define  IONOSPHORIC_CORRECTION_CMD 				20
#define  AUTO_MAX_ANGLE_ATTITUDE_CMD 				21
#define  POSITIONING_TYPE_CMD 						22
#define  READ_LICENCE_CMD 							23
#define  ADD_LICENCE_CMD 							24
#define  REMOVE_LICENCE_CMD 						25
#define  ATTITUDE_INFO_CMD 							26
#define  BASE_STATION_SET_TIME_CMD 					27
#define  BASE_STATION_SET_POS_CMD 					28
#define  BASE_STATION_GET_STATUS_CMD				29
#define  BASE_STATION_RESET_LTR_CMD					30
#define	 AOUTOMATIC_SEARCH_ENABLE_CMD				31
#define  AOUTOMATIC_SEARCH_DISABLE_CMD				32
#define  CHANGE_BASE_STATION_NUMBER_CMD				33
#define  CHANGE_BASE_STATION_POSITIONING_MODE_CMD	34
#define  SET_RANGE_OFFSET_CMD						35

#define  GET_STATUS_BASED_ON_MAC_CMD				36
#define	 SET_LTR_TX_POWER_LEVEL_CMD					37
#define	 TURN_ON_OFF_LTR_CMD						38

#define startCommandCount								1
#define serialSettingsCommandCount						6
#define decimationRateCommandCount						1
#define deassignCommandCount							8
#define searchTypeCommandCount							1
#define baseStationNumberCommandCount					5
#define gdopThresholdCommandCount						2
#define distanceErrorThresholdCommandCount				2
#define deassignThresholdCommandCount					1
#define glonassDeassignThresholdCommandCount			1
#define reliabilityDeassignThresholdCommandCount		1
#define useThresholdCommandCount						1
#define glonassUseThresholdCommandCount					1
#define useTroposphericCorrectionCommandCount			1
#define maxSpeedCommandCount							2
#define maxAccelerationCommandCount						2
#define greenTypeCommandCount							1
#define saveSettingsCommandCount						1
#define readSettingsCommandCount						0
#define maskAngleCommandCount							1
#define useIonosphericCorrectionCommandCount			1
#define automaticmaskAngleAtHighAttitudeCommandCount	1
#define positioningTypeCommandCount						1


#define baseStationSetTimeCommandCount					8
#define baseStationSetPositionCommandCount				13
#define baseStationGetStatusCommandCount				1
#define baseStationResetLTRCommandCount					1
#define aoutomaticSearchCommandCount					2
#define changeBaseStationNumberCommandCount				2
#define changeBseStationPositioningModeCommandCount		2
#define setRangeOffsetCommandCount						4

#define getStatusBasedOnMACCommandCount					1
#define setLTRTxPowerLevelCommandCount					2
#define turnOnOffLTRCommandCount						2

#endif