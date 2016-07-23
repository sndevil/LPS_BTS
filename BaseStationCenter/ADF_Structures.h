//###########################################################################
//
// FILE:   DSP281x_Adc.h
//
// TITLE:  DSP281x Device ADC Register Definitions.
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  1.00| 11 Sep 2003 | L.H. | Changes since previous version (v.58 Alpha)
//      |             |      | Separated the MAX_CONV bit111 fields into two
//      |             |      | MAX_CONV1  // 3:0
//      |             |      | MAX_CONV2  // 6:4
//      |             |      | Added SEQ_OVRD bit1 to ADCTRL1 for RevC and after silicon
//###########################################################################

#ifndef ADF_REGS_H

#define ADF_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

struct R0_NRegister_Reg
{
       // 0-->Regulator Ready, 1-->R Divider Output, 2-->N Divider Output, 3-->Digital Lock Detect
       // 4-->Analog Lock Detect, 5-->Three_State, 6-->PLL Test Modes, 7-->Fractional Test Modes
       unsigned int Add;
       unsigned int Fractional_N_DivideRatio; // 15bit111S FOR FRACTIONAL DEVIDE RATIO,FROM 0 TO 32767
       unsigned int Integer_N_DivideRatio; // Integer_N_DivideRatio = 31... 255
           unsigned int Tx_Rx_bit;  //0 --> TRANSMIT  1 --> RECEIVE
           unsigned int PLL_enablebit;    // PLL_enable bit SHOULD BE SET TO 1 FOR TURNING_ON PLL
       unsigned int MUXOUT;        
};
            
struct R1_OSC_FilterRegister_Reg
{
       unsigned int Add;
       unsigned int Rcounter_DivideRatio; // RCOUNTER DIVIDES THE REFRENCE FREQ. BY AN INTEGER RANGING FROM 1 TO 7 ,FOR PFD
       unsigned int XTAL_Doublerbit;
           unsigned int CLKOUT_DivideRatio; // AN EVEN DIVIDE RATIO FROM 2 TO 30 IS AVAILABLE: o-->off 1-->2 ... 15-->30
           unsigned int XOSC_enablebit; // XTAL OSC ENABLE bit111 MUST SET TO 1 TO ENABLE THE XTAL OSC;
           unsigned int VCO_Band; // 0-->862to956MHz 1-->431to478MHz
           unsigned int CP_Current; // Charge Pump Current(mA): 0-->0.3 1-->0.9 2-->1.5 3-->2.1
           unsigned int VCO_Bias; // 0-->125uA 1-->375uA 2-->625uA .... 15-->3.875mA
           unsigned int VCO_Adjust; // frequency of operation(MHz): 0-->850to920 1-->860to930 2-->870to940 3-->880to950
           unsigned int IF_FilterBW; // 0-->100KHz 1-->150KHz 2-->200KHz 3-->not used         
           unsigned int rsvd;
};

// Register 2 can be programmed in three different modes,
// GFSK , ASK , FSk . . . .

struct R2_TransmitModulationRegister_ASK_OOK_Reg
{
           unsigned int Add;
           unsigned int PA_enablebit;
           unsigned int MutePA_until_Lock; // IF THIS bit111 IS HIGH MUTE PA IS ON UNTIL LOCK DETECT HIGH
           unsigned int Modulation_Scheme; // 0-->FSK 1-->GFSK 2-->ASK 3-->OOK 7-->GOOK
           unsigned int PA_OutLevel_high; // 0-->-16dBm 1-->-16+.45dBm 2-->-16+ .... 63-->13dBm
       unsigned int PA_OutLevel_low; // 0-->-16dBm 1-->-16+.45dBm 2-->-16+ .... 63-->13dBm
           unsigned int NotUsed; // Not Used
       unsigned int TxDataInvertbit;
       unsigned int PA_Bias; // 0-->5uA 1-->7uA 2-->9uA 3-->11uA
};

struct R2_TransmitModulationRegister_FSK_Reg
{
           unsigned int PA_enablebit;
           unsigned int MutePA_until_Lock; // IF THIS bit111 IS HIGH MUTE PA IS ON UNTIL LOCK DETECT HIGH
           unsigned int Modulation_Scheme; // 0-->FSK 1-->GFSK 2-->ASK 3-->OOK 7-->GOOK
           unsigned int PA_OutLevel; // 0-->-16dBm 1-->-16+.45dBm 2-->-16+ .... 63-->13dBm
       unsigned int fDeviation; // 0-->-16dBm 1-->-16+.45dBm 2-->-16+ .... 63-->13dBm
           unsigned int NotUsed; // Not Used
       unsigned int TxDataInvertbit;
       unsigned int PA_Bias; // 0-->5uA 1-->7uA 2-->9uA 3-->11uA
};


struct R2_TransmitModulationRegister_GFSK_GOOK_Reg
{
           unsigned int Add;
           unsigned int PA_enablebit;
           unsigned int MutePA_until_Lock; // IF THIS bit111 IS HIGH MUTE PA IS ON UNTIL LOCK DETECT HIGH
           unsigned int Modulation_Scheme; // 0-->FSK 1-->GFSK 2-->ASK 3-->OOK 7-->GOOK
           unsigned int PA_OutLevel; // 0-->-16dBm 1-->-16+.45dBm 2-->-16+ .... 63-->13dBm
           unsigned int Divider_Factor; // USED FOR GFSK&GOOK MODES: CHANGES BETWEEN 1 TO 127
           unsigned int GOOK_Mode; // 0-->NormalMode 1-->OutputBufferOn 2-->BleedCurrentOn 3-->BleedBufferOn        
           unsigned int GFSK_Mode_Control; // JUST IN GFSK&GOOK MODES IS USED & VARIES FROM 0 TO 7
           unsigned int IndexCounter; // JUST IN GFSK&GOOK MODES IS USED: 0-->16 1-->32 2-->64 3-->128
       unsigned int TxDataInvertbit;
       unsigned int PA_Bias; // 0-->5uA 1-->7uA 2-->9uA 3-->11uA
};


struct R3_ReceiverCLKRegister_Reg
{
           unsigned int Add:4;
       // Address=3;
       unsigned int BBOS_CLK_Divide; // BASEBAND OFFSET CLOCK FREQUENCY DIVIDE RATIO: 00-->4 01-->8 1X-->16
       unsigned int DEMOD_CLK_Divide; // DEMODULATOR CLOCK DIVIDE RATIO: O-->4 1-->1 2-->2 3-->3
           unsigned int CDR_CLK_Divide; // DATA/CLOCK RECOVERY FREQUENCY DIVIDE RATIO RANGING IS FROM 1 TO 255
       unsigned int SEQ_CLK_Divide; // SEQUENCER CLOCK DIVIDES THE CLOCK BY AN INTEGER RANGING FROM 1 TO 255 ,FOR DIGITAL RECEIVE BLOCK
           unsigned int rsvd;
};

struct R4_Demod_SetupRegister_Reg
{
           unsigned int Add:4;
       // address=4
       unsigned int Demod_Select; // 0-->LINEAR DEMODULATOR   1-->CORRELATOR/DEMODULATOR   2-->ASK/OOK   3-->INVALIED 
       unsigned int PostDemod_BW; // PostDemod_BW=(2^11)*(pi)*(fCUTOFF)/DEMOD_CLK //  typically fCUTOFF=.75data rate
       unsigned int Demod_Lock_Timeout; // Time out delay to lock Threshold=(Demod_Lock_Timeout)/(SEQ_CLK)
       unsigned int Sync_Mode;
       unsigned int rsvd; 
       // 0-->SERIAL PORT CONTROL_free running   1-->SERIAL PORT CONTROL_lock threshold
       // 2-->SYNC WORD DETECT_free running      3-->SYNC WORD DETECT_lock threshold
       // 4,5-->INTERRUPT/LOCK PIN LOCKS THERSHOLD  6,7-->DEMOD LOCKED AFTER Demod_Lock_Timeout bit111S 
};

struct R5_Sync_WordRegister_Reg
{
           unsigned int Add:4;
       //address=5
       unsigned int Sync_Length; // SYNC WORD LENGTH: 0-->12bit111s 1-->16bit111 2-->20bit111s 3-->24bit111s
       unsigned int Match_Tolerance; // MATCHING TOLERANCE IS FROM 0 TO 3 bit111S ERRORS
       unsigned long Sync_Word_Byte;
};    
 
struct R6_Correlator_DemodRegiter_Reg
{ 
           unsigned int Add:4;
       // Address=6;
       unsigned int Discrimin_BW; // Discrimin_BW=(DEMOD_CLK)*(K)/(800*10^3) 
       unsigned int Dot_Product; // 0-->CROSS PRODUCT 1-->DOT PRODUCT
       unsigned int LNA_Mode; // 0-->DEFAULT 1-->REDUCED GAIN
       unsigned int LNA_Current; // 0-->LNA BIAS CURRENT IS 800uA (BY DEFAULT)
       unsigned int Mixer_Linearity; // 0-->DeafultLinearity 1-->HighLinearity
       unsigned int IF_FilterCal; // TO CALIBRATE IF FILTER THIS bit111 SET TO 1
       unsigned int IF_FilterDivider; // THIS FILTER CLOCK DIVIDE RATIO CHANGES BETWEEN 1 TO 511
       unsigned int RxData_InvertBit; // THIS bit111 SHOULD SET TO 1 TO INVERT RXDATA
       unsigned int Demod_ResetBit; // 0-->NORMAL OPERATION 1-->DEMOD RESET
       unsigned int Cdr_ResetBit; // 0-->NORMAL OPERATION 1-->CDR RESET
};
 
struct R7_ReadbackSetupRegsiter_Reg
{
           unsigned int Add;
       // Address=7;
       unsigned int ADC_Mode; // 0-->MeasureRSSI 1-->BatteryVoltage 2-->Temp.Sensor 3-->ToExternalPin
       unsigned int Readback_Mode; // 0-->AFCWord 1-->ADCOutput 2-->FilterCalibration 3-->SiliconRevision
       unsigned int Readback_enablebit;
};
  
struct R8_PowerDownRegister_Reg
{
           unsigned int Add;
       // Address=8;
       unsigned int VCO_Synth_enablebits; 
       /*IF PLL_ENABLE_bit111 FROM REGISTER0 IS SET TO 0: 0-->PLL&VCO OFF  1-->PLL ON  2-->VCO ON  3-->PLL&VCO ON
         OTHERWISE VCO&PLL ARE ON
       */
       unsigned int LNA_Mixer_enablebit;
       unsigned int Filter_enablebit; 
       unsigned int ADC_enablebit;
       unsigned int Demod_Enablebit; // O-->DEMODULATOR ON     1-->DEMODULATOR OFF 
       unsigned int RSSI_Mode; // 0,2-->RSSI ON    1,3-->RSSI OFF
       unsigned int Tx_Rx_Switch; // 0-->ON(DEFAULT) 1-->OFF
       unsigned int PA_RxMode; // 0-->PA OFF     1-->PA ON
};
 
struct R9_AGCRegister_Reg
{
           unsigned int Add;
       // Address=9;
       unsigned int AGC_LowThreshold; // varies from 1 to 80
       unsigned int AGC_HighThreshold; // varies from 1 to 80
       unsigned int AGC_Search; // 0-->Auto AGC  1-->Hold Setting 
       unsigned int Gain_Control; // 0-->AUTO 1-->USER
       unsigned int LNA_Gain; // 0-->LessThan1 1-->3 2-->10 3-->30
       unsigned int Filter_Gain; // 0-->8 1-->24 2-->72 3-->invalid
       unsigned int Filter_Current; // 0-->LOW  1-->HIGH
};

struct R10_AGC2Register_Reg
{
           unsigned int Add:4;
       // Address=10;
       unsigned int Peak_Response; // DEFAULT=0X2
       unsigned int Leak_Factor; // DEFAULT=0XA
       unsigned int AGC_delay; // DEFAULT=0XA
       unsigned int IQ_Gain_Att_Adjust;
       unsigned int Gain_Attenuate; // 0-->GAIN IS SELECTED
                            // 1-->ATTENUATE IS SELECTED
       unsigned int IQ_GainSelect; // 0-->GAIN TO I CHANNEL
                            // 1-->GAIN TO Q CHANNEL
       unsigned int Reserved;
       unsigned int IQ_PhaseAdjust;
       unsigned int IQ_PhaseSelect; // 0-->PHASE TO I CHANNEL
                             // 1-->PHASE TO Q CHANNEL
};

struct R11_AFCRegister_Reg
{
           unsigned int Add:4;
       // Address=11;
       unsigned int AFC_ScaleCoefficient; // AFC_ScaleCoefficient=Round(500*(2^11)/XTAL)
       unsigned int AFC_enableBit;
};

struct R12_TestRegister_Reg // THIS REGISTER DOSE NOT TO BE WRITTEN IN NORMAL OPERATION,THE DEFAULT TEST MODE IS 0x0000000C FOR NORMAL OPERATION
{
           unsigned int Add:4;
       // Address=12;
       unsigned int PLL_TestModes;
       unsigned int Frac_TestModes;
       unsigned int Counter_Reset;
       unsigned int Digit_TestModes; 
       // 4bit111s LENGTH, 7-->ENABLES TEST DAC,WITH NO OFFSET REMOVAL, 
       // 10-->ENABLES TEST DAC,WITH OFFSET REMOVAL(NEEDED FOR LINEAR MODULATION ONLY)
       unsigned int Manual_FilterCal; // DEFAULT=32 , 6bit111s LENGTH
       unsigned int Cal_Source; // 0-->INTERNAL   1-->SERIAL IF BW CAL
       unsigned int OSC_Test;
       unsigned int ForceLD_High;
       unsigned int Analog_Test_Mux; // 4bit111s LENGTH
       unsigned int Prescaler; // 0-->4/5  1-->8/9
};

struct R13_Offset_GainRegister_Reg
{
           unsigned int Add;
       // Address_bit111s=13;
       unsigned int KP; // KP(default)=2
       unsigned int KI; // KI(default)=3
       unsigned int Pulse_Extention; // 0-->NORMAL PULSE WIDTH, 1-->2*PULSE WIDTH .... 15-->16*PULSE WIDTH
       unsigned int TestDAC_OffsetRemoval;   // DAC_input=(2^TestDAC_Gain)*(TestDAC_OffsetRemoval/4096)
       unsigned int TestDAC_Gain;
 };   

union R0_NRegister
{
//        Uint32 all;
        struct R0_NRegister_Reg bit1;
};

union R1_OSC_FilterRegister
{
        struct R1_OSC_FilterRegister_Reg bit1;
};

union R2_TransmitModulationRegister_ASK_OOK
{
        struct R2_TransmitModulationRegister_ASK_OOK_Reg bit1;
};

union R2_TransmitModulationRegister_FSK
{
        struct R2_TransmitModulationRegister_FSK_Reg bit1;
};

union R2_TransmitModulationRegister_GFSK_GOOK
{
        struct R2_TransmitModulationRegister_GFSK_GOOK_Reg bit1;
};

union R3_ReceiverCLKRegister
{
        struct R3_ReceiverCLKRegister_Reg bit1;
};

union R4_Demod_SetupRegister
{
        struct R4_Demod_SetupRegister_Reg bit1;
};

union R5_Sync_WordRegister
{
        struct R5_Sync_WordRegister_Reg bit1;
};

union R6_Correlator_DemodRegiter
{
        struct R6_Correlator_DemodRegiter_Reg bit1;
};

union R7_ReadbackSetupRegsiter
{
        struct R7_ReadbackSetupRegsiter_Reg bit1;
};

union R8_PowerDownRegister
{
        struct R8_PowerDownRegister_Reg bit1;
};

union R9_AGCRegister
{
        struct R9_AGCRegister_Reg bit1;
};

union R10_AGC2Register
{
        struct R10_AGC2Register_Reg bit1;
};

union R11_AFCRegister
{
        struct R11_AFCRegister_Reg bit1;
};

union R12_TestRegister
{
        struct R12_TestRegister_Reg bit1;
};

union R13_Offset_GainRegister
{
        struct R13_Offset_GainRegister_Reg bit1;
};

struct ADF_REGS
{
        union R0_NRegister R0_N;
        union R1_OSC_FilterRegister R1_OSC_Filter;
        union R2_TransmitModulationRegister_ASK_OOK R2_TransmitModulation_ASK_OOK;
        union R2_TransmitModulationRegister_FSK R2_TransmitModulation_FSK;
        union R2_TransmitModulationRegister_GFSK_GOOK R2_TransmitModulation_GFSK_GOOK;
        union R3_ReceiverCLKRegister R3_ReceiverCLK;
        union R4_Demod_SetupRegister R4_Demod_Setup;
        union R5_Sync_WordRegister R5_Sync_Word;
        union R6_Correlator_DemodRegiter R6_Correlator_Demod;
        union R7_ReadbackSetupRegsiter R7_ReadbackSetup;
        union R8_PowerDownRegister R8_PowerDown;
        union R9_AGCRegister R9_AGC;
        union R10_AGC2Register R10_AGC2;
        union R11_AFCRegister R11_AFC;
        union R12_TestRegister R12_Test;
        union R13_Offset_GainRegister R13_Offset_Gain;
};
struct ADF_REGS ADF_Regs;
// ---------------------------------------------------------------

#define R0_Generate(object,out_32bit) \
    out_32bit = 0x00000000;\
    out_32bit |= (object.MUXOUT & 0x7);\
    out_32bit <<=1;\
    out_32bit |= (object.PLL_enablebit & 0x1);\
    out_32bit <<=1;\
    out_32bit |= (object.Tx_Rx_bit & 0x1);\
    out_32bit <<=8;\
    out_32bit |= (object.Integer_N_DivideRatio & 0xFF);\
    out_32bit <<=15;\
    out_32bit |= (object.Fractional_N_DivideRatio & 0x7FFF);\
    out_32bit <<=4;\
    out_32bit += 0;
// ---------------------------------------------------------------
#define R1_Generate(object,out_32bit) \
    out_32bit = 0x00000000;\
    out_32bit |= (object.IF_FilterBW & 0x3);\
    out_32bit <<=2;\
    out_32bit |= (object.VCO_Adjust & 0x3);\
    out_32bit <<=4;\
    out_32bit |= (object.VCO_Bias & 0xF);\
    out_32bit <<=2;\
    out_32bit |= (object.CP_Current & 0x3);\
    out_32bit <<=1;\
    out_32bit |= (object.VCO_Band & 0x1) ;\
    out_32bit <<=1;\
    out_32bit |= (object.XOSC_enablebit & 0x1) ;\
    out_32bit <<=4;\
    out_32bit |= (object.CLKOUT_DivideRatio & 0xF) ;\
    out_32bit <<=1;\
    out_32bit |= (object.XTAL_Doublerbit & 0x1) ;\
    out_32bit <<=3;\
    out_32bit |= (object.Rcounter_DivideRatio & 0x7) ;\
    out_32bit <<=4;\
    out_32bit += 1;
// ---------------------------------------------------------------
#define R2_Generate_ASK_OOK(object,out_32bit) \
    out_32bit = 0x0;\
    out_32bit |= (object.PA_Bias & 3);\
    out_32bit <<=1;\
    out_32bit |= (object.TxDataInvertbit & 1);\
    out_32bit <<=8;\
    out_32bit |= (object.NotUsed & 0x7);\
    out_32bit <<=6;\
    out_32bit |= (object.PA_OutLevel_low & 0x3F);\
    out_32bit <<=6;\
    out_32bit |= (object.PA_OutLevel_high & 0x3F);\
    out_32bit <<=3;\
    out_32bit |= (object.Modulation_Scheme & 0x3F);\
    out_32bit <<=1;\
    out_32bit |= (object.MutePA_until_Lock & 0x3F);\
    out_32bit <<=1;\
    out_32bit |= (object.PA_enablebit & 0x3F);\
    out_32bit <<=4;\
    out_32bit += 2;

// ---------------------------------------------------------------
#define R2_Generate_FSK(object,out_32bit) \
    out_32bit = 0x0;\
    out_32bit |= (object.PA_Bias & 3);\
    out_32bit <<=1;\
    out_32bit |= (object.TxDataInvertbit & 1);\
    out_32bit <<=5;\
    out_32bit |= (object.NotUsed & 0x7);\
    out_32bit <<=9;\
    out_32bit |= (object.fDeviation & 0x1FF);\
    out_32bit <<=6;\
    out_32bit |= (object.PA_OutLevel & 0x3F);\
    out_32bit <<=3;\
    out_32bit |= (object.Modulation_Scheme & 0x3F);\
    out_32bit <<=1;\
    out_32bit |= (object.MutePA_until_Lock & 0x3F);\
    out_32bit <<=1;\
    out_32bit |= (object.PA_enablebit & 0x3F);\
    out_32bit <<=4;\
    out_32bit += 2;

#define R2_Generate_GFSK_GOOK(object,out_32bit) \
    out_32bit = 0x0;\
    out_32bit |= (object.PA_Bias & 3);\
    out_32bit <<=1;\
    out_32bit |= (object.TxDataInvertBit & 1);\
    out_32bit <<=2;\
    out_32bit |= (object.IndexCounter & 0x7);\
    out_32bit <<=3;\
    out_32bit |= (object.GFSK_Mode_Control & 0x7);\
    out_32bit <<=2;\
    out_32bit |= (object.GOOK_Mode & 0x7);\
    out_32bit <<=7;\
    out_32bit |= (object.Divider_Factor & 0x1FF);\
    out_32bit <<=6;\
    out_32bit |= (object.PA_OutLevel & 0x3F);\
    out_32bit <<=3;\
    out_32bit |= (object.Modulation_Scheme & 0x3F);\
    out_32bit <<=1;\
    out_32bit |= (object.MutePA_until_Lock & 0x3F);\
    out_32bit <<=1;\
    out_32bit |= (object.PA_enableBit & 0x3F);\
    out_32bit <<=4;\
    out_32bit += 2;

#define R3_Generate(object,out_32bit) \
    out_32bit =0x00000000;\
    out_32bit |= (object.SEQ_CLK_Divide & 0xFF);\
    out_32bit <<=8;\
    out_32bit |= (object.CDR_CLK_Divide & 0xFF);\
    out_32bit <<=2;\
    out_32bit |= (object.DEMOD_CLK_Divide & 0x03);\
    out_32bit <<=2;\
    out_32bit |= (object.BBOS_CLK_Divide & 0x03);\
    out_32bit <<=4;\
    out_32bit += 3;
// ---------------------------------------------------------------
#define R4_Generate(object,out_32bit) \
    out_32bit =0x00000000;\
    out_32bit |= (object.Sync_Mode & 0x07);\
    out_32bit <<=8;\
    out_32bit |= (object.Demod_Lock_Timeout & 0xFF );\
    out_32bit <<=10;\
    out_32bit |= (object.PostDemod_BW & 0x03FF);\
    out_32bit <<=2;\
    out_32bit |= (object.Demod_Select & 0x03);\
    out_32bit <<=4;\
    out_32bit += 4;
// ---------------------------------------------------------------
#define R5_Generate(object,out_32bit) \
    out_32bit =0x00000000;\
    out_32bit |= (object.Sync_Word_Byte & 0x00FFFFFF);\
    out_32bit <<=2;\
    out_32bit |= (object.Match_Tolerance & 0x03);\
    out_32bit <<=2;\
    out_32bit |= (object.Sync_Length & 0x03);\
    out_32bit <<=4;\
    out_32bit += 5;
// ---------------------------------------------------------------
#define R6_Generate(object,out_32bit) \
    out_32bit =0x00000000;\
    out_32bit |= (object.Cdr_ResetBit & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.Demod_ResetBit & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.RxData_InvertBit & 0x01);\
    out_32bit <<=9;\
    out_32bit |= (object.IF_FilterDivider & 0x01FF);\
    out_32bit <<=1;\
    out_32bit |= (object.IF_FilterCal & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.Mixer_Linearity & 0x01);\
    out_32bit <<=2;\
    out_32bit |= (object.LNA_Current & 0x03);\
    out_32bit <<=1;\
    out_32bit |= (object.LNA_Mode & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.Dot_Product & 0x01);\
    out_32bit <<=10;\
    out_32bit |= (object.Discrimin_BW & 0x03FF);\
    out_32bit <<=4;\
    out_32bit += 6;
// ---------------------------------------------------------------
#define R7_Generate(object,out_32bit) \
    out_32bit =0x00000000;\
    out_32bit |= (object.Readback_enablebit & 0x01);\
    out_32bit <<=2;\
    out_32bit |= (object.Readback_Mode & 3);\
    out_32bit <<=2;\
    out_32bit |= (object.ADC_Mode & 3);\
    out_32bit <<=4;\
    out_32bit += 7;
// ---------------------------------------------------------------
#define R8_Generate(object,out_32bit) \
    out_32bit =0x0000;\
    out_32bit |= (object.PA_RxMode & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.Tx_Rx_Switch & 0x01);\
    out_32bit <<=2;\
    out_32bit |= (object.RSSI_Mode & 3);\
    out_32bit <<=1;\
    out_32bit |= (object.Demod_Enablebit & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.ADC_enablebit & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.Filter_enablebit & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.LNA_Mixer_enablebit & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.VCO_Synth_enablebits & 0x01);\
    out_32bit <<=4;\
    out_32bit += 8;
// ---------------------------------------------------------------

#define R9_Generate(object,out_32bit) \
    out_32bit =0x00000000;\
    out_32bit |= (object.Filter_Current & 0x01);\
    out_32bit <<=2;\
    out_32bit |= (object.Filter_Gain & 0x3);\
    out_32bit <<=2;\
    out_32bit |= (object.LNA_Gain & 0x3);\
    out_32bit <<=1;\
    out_32bit |= (object.Gain_Control & 0x01);\    
    out_32bit <<=1;\
    out_32bit |= (object.AGC_Search & 0x01);\    
    out_32bit <<=7;\
    out_32bit |= (object.AGC_HighThreshold & 0x7F);\
    out_32bit <<=7;\
    out_32bit |= (object.AGC_LowThreshold & 0x7F);\
    out_32bit <<=4;\
    out_32bit += 9;
// ---------------------------------------------------------------

#define R10_Generate(object,out_32bit) \
    out_32bit =0x00000000;\
    out_32bit |= (object.IQ_PhaseSelect & 0x01);\
    out_32bit <<=4;\
    out_32bit |= (object.IQ_PhaseAdjust & 0xF);\
    out_32bit <<=1;\
    out_32bit |= (object.Reserved & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.IQ_GainSelect & 0x01);\
    out_32bit <<=1;\
    out_32bit |= (object.Gain_Attenuate & 0x01);\
    out_32bit <<=5;\
    out_32bit |= (object.IQ_Gain_Att_Adjust & 0x1F);\
    out_32bit <<=4;\
    out_32bit |= (object.AGC_delay & 0xF);\
    out_32bit <<=4;\
    out_32bit |= (object.Leak_Factor & 0b00001111);\
    out_32bit <<=4;\
    out_32bit |= (object.Peak_Response & 0xF);\
    out_32bit <<=4;\
    out_32bit += 10;
// ---------------------------------------------------------------
#define R11_Generate(object,out_32bit) \
    out_32bit =0x00000000;\
    out_32bit |= (object.AFC_enableBit & 0x01);\
    out_32bit <<=16;\
    out_32bit |= (object.AFC_ScaleCoefficient & 0xFFFF);\
    out_32bit <<=4;\
    out_32bit += 11;
// ---------------------------------------------------------------

//-----------------------------------------------------------
#ifdef __cplusplus
}
#endif /* extern "C" */


#endif  // end of ADF_REGS_H definition

//===========================================================================
// No more.
//===========================================================================
