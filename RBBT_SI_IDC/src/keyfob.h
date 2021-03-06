#ifndef _RKE_DEMO_H
#define _RKE_DEMO_H
/*------------------------------------------------------------------------------
 *                          Silicon Laboratories, Inc.
 *                           http://www.silabs.com
 *                               Copyright 2010
 *------------------------------------------------------------------------------
 *
 *    FILE:       keyfob.h
 *    TARGET:     Si4010 RevC
 *    TOOLCHAIN:  Keil
 *    DATE:       Nov 30, 2012,
 *    RELEASE:    1.0 (Tamas Nagy), ROM version 02.00
 *
 *------------------------------------------------------------------------------
 *
 *    DESCRIPTION:
 *      Header file for the keyfob_main module.
 *
 *------------------------------------------------------------------------------
 *
 *    INCLUDES:
 */
#include "si4010_types.h"


//define pinselect if frequency will be selected by gpio pins
#define PINSELECT
//#define CRYSTAL			//define this if external crystal is used on GPIO0

#define bButtonMask_c   (bButton1_c | bButton2_c | bButton3_c | bButton4_c | bButton5_c)
//defines buttons transmitting normal packet in FSK
#define bFskButtons_c   (bButton1_c | bButton2_c)
//defines buttons transmitting Holtek code in OOK
#define bHoltekButtons_c        (bButton4_c | bButton5_c)      

#define bEncodeNoneNrz_c       	0   /* No encoding */
#define bEncodeManchester_c    	1   /* Manchester encoding */
#define bModOOK_c				0
#define bModFSK_c				1

#define bHFrameSize_c                   2
#define bPreambleSize_c		   	4	
#define bSyncSize_c                     2       
#define bPayloadSize_c		   	10
#define bCrcSize_c                      2	
#define bFrameSize_c		        (bPreambleSize_c + bSyncSize_c + bPayloadSize_c + bCrcSize_c)
#define bPayloadStartIndex_c            (bPreambleSize_c + bSyncSize_c)
#define bPreambleNrz_c			0xaa
#define bPreambleManch_c		0xff
#define bSync1_c			0x2d
#define bSync2_c			0xd4

#define iLowBatMv_c		2500

//defines GPIO pins used for button input (see button vector mapping in AN370)
#define bRightButton_c          0x01    //GPIO0
#define bMiddleButton_c         0x02    //GPIO1	
#define bLeftButton_c 		0x04    //GPIO2
#define bRearButton_c           0x08    //GPIO3
#define bFrontButton_c          0x10    //GPIO4
#define bButton1_c              0x01    //GPIO0
#define bButton2_c              0x02    //GPIO1
#define bButton3_c              0x04    //GPIO2
#define bButton4_c              0x08    //GPIO3
#define bButton5_c              0x10    //GPIO4
//defines position of button bits in status byte of the transmitted packets
#define M_ButtonBits_c			0x1F    

#define SELOUT                  GPIO6   //pin8 (SOIC)
#define FREQSEL0                GPIO8   //pin14 (SOIC)
#define FREQSEL1                GPIO9   //pin1 (SOIC)
#define DEBUGPIN                GPIO7   //pin7 (SOIC)

#define wSys_16Bit_40ms_c		41737
// Amount of time, the application will wait after boot without
// getting a qualified button push before giving up and shutting the chip down
#define	bMaxWaitForPush_c		50
#define wRepeatInterval_c		40     //ms
#define bRepeatCount_c			4
#define bBatteryWait_c 			100
 // This specifies the number of times that the RTC ISR is called between 
 // each call to the button service routine.  The actual time between
 // calls is dependent on the setting of RTC_CTRL.RTC_DIV which dictates how
 // often the RTC ISR is called (5ms in this demo).It means that the interval
 // between calls to the vBsr_Service() function is 5*2=10ms.
#define	bDebounceInterval_c		2
// Size of FIFO of captured buttons .. max number of unserviced button changes
#define bPtsSize_c                      3  
//---------------------------------------------------------------------------
//    PROTOTYPES OF STATIC FUNCTIONS:
//-------------------------------------------------------------------------
void vPacketAssemble    /* Assemble packet for Rke demo receiver*/
      (
        void
      );
void vButtonCheck       /* Buttons checking */
      (
        void
      );
void vRepeatTxLoop
      (
       void
      );
void vCalculateCrc
	  (
	  void
	  );
void vConvertPacket     /* change bit order and invert if necessary*/
      (
       BYTE bModType
      );
  //---------------------------------------------------------------------------
  //    VARIABLES:

  BYTE bIsr_DebounceCount;
  LWORD xdata lLEDOnTime;
  BYTE xdata bRepeatCount;
  LWORD lTimestamp;
  BYTE bdata bPrevBtn = 0;
  BYTE xdata *pbFrameHead;//Pointer to the head of the frame to be sent out.
  BYTE xdata bFrameSize;
  xdata BYTE abFrameBuffer[bHFrameSize_c];
  xdata LWORD lPartID;
  xdata LWORD lShiftedPartID;
  code BYTE abConvTable[16] = {0x00,0x88,0x44,0xcc,0x22,0xaa,0x66,0xee,0x11,0x99,
  	0x55,0xdd,0x33,0xbb,0x77,0xff};
  int iBatteryMv;

BYTE xdata pbRData[4];
//conversion table 2bit to 6bit for HT6P20
code BYTE ab2to6bitTable[4] = {0x36,0x26,0x34,0x24};

  BYTE bStatus;
 
  BYTE xdata bButtonState;
// return status of the vPacketAssemble() function
 BYTE xdata bValidBtn;

  float xdata fDesiredFreqOOK;
  float xdata fDesiredFreqFSK;
  float xdata fDesiredFreq;
  BYTE  xdata bFskDev;
  BYTE	bPreamble;
  /* Structure for setting up the ODS .. must be in XDATA */
  tOds_Setup xdata rOdsSetup;

  /* Structure for setting up the XO .. must be in XDATA */
  tFCast_XoSetup xdata rXoSetup;

  /* Structure for setting up the PA .. must be in XDATA */
  tPa_Setup xdata  rPaSetup;

  /* BSR control structure */
  tBsr_Setup xdata rBsrSetup;
  BYTE xdata abBsrPtsPlaceHolder [bPtsSize_c * 2] = {0};
  WORD wPacketCount;

#endif /* _RKE_DEMO_H */