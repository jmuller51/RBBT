/*------------------------------------------------------------------------------
 *                          Silicon Laboratories, Inc.
 *                           http://www.silabs.com
 *                               Copyright 2010
 *------------------------------------------------------------------------------
 *
 *    FILE:       keyfob_main.c
 *    TARGET:     Si4010 RevC
 *    TOOLCHAIN:  Keil
 *    DATE:       Dec 05, 2012,
 *    RELEASE:    1.0 (Tamas Nagy), ROM version 02.00
 *
 *------------------------------------------------------------------------------
 *
 *    DESCRIPTION:
 *      This file contains the main function for the Keil toolchain
 *      4010-kfob-dir project.
 *
 *      BL51 linker directives for building this application:
 *
 *      BL51: PL(68) PW(78) IXREF RS(256)
 *            CODE (0x0-0x08FF)
 *            DATA (0x40)
 *            IDATA (0x70)
 *            XDATA (0x0900-0x107F)
 *            STACK (?STACK(0x90))
 *
 *------------------------------------------------------------------------------
 *
 *    INCLUDES:
 */
#include "stdlib.h"
#include "si4010.h"
#include "si4010_api_rom.h"

// Demo header
#include "keyfob.h"

void  main (void)
{

/*------------------------------------------------------------------------------
 *    SETUP PHASE
 *------------------------------------------------------------------------------
 */
        //Set DMD interrupt to high priority,
        // any other interrupts have to stay low priority
        PDMD=1;
        // Disable the Matrix and Roff modes on GPIO[3:1] 
        PORT_CTRL &= ~(M_PORT_MATRIX | M_PORT_ROFF | M_PORT_STROBE);
        PORT_CTRL |=  M_PORT_STROBE;
        PORT_CTRL &= (~M_PORT_STROBE);
        // Turn LED control off 
        GPIO_LED = 0;
        vSys_Setup( 1 );
        vSys_SetMasterTime(0);
        // Setup the bandgap 
        vSys_BandGapLdo( 1 );

/*        rBsrSetup.pbPtsReserveHead = abBsrPtsPlaceHolder;
        rBsrSetup.bPtsSize = 3;
        rBsrSetup.bPushQualThresh = 3;
        // Setup the BSR 
        vBsr_Setup( &rBsrSetup );
*/
        // Setup the RTC to tick every 5ms and clear it. Keep it disabled. 
        RTC_CTRL = (0x07 << B_RTC_DIV) | M_RTC_CLR;
        // Enable the RTC 
        RTC_CTRL |= M_RTC_ENA;
        // Enable the RTC interrupt and global interrupt enable 
        ERTC = 1;
        EA = 1;
        pbFrameHead = abFrameBuffer;
//------------------------------------------------------------
// Setup RF parameters.

		fDesiredFreqOOK 	 = 166378000.0; // Tweak frequency so that the transduced signal falls at the top of the 3-8kHz detection range of SensorGnome
        fDesiredFreqFSK 	 = 166378000.0;
        bFskDev 		     = 3.5; // Freq mod of 3.5kHz
		rOdsSetup.bModulationType = 1; // OOK = 0; FSK = 1
        // Setup the PA.
        rPaSetup.bLevel      = 77;
        rPaSetup.wNominalCap = 285;
        rPaSetup.bMaxDrv     = 1;

        rPaSetup.fAlpha      = 0.0;
        rPaSetup.fBeta       = 0.0870;
        vPa_Setup( &rPaSetup );


// Setup frequency casting .. needed to be called once per boot 
vFCast_Setup();


// Measure the battery voltage in mV, only once per boot to save power
// Make loaded measurement .. bBatteryWait_c * 17ms wait after loading
iBatteryMv = iMVdd_Measure( bBatteryWait_c );
  
// Setup the DMD temperature sensor for temperature mode 
vDmdTs_RunForTemp( 3 ); // Skip first 3 samples 
// Wait until there is a valid DMD temp sensor sample 
while ( 0 == bDmdTs_GetSamplesTaken() )
{
        //wait
}

 /*------------------------------------------------------------------------------
  *    TRANSMISSION LOOP PHASE
  *------------------------------------------------------------------------------
  */

// Application loop, including push button state analyzation and transmission. 
while(1)
{

	if (GPIO1 == 1)
	{

		vPacketAssemble();
		vSys_SetMasterTime(0);
		vRepeatTxLoop(); 		// Transmit pulse #1

   		while( lSys_GetMasterTime()  < 40 )		// Period (in milliseconds) between starts of pulses 1 & 2
		{}

		vRepeatTxLoop();	// Transmit pulse #2

		while( lSys_GetMasterTime()  < 160 )	// Period (ms) between starts of pulses 1 & 3
		{}

		vRepeatTxLoop();	// Transmit pulse #3

		while( lSys_GetMasterTime()  < 320 )	// Period (ms) between starts of pulses 1 & 4
		{}

		vRepeatTxLoop();	// Transmit pulse #4
    	
		if ((PROT0_CTRL & M_NVM_BLOWN) > 1) //if part is burned to user or run mode.
  		{
            // enable Sleep timer for Low Battery monitoring
			// timer is 2.4kHz - 2400 ticks per second
	 		// for five seconds, 12000 ticks
     		vSleepTim_SetCount(12000 | 0x01000000);
			EA = 0;		//Disable all interrupts
            vSys_Shutdown();      	
		}
	
		//vSys_SetMasterTime(0);
		while((lSys_GetMasterTime() < 3000) && (GPIO1 == 1) ) // Use shorter pulse interval when soft programmed
		{}   
		
		 
	} else {
    	vSys_SetMasterTime(0);
		while( lSys_GetMasterTime()  < 2000 ) //make sure of a deliberate connection
		{} 
		if (GPIO1 == 0) 
		{
			while( lSys_GetMasterTime()  < 5000 ) //make sure connection gets released
			{} 
			
			if (GPIO1 == 1)
			{
				if ((PROT0_CTRL & M_NVM_BLOWN) > 1) //if part is burned to user or run mode.
  				{
            		RTC_CTRL = 0x00;   //Disable the RTC
					EA = 0;		//Disable all interrupts
            		vSys_Shutdown();   //  Need a button push to wake 	
		
				}
		
				while(lSys_GetMasterTime() < 18000 )  // start broadcasting 18s after clip removal
		 		{}
			}
		}
	}
}
}

void vRepeatTxLoop (void)
{ 


        vFCast_Tune( fDesiredFreq );

        // Wait until there is a temperature sample available
        while ( 0 == bDmdTs_GetSamplesTaken() )
        {
                //wait
        }
        //  Tune the PA with the temperature as an argument 
        vPa_Tune( iDmdTs_GetLatestTemp());

        if (rOdsSetup.bModulationType == 1) // ie. if using FSK
        {
                vFCast_FskAdj( bFskDev ); 
        }

        vStl_PreLoop();

		vStl_SingleTxLoop(pbFrameHead,bFrameSize);

        vStl_PostLoop();

//        return; // Doesn't appear necessary
} 

//------------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void isr_rtc (void) interrupt INTERRUPT_RTC using 1

{ 

  // Update the master time by 5 every time this isr is run.
  // clear the RTC_INT 
  RTC_CTRL &= ~M_RTC_INT;
  vSys_IncMasterTime(5); 
  bIsr_DebounceCount ++;
  if ((bIsr_DebounceCount % bDebounceInterval_c) == 0)
  {
    vBsr_Service();
  }
  return;
}
 
/*------------------------------------------------------------------------------
 *
 *    FUNCTION DESCRIPTION:
 *      Update bAp_ButtonState which indicates what to be transmitted.
 *      Check the elements on PTS (push tracking strcture) to see if any GPIO
 *      has been pressed or released.
 *      If any new pressed button has detected, the corresponding flag will be set and
 *      the associated information will be transmitted in
 *      application loop procedure.
 *
 *
 *------------------------------------------------------------------------------ 
 */

void vButtonCheck (void) // IS THIS NECESSARY?
{ 
  // Disable RTC interrupt, or button state might be updated. 
  ERTC = 0;
  bButtonState = 0;  //comment this line out if autorepeat needed
  if (bBsr_GetPtsItemCnt())
     //Some buttons were pressed or released
  {
    
	bButtonState = wBsr_Pop() & 0xFF;
	if (bPrevBtn)
	{
		bPrevBtn = bButtonState;
		bButtonState = 0;
	}
	else
	{
		bPrevBtn = bButtonState;
	}
  }
  // Enable RTC interrupt 
  ERTC = 1;
  return;
}

/*  POSSIBLE BUILD: allow multiple Packet assemblies OR somehow broadcast part of packet at much higher frequency (for hidden barcode)
 *------------------------------------------------------------------------------ 
 */
void vPacketAssemble (void)
{ 
       // BYTE i, j;

        //according to button pushed, insert data bits in packet
        
                        vStl_EncodeSetup( bEnc_NoneNrz_c, NULL );
////                    rOdsSetup.bModulationType = bModOOK_c;
                        if (rOdsSetup.bModulationType == 0)
        				{ fDesiredFreq = fDesiredFreqOOK; }
						else
						{ fDesiredFreq = fDesiredFreqFSK; }

						bFrameSize = 10; // This needs to equal the number of frame buffers used below

                        abFrameBuffer[0] = 0x0f;	
                        abFrameBuffer[1] = 0xff;	
	                    abFrameBuffer[2] = 0x00;	
	                    abFrameBuffer[3] = 0x00;
	                    abFrameBuffer[4] = 0xff;	
	                    abFrameBuffer[5] = 0xff;		
	                    abFrameBuffer[6] = 0x00;		
						abFrameBuffer[7] = 0x00;
						abFrameBuffer[8] = 0xff;
						abFrameBuffer[9] = 0xf0;
/*						abFrameBuffer[10] = 0xf0;
						abFrameBuffer[11] = 0xf0;
*/  // Comment out the rows not needed for desired frame size


// NOTE: check whether low or high state priority is set. This can cause reversal of anticipated pattern.

	                //vCalculateCrc();
	                //vConvertPacket(rOdsSetup.bModulationType);

                        // Packet bitrate (consult SI4010 register calculator spreadsheet)
                        rOdsSetup.wBitRate      = 167;
                        rOdsSetup.bClkDiv       = 5;
                        rOdsSetup.bEdgeRate     = 0;
                        rOdsSetup.bGroupWidth   = 7;
                        rOdsSetup.bLcWarmInt    = 0;
                        rOdsSetup.bDivWarmInt   = 5;
                        rOdsSetup.bPaWarmInt    = 4;
                        vOds_Setup( &rOdsSetup );
	                //bValidBtn = 1;
               
        return;
}

//  IS THIS NECESSARY???
  //-------------------------------------------------------------------
  //MSB first to LSB first conversion, and inversion if FSK used
void vConvertPacket (BYTE bModType)
{
  BYTE i,low,high;

  if (bModType)
  {
    bModType = 0xff;
  }
  for (i=0;i<(sizeof abFrameBuffer);i++)
  {
	low = abConvTable[(abFrameBuffer[i] & 0xf0) >> 4] & 0x0f;
	high = abConvTable[abFrameBuffer[i] & 0x0f] & 0xf0;
	abFrameBuffer[i] = (high | low) ^ bModType;
  }
  return;
}
