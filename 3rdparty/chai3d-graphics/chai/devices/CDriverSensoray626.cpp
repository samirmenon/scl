//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Federico Barbagli
    \version   2.0.0 $Rev: 258 $
*/
//===========================================================================

//---------------------------------------------------------------------------
//#include "extras/CGlobals.h"
//---------------------------------------------------------------------------
#include "devices/CDriverSensoray626.h"
#include <stdio.h>
//---------------------------------------------------------------------------
#if defined(_ENABLE_SENSORAY626_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Special types 
//---------------------------------------------------------------------------
typedef void	(* FPTR_ISR)();					// Pointer to application's interrupt callback function.
typedef void	(* FPTR_ERR)(long ErrFlags);	// Pointer to application's error callback function.


//---------------------------------------------------------------------------
// Error codes returned by GetErrors()
//---------------------------------------------------------------------------
												// System errors that are not reset by S626_GetErrors():
#define ERR_OPEN			0x00000001			//	Can't open driver.
#define ERR_CARDREG			0x00000002			//	Can't attach driver to board.
#define ERR_ALLOC_MEMORY	0x00000004			//	Memory allocation error.
#define ERR_LOCK_BUFFER		0x00000008			//	Error locking DMA buffer.
#define ERR_THREAD			0x00000010			//	Error starting a thread.
#define ERR_INTERRUPT		0x00000020			//	Error enabling interrupt.
#define ERR_LOST_IRQ		0x00000040			//  Missed interrupt.
#define ERR_INIT			0x00000080			//  Board object not instantiated.
#define ERR_VERSION			0x00000100			//  Unsupported WinDriver version.
#define ERR_SUBIDS			0x00000200			//  PCI SubDevice/SubVendor ID mismatch.
#define ERR_CFGDUMP			0x00000400			//  PCI configuration dump failed.

												// Board errors that are reset by S626_GetErrors():
#define ERR_ILLEGAL_PARM	0x00010000			//  Illegal function parameter value was specified.
#define ERR_I2C				0x00020000			//	I2C error.
#define ERR_DACTIMEOUT		0x00100000			//	DAC FB_BUFFER write timeout.
#define ERR_COUNTERSETUP	0x00200000			//	Illegal setup specified for counter channel.


//---------------------------------------------------------------------------
// ADC poll list constants
//---------------------------------------------------------------------------

#define ADC_EOPL			0x80				// End-Of-Poll-List marker.
#define ADC_RANGE_10V		0x00				// Range code for ±10V range.
#define ADC_RANGE_5V		0x10				// Range code for ±5V range.
#define ADC_CHANMASK		0x0F				// Channel number mask.


//---------------------------------------------------------------------------
// Counter constants
//---------------------------------------------------------------------------

												// LoadSrc values:
#define LOADSRC_INDX		0					//	Preload core in response to Index.
#define LOADSRC_OVER		1					//	Preload core in response to Overflow.
#define LOADSRCB_OVERA		2					//	Preload B core in response to A Overflow.
#define LOADSRC_NONE		3					//	Never preload core.

												// IntSrc values:
#define INTSRC_NONE 		0 					//	Interrupts disabled.
#define INTSRC_OVER 		1 					//	Interrupt on Overflow.
#define INTSRC_INDX 		2					//	Interrupt on Index.
#define INTSRC_BOTH 		3					//	Interrupt on Index or Overflow.

												// LatchSrc values:
#define LATCHSRC_AB_READ	0					//	Latch on read.
#define LATCHSRC_A_INDXA	1					//	Latch A on A Index.
#define LATCHSRC_B_INDXB	2					//	Latch B on B Index.
#define LATCHSRC_B_OVERA	3					//	Latch B on A Overflow.

												// IndxSrc values:
#define INDXSRC_HARD		0					//	Hardware or software index.
#define INDXSRC_SOFT		1					//	Software index only.

												// IndxPol values:
#define INDXPOL_POS 		0					//	Index input is active high.
#define INDXPOL_NEG 		1					//	Index input is active low.

												// ClkSrc values:
#define CLKSRC_COUNTER		0					//	Counter mode.
#define CLKSRC_TIMER		2					//	Timer mode.
#define CLKSRC_EXTENDER		3					//	Extender mode.

												// ClkPol values:
#define CLKPOL_POS			0					//	Counter/Extender clock is active high.
#define CLKPOL_NEG			1					//	Counter/Extender clock is active low.
#define CNTDIR_UP			0					//	Timer counts up.
#define CNTDIR_DOWN 		1					//	Timer counts down.

												// ClkEnab values:
#define CLKENAB_ALWAYS		0					//	Clock always enabled.
#define CLKENAB_INDEX		1					//	Clock is enabled by index.

												// ClkMult values:
#define CLKMULT_4X 			0					//	4x clock multiplier.
#define CLKMULT_2X 			1					//	2x clock multiplier.
#define CLKMULT_1X 			2					//	1x clock multiplier.

												// Bit Field positions in COUNTER_SETUP structure:
#define BF_LOADSRC			9					//	Preload trigger.
#define BF_INDXSRC			7					//	Index source.
#define BF_INDXPOL			6					//	Index polarity.
#define BF_CLKSRC			4					//	Clock source.
#define BF_CLKPOL			3					//	Clock polarity/count direction.
#define BF_CLKMULT			1					//	Clock multiplier.
#define BF_CLKENAB			0					//	Clock enable.

												// Counter channel numbers:
#define CNTR_0A 			0					//	Counter 0A.
#define CNTR_1A 			1					//	Counter 1A.
#define CNTR_2A 			2					//	Counter 2A.
#define CNTR_0B 			3					//	Counter 0B.
#define CNTR_1B 			4					//	Counter 1B.
#define CNTR_2B 			5					//	Counter 2B.

// Counter overflow/index event flag masks for S626_CounterCapStatus() and S626_InterruptStatus().
#define INDXMASK(C)		( 1 << ( ( (C) > 2 ) ? ( (C) * 2 - 1 ) : ( (C) * 2 +  4 ) ) )
#define OVERMASK(C)		( 1 << ( ( (C) > 2 ) ? ( (C) * 2 + 5 ) : ( (C) * 2 + 10 ) ) )


//---------------------------------------------------------------------------
#include <windows.h>
//---------------------------------------------------------------------------

// Error codes returned by S626_DLLOpen().
#define ERR_LOAD_DLL	1		// Failed to open S626.DLL.
#define ERR_FUNCADDR	2		// Failed to find function name in S626.DLL.

//#ifndef XFUNC626

// Pointers to DLL functions:
//#define XFUNC626(RTNTYPE,FUNCNAME)		extern "C" RTNTYPE (__stdcall *FUNCNAME)

// S626.DLL Startup and Shutdown functions: 
extern "C" unsigned long S626_DLLOpen();		// Open DLL and initialize function pointers.
extern "C" void S626_DLLClose();	// Close DLL.

//#endif	

#define XFUNC626( RTNTYPE, FUNCNAME )		RTNTYPE ( __stdcall *FUNCNAME )

//---------------------------------------------------------------------------
// Pointers to functions that are exported by S626.DLL.
//---------------------------------------------------------------------------

// Status and control functions.
XFUNC626( unsigned long,	S626_GetAddress				)( unsigned long hbd );
XFUNC626( unsigned long,	S626_GetErrors				)( unsigned long hbd );
XFUNC626( void,				S626_OpenBoard				)( unsigned long hbd, unsigned long PhysLoc, FPTR_ISR IntFunc, unsigned long Priority );
XFUNC626( void,				S626_CloseBoard				)( unsigned long hbd );
XFUNC626( void,				S626_InterruptEnable		)( unsigned long hbd, unsigned short enable );
XFUNC626( void,				S626_InterruptStatus		)( unsigned long hbd, unsigned short *status );
XFUNC626( void,				S626_SetErrCallback			)( unsigned long hbd, FPTR_ERR Callback );

// Diagnostics.
XFUNC626( unsigned char,	S626_I2CRead				)( unsigned long hbd, unsigned char addr );
XFUNC626( void,				S626_I2CWrite				)( unsigned long hbd, unsigned char addr, unsigned char value );
XFUNC626( unsigned short,	S626_RegRead				)( unsigned long hbd, unsigned short addr );
XFUNC626( void,				S626_RegWrite				)( unsigned long hbd, unsigned short addr, unsigned short value );

// Analog I/O functions.
XFUNC626( void,				S626_ReadADC				)( unsigned long hbd, short *databuf );
XFUNC626( void,				S626_ResetADC				)( unsigned long hbd, unsigned char *pollist );
XFUNC626( void,				S626_StartADC				)( unsigned long hbd);
XFUNC626( void,				S626_WaitDoneADC			)( unsigned long hbd, short *pdata);
XFUNC626( void,				S626_WriteDAC				)( unsigned long hbd, unsigned short chan, long value );
XFUNC626( void,				S626_WriteTrimDAC			)( unsigned long hbd, unsigned char chan, unsigned char value );

// Digital I/O functions.
XFUNC626( unsigned short,	S626_DIOReadBank			)( unsigned long hbd, unsigned short group );
XFUNC626( unsigned short,	S626_DIOWriteBankGet		)( unsigned long hbd, unsigned short group );
XFUNC626( void,				S626_DIOWriteBankSet		)( unsigned long hbd, unsigned short group, unsigned short value );
XFUNC626( unsigned short,	S626_DIOEdgeGet				)( unsigned long hbd, unsigned short group );
XFUNC626( void,				S626_DIOEdgeSet				)( unsigned long hbd, unsigned short group, unsigned short value );
XFUNC626( void,				S626_DIOCapEnableSet		)( unsigned long hbd, unsigned short group, unsigned short chanmask, unsigned short enable );
XFUNC626( unsigned short,	S626_DIOCapEnableGet		)( unsigned long hbd, unsigned short group );
XFUNC626( void,				S626_DIOCapStatus			)( unsigned long hbd, unsigned short group );
XFUNC626( void,				S626_DIOCapReset			)( unsigned long hbd, unsigned short group, unsigned short value );
XFUNC626( unsigned short,	S626_DIOIntEnableGet		)( unsigned long hbd, unsigned short group );
XFUNC626( void,				S626_DIOIntEnableSet		)( unsigned long hbd, unsigned short group, unsigned short value );
XFUNC626( unsigned short,	S626_DIOModeGet				)( unsigned long hbd, unsigned short group );
XFUNC626( void,				S626_DIOModeSet				)( unsigned long hbd, unsigned short group, unsigned short value );

// Counter functions.
XFUNC626( void,				S626_CounterModeSet			)( unsigned long hbd, unsigned short chan, unsigned short mode );
XFUNC626( unsigned short,	S626_CounterModeGet			)( unsigned long hbd, unsigned short chan );
XFUNC626( void,				S626_CounterEnableSet		)( unsigned long hbd, unsigned short chan, unsigned short enable );
XFUNC626( void,				S626_CounterPreload			)( unsigned long hbd, unsigned short chan, unsigned long value );
XFUNC626( void,				S626_CounterLoadTrigSet		)( unsigned long hbd, unsigned short chan, unsigned short value );
XFUNC626( void,				S626_CounterLatchSourceSet	)( unsigned long hbd, unsigned short chan, unsigned short value );
XFUNC626( unsigned long,	S626_CounterReadLatch		)( unsigned long hbd, unsigned short chan );
XFUNC626( unsigned short,	S626_CounterCapStatus		)( unsigned long hbd );
XFUNC626( void,				S626_CounterCapFlagsReset	)( unsigned long hbd, unsigned short chan );
XFUNC626( void,				S626_CounterSoftIndex		)( unsigned long hbd, unsigned short chan );
XFUNC626( void,				S626_CounterIntSourceSet	)( unsigned long hbd, unsigned short chan, unsigned short value );

// Battery functions:
XFUNC626( unsigned short,	S626_BackupEnableGet		)( unsigned long hbd );
XFUNC626( void,				S626_BackupEnableSet		)( unsigned long hbd, unsigned short en );
XFUNC626( unsigned short,	S626_ChargeEnableGet		)( unsigned long hbd );
XFUNC626( void,				S626_ChargeEnableSet		)( unsigned long hbd, unsigned short en );

// Watchdog functions:
XFUNC626( unsigned short,	S626_WatchdogTimeout		)( unsigned long hbd );
XFUNC626( unsigned short,	S626_WatchdogEnableGet		)( unsigned long hbd );
XFUNC626( void,				S626_WatchdogEnableSet		)( unsigned long hbd, unsigned short en );
XFUNC626( unsigned short,	S626_WatchdogPeriodGet		)( unsigned long hbd );
XFUNC626( void,				S626_WatchdogPeriodSet		)( unsigned long hbd, unsigned short val );
XFUNC626( void,				S626_WatchdogReset			)( unsigned long hbd );


//---------------------------------------------------------------------------
static HINSTANCE hlib;
//---------------------------------------------------------------------------

DWORD GetFuncPtrs()
{
	// Status and control functions.
    S626_GetAddress         = (unsigned long (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_GetAddress");
    S626_GetErrors          = (unsigned long (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_GetErrors");
    S626_OpenBoard          = (void (__stdcall*)(unsigned long, unsigned long, FPTR_ISR, unsigned long ))GetProcAddress(hlib, "S626_OpenBoard");
    S626_CloseBoard         = (void (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_CloseBoard");
    S626_InterruptEnable    = (void (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_InterruptEnable");
    S626_InterruptStatus    = (void (__stdcall*)(unsigned long, unsigned short*))GetProcAddress(hlib, "S626_InterruptStatus");
    
	// Diagnostics.
    S626_I2CRead          = (unsigned char (__stdcall*)(unsigned long, unsigned char))GetProcAddress(hlib, "S626_I2CRead");
    S626_I2CWrite         = (void (__stdcall*)(unsigned long, unsigned char, unsigned char))GetProcAddress(hlib, "S626_I2CWrite");
    S626_RegRead          = (unsigned short (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_RegRead");
    S626_RegWrite         = (void (__stdcall*)(unsigned long, unsigned short, unsigned short ))GetProcAddress(hlib, "S626_RegWrite");
        
	// Analog I/O functions.
    S626_ReadADC          = (void (__stdcall*)(unsigned long, short *))GetProcAddress(hlib, "S626_ReadADC");
    S626_ResetADC         = (void (__stdcall*)(unsigned long, unsigned char *))GetProcAddress(hlib, "S626_ResetADC");
    S626_WriteDAC         = (void (__stdcall*)(unsigned long, unsigned short, long))GetProcAddress(hlib, "S626_WriteDAC");
    S626_WriteTrimDAC     = (void (__stdcall*)(unsigned long, unsigned char, unsigned char))GetProcAddress(hlib, "S626_WriteTrimDAC");

	// Digital I/O functions.
    S626_DIOReadBank        = (unsigned short (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_DIOReadBank");
    S626_DIOWriteBankGet    = (unsigned short (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_DIOWriteBankGet");
    S626_DIOWriteBankSet    = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_DIOWriteBankSet");
    S626_DIOEdgeGet         = (unsigned short (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_DIOEdgeGet");
    S626_DIOEdgeSet         = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_DIOEdgeSet");
    S626_DIOCapEnableSet    = (void (__stdcall*)(unsigned long, unsigned short, unsigned short, unsigned short))GetProcAddress(hlib, "S626_DIOCapEnableSet");
    S626_DIOCapEnableGet    = (unsigned short (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_DIOCapEnableGet");
    S626_DIOCapReset        = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_DIOCapReset");
    S626_DIOCapStatus       = (void (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_DIOCapStatus");
    S626_DIOIntEnableGet    = (unsigned short (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_DIOIntEnableGet");
    S626_DIOIntEnableSet    = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_DIOIntEnableSet");
    S626_DIOModeGet         = (unsigned short (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_DIOModeGet");
    S626_DIOModeSet         = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_DIOModeSet");

	// Counter functions.
    S626_CounterModeSet     = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_CounterModeSet");
    S626_CounterModeGet     = (unsigned short (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_CounterModeGet");
    S626_CounterEnableSet   = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_CounterEnableSet");
    S626_CounterPreload     = (void (__stdcall*)(unsigned long, unsigned short, unsigned long))GetProcAddress(hlib, "S626_CounterPreload");
    S626_CounterLoadTrigSet = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_CounterLoadTrigSet");
    S626_CounterLatchSourceSet  = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_CounterLatchSourceSet");
    S626_CounterReadLatch       = (unsigned long (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_CounterReadLatch");
    S626_CounterCapStatus       = (unsigned short (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_CounterCapStatus");
    S626_CounterCapFlagsReset   = (void (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_CounterCapFlagsReset");
    S626_CounterSoftIndex       = (void (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_CounterSoftIndex");
    S626_CounterIntSourceSet    = (void (__stdcall*)(unsigned long, unsigned short, unsigned short))GetProcAddress(hlib, "S626_CounterIntSourceSet");

	// Battery functions.
    S626_BackupEnableGet    = (unsigned short (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_BackupEnableGet");
    S626_BackupEnableSet    = (void (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_BackupEnableSet");
    S626_ChargeEnableGet    = (unsigned short (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_ChargeEnableGet");
    S626_ChargeEnableSet    = (void (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_ChargeEnableSet");
    
	// Watchdog functions.
    S626_WatchdogTimeout    = (unsigned short (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_WatchdogTimeout");
    S626_WatchdogEnableGet  = (unsigned short (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_WatchdogEnableGet");
    S626_WatchdogEnableSet  = (void (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_WatchdogEnableSet");
    S626_WatchdogPeriodGet  = (unsigned short (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_WatchdogPeriodGet");
    S626_WatchdogPeriodSet  = (void (__stdcall*)(unsigned long, unsigned short))GetProcAddress(hlib, "S626_WatchdogPeriodSet");
    S626_WatchdogReset      = (void (__stdcall*)(unsigned long))GetProcAddress(hlib, "S626_WatchdogReset");


    if (S626_GetAddress == NULL)
    {
        return (-1);    // Failed to find functions in DLL.
    }
    else
    {
	    return (0);		// Indicate all is OK.
    }
}

//---------------------------------------------------------------------------
// Open S626.DLL and get pointers to exported DLL functions.
//---------------------------------------------------------------------------

DWORD S626_DLLOpen()
{
	// Dynamically link to S626.DLL, exit with error if link failed.
	hlib = LoadLibrary( "S626.DLL" );
	if ( hlib == NULL )
		return ERR_LOAD_DLL;

	// Fill pointers to S626.DLL functions, exit with error if attempt failed.
	if ( GetFuncPtrs() )
	{
		FreeLibrary( hlib );
		return ERR_FUNCADDR;
	}

	// Normal return.
	return 0;
}

//---------------------------------------------------------------------------
// Release dynamic link to S626.DLL.
//---------------------------------------------------------------------------

VOID S626_DLLClose()
{
	// Unlink from S626.DLL.
	if ( hlib )
		FreeLibrary( hlib );
}


//---------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------
const int MAX_AXIS_626              = 6;
const int NUM_DACs_626              = 4;
const int DAC_RANGE_626             = 8191;
const double VOLT_RANGE_626         = 10.0;
int cDriverSensoray626::m_BOARD_NUM = 0;
//---------------------------------------------------------------------------

//==========================================================================
/*!
      Constructor of cDriverSensoray626.

      \fn       cDriverSensoray626::cDriverSensoray626()
*/
//===========================================================================
cDriverSensoray626::cDriverSensoray626()
{
    m_boardHandle = m_BOARD_NUM;
    m_BOARD_NUM++;
    m_systemReady = false;
    m_systemAvailable = false;
}


//==========================================================================
/*!
      Destructor of cDriverSensoray626.

      \fn       cDriverSensoray626::~cDriverSensoray626()
*/
//===========================================================================
cDriverSensoray626::~cDriverSensoray626()
{
    close();
}

//==========================================================================
/*!
      Open connection to Sensoray board

      \fn     int cDriverSensoray626::open()
      \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cDriverSensoray626::open()
{
    // check if device is not already opened
    if (m_systemReady) { return (0); }

    // number of encoders used.
    for (int i = 0; i<MAX_AXIS_626; i++)
    {
        homeposition[i] = 0;
    }

    // open DLL to talk to the board
    if (S626_DLLOpen()==0)
    {
        m_systemAvailable = true;

        // Declare Model 626 board to driver and launch the interrupt thread.
        // NOTE: we're supposing to only use one board. With two board we need
        // to specify the physical address for each board used.
        // ALSO: we're NOT using interrupts from the board.
        S626_OpenBoard( m_boardHandle, 0, 0, THREAD_PRIORITY_ABOVE_NORMAL );
        unsigned long ErrCode = S626_GetErrors( 0 );
        if (ErrCode)
        {
            m_systemReady = false;
            return (-1);
        }
        else
        {
            m_systemReady = true;
        }

        m_wBaseAddress = 0;
        encoderInit();
        for (int i = 0; i<NUM_DACs_626; i++)
        {
            S626_WriteDAC(m_boardHandle, (int) i, 0);
        }
        return (0);
    }
    else
    {
        m_systemReady = false;
        m_systemAvailable = false;
        return (-1);
    }

    return (0);
}


//==========================================================================
/*!
    Close connection to the board, write a zero value to all DACs.

    \fn       int cDriverSensoray626::close()
    \return   Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cDriverSensoray626::close()
{
    if (m_systemReady)
    {
        for (int i = 0; i<NUM_DACs_626; i++)
        {
            S626_WriteDAC(m_boardHandle, (int) i, 0);
        }

        S626_CloseBoard(m_boardHandle);
    }

    m_systemReady = false;
    m_systemAvailable = false;

    return (0);
}


//==========================================================================
/*!
    Sets all counters on the board to be used as encoders.

    \fn     void cDriverSensoray626::encoderInit()
*/
//===========================================================================
void cDriverSensoray626::encoderInit()
{
    // Initialize all encoders at once
    for (int i = 0; i<6; i++)
    {
        S626_CounterModeSet(m_boardHandle, i,
        ( LOADSRC_INDX		<<	BF_LOADSRC	) |
        ( INDXSRC_SOFT		<<	BF_INDXSRC	) |
        ( INDXPOL_POS		<<	BF_INDXPOL	) |
        ( CLKSRC_COUNTER	<<	BF_CLKSRC	) |
        ( CLKPOL_POS		<<	BF_CLKPOL	) |
        ( CLKMULT_4X		<<	BF_CLKMULT	) |
        ( CLKENAB_ALWAYS	<<	BF_CLKENAB	) );

        // Set the counter CNTR_0A on BOARD to 0
        S626_CounterPreload( m_boardHandle, i, 100000 );
        S626_CounterLoadTrigSet(m_boardHandle, i, 1);
        S626_CounterLatchSourceSet( m_boardHandle, i, LATCHSRC_AB_READ );
        S626_CounterIntSourceSet( m_boardHandle, i, INTSRC_INDX );
        homeposition[i] = S626_CounterReadLatch(m_boardHandle, i);
    }
}


//==========================================================================
/*!
      Initializes board. In this implementation there's really nothing to do
      that hasn't been done in the opening phase.

      \fn     int cDriverSensoray626::initialize(const bool a_resetEncoders=false)
      \param  a_resetEncoders  Ignored; reserved for forward compatibility
      \return Return 0 is operation succeeds, -1 if an error occurs.

*/
//===========================================================================
int cDriverSensoray626::initialize(const bool a_resetEncoders)
{
    if (m_systemReady)
    {
        return (0);
    }
    else
    {
        return (-1);
    }
}


//===========================================================================
/*!
    Set command to the Sensoray626 board.

    \fn     int cDriverSensoray626::command(int a_command, void* a_data)
    \param  a_command    Selected command.
    \param  a_data       Pointer to the corresponding data structure.
    \return Return status of command.
*/
//===========================================================================
int cDriverSensoray626::command(int a_command, void* a_data)
{
    int retval = CHAI_MSG_OK;
    if (m_systemReady)
    {
       switch (a_command)
       {
           case CHAI_CMD_GET_DEVICE_STATE:
           {
               *(int *) a_data = m_systemReady;
           }
           break;
           // reset encoder 0
           case CHAI_CMD_RESET_ENCODER_0:
           {
               // Read in encoder positions here
               unsigned long cur_pos;
               cur_pos = S626_CounterReadLatch(m_boardHandle, 0);
               homeposition[0] = cur_pos;
           }
           break;
           // reset encoder 1
           case CHAI_CMD_RESET_ENCODER_1:
           {
               // Read in encoder positions here
               unsigned long cur_pos;
               cur_pos = S626_CounterReadLatch(m_boardHandle, 1);
               homeposition[1] = cur_pos;
           }
           break;
           // reset encoder 2
           case CHAI_CMD_RESET_ENCODER_2:
           {
               // Read in encoder positions here
               unsigned long cur_pos;
               cur_pos = S626_CounterReadLatch(m_boardHandle, 2);
               homeposition[2] = cur_pos;
           }
           break;
           // reset encoder 3
           case CHAI_CMD_RESET_ENCODER_3:
           {
               // Read in encoder positions here
               unsigned long cur_pos;
               cur_pos = S626_CounterReadLatch(m_boardHandle, 3);
               homeposition[3] = cur_pos;
           }
           break;
           // reset encoder 4
           case CHAI_CMD_RESET_ENCODER_4:
           {
               // Read in encoder positions here
               unsigned long cur_pos;
               cur_pos = S626_CounterReadLatch(m_boardHandle, 4);
               homeposition[4] = cur_pos;
           }
           break;
           // reset encoder 5
           case CHAI_CMD_RESET_ENCODER_5:
           {
               // Read in encoder positions here
               unsigned long cur_pos;
               cur_pos = S626_CounterReadLatch(m_boardHandle, 5);
               homeposition[5] = cur_pos;
           }
           break;
           // read encoder 0
           case CHAI_CMD_GET_ENCODER_0:
           {
               long* iValue = (long *) a_data;
               unsigned long cur_pos;
               // Read in encoder positions here
               cur_pos = S626_CounterReadLatch(m_boardHandle, 0) - homeposition[0];
               *iValue = (long) cur_pos;
           }
           break;
           // read encoder 1
           case CHAI_CMD_GET_ENCODER_1:
           {
               long* iValue = (long *) a_data;
               unsigned long cur_pos;
               // Read in encoder positions here
               cur_pos = S626_CounterReadLatch(m_boardHandle, 1) - homeposition[1];
               *iValue = (long) cur_pos;
           }
           break;
           // read encoder 2
           case CHAI_CMD_GET_ENCODER_2:
           {
               long* iValue = (long *) a_data;
               unsigned long cur_pos;
               // Read in encoder positions here
               cur_pos = S626_CounterReadLatch(m_boardHandle, 2) - homeposition[2];
               *iValue = (long) cur_pos;
           }
           break;
           // read encoder 3
           case CHAI_CMD_GET_ENCODER_3:
           {
               long* iValue = (long *) a_data;
               unsigned long cur_pos;
               // Read in encoder positions here
               cur_pos = S626_CounterReadLatch(m_boardHandle, 3) - homeposition[3];
               *iValue = (long) cur_pos;
           }
           break;
           // read encoder 4
           case CHAI_CMD_GET_ENCODER_4:
           {
               long* iValue = (long *) a_data;
               unsigned long cur_pos;
               // Read in encoder positions here
               cur_pos = S626_CounterReadLatch(m_boardHandle, 4) - homeposition[4];
               *iValue = (long) cur_pos;
           }
           break;
           // read encoder 5
           case CHAI_CMD_GET_ENCODER_5:
           {
               long* iValue = (long *) a_data;
               unsigned long cur_pos;
               // Read in encoder positions here
               cur_pos = S626_CounterReadLatch(m_boardHandle, 5) - homeposition[5];
               *iValue = (long) cur_pos;
           }
           break;
           // write motor 0
           case CHAI_CMD_SET_DAC_0:
           {
               short lCounts;
               double *iVolts = (double *) a_data;
               if (*iVolts> VOLT_RANGE_626)
                    *iVolts = VOLT_RANGE_626;
               if (*iVolts < -VOLT_RANGE_626)
                    *iVolts = -VOLT_RANGE_626;
               // convert value from volts to a value between -DAC_RANGE_626 and DAC_RANGE_626
               lCounts = (short ((double) DAC_RANGE_626 * (*iVolts/VOLT_RANGE_626)));
               S626_WriteDAC(m_boardHandle, (int) 0, lCounts);
            }
            break;
            // write motor 1
           case CHAI_CMD_SET_DAC_1:
           {
               short lCounts;
               double *iVolts = (double *) a_data;
               if (*iVolts> VOLT_RANGE_626)
                    *iVolts = VOLT_RANGE_626;
               if (*iVolts < -VOLT_RANGE_626)
                    *iVolts = -VOLT_RANGE_626;
               // convert value from volts to a value between -DAC_RANGE_626 and DAC_RANGE_626
               lCounts = (short ((double) DAC_RANGE_626 * (*iVolts/VOLT_RANGE_626)));
               S626_WriteDAC(m_boardHandle, (int) 1, lCounts);
            }
            break;
           // write motor 2
           case CHAI_CMD_SET_DAC_2:
           {
               short lCounts;
               double *iVolts = (double *) a_data;
               if (*iVolts> VOLT_RANGE_626)
                    *iVolts = VOLT_RANGE_626;
               if (*iVolts < -VOLT_RANGE_626)
                    *iVolts = -VOLT_RANGE_626;
               // convert value from volts to a value between -DAC_RANGE_626 and DAC_RANGE_626
               lCounts = (short ((double) DAC_RANGE_626 * (*iVolts/VOLT_RANGE_626)));
               S626_WriteDAC(m_boardHandle, (int) 2, lCounts);
            }
            break;
            // write motor 3
            case CHAI_CMD_SET_DAC_3:
            {
               short lCounts;
               double *iVolts = (double *) a_data;
               if (*iVolts> VOLT_RANGE_626)
                    *iVolts = VOLT_RANGE_626;
               if (*iVolts < -VOLT_RANGE_626)
                    *iVolts = -VOLT_RANGE_626;
               // convert value from volts to a value between -DAC_RANGE_626 and DAC_RANGE_626
               lCounts = (short ((double) DAC_RANGE_626 * (*iVolts/VOLT_RANGE_626)));
               S626_WriteDAC(m_boardHandle, (int) 3, lCounts);
            }
            break;
           // function is not implemented for phantom devices
           default:
            retval = CHAI_MSG_NOT_IMPLEMENTED;
        }
    }
    else
    {
          retval = CHAI_MSG_ERROR;
    }
    return retval;
}

//---------------------------------------------------------------------------
#endif //_ENABLE_SENSORAY626_DEVICE_SUPPORT
//---------------------------------------------------------------------------
