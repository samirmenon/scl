//////////////////////////////////////////////////////////////////////////////////
// Module	 : App2600.h
// Function  : Header file for Sensoray 2600 applications
// Target OS : Any (OS independent)
// Usage	 : Included by all apps using 2600 boards
// Copyright : (C) 2008 Sensoray
//////////////////////////////////////////////////////////////////////////////////


#ifndef _INC_APP2600_H_		// These declarations may only be included once:
#define _INC_APP2600_H_

#include "s26app.h"							// OS-dependent declarations.

// Device identifiers ------------------------------------------------------

// Logical devices.
#define LOGDEV_GATEWAY			0x00		// Gateway.
#define LOGDEV_COM1				0x01		// ComPort #1.
#define LOGDEV_COM2				0x02		// ComPort #2.
#define LOGDEV_COM3				0x03		// ComPort #3.
#define LOGDEV_COM4				0x04		// ComPort #4.

// Module identifiers.
#define MODID_GATEWAY			0xFF		// MM gateway.

// Constants for model 2608 analog i/o module ---------------------------------------------------

#define MAX_NUM_AOUTS			8			// Maximum analog output channel count (model 2608-8).

// Analog input types for S26_Sched2608_SetAinTypes().
											// Voltage:
#define RAW_LG_TYPE				0			//   Offset/range corrected ADC (in internal units, +/-32K): 10V scale
#define RAW_HG_TYPE				1			//   Offset/range corrected ADC (in internal units, +/-32K): 0.01V scale
#define V_10_TYPE				2			//   Measured voltage (in Volts): 10V scale
#define V_001_TYPE				3 			//   Measured voltage (in Volts): 0.01V scale
											// Thermocouple: output values are temperatures in degrees C.
#define TC_B_TYPE				4			//   Thermocouple B type.
#define TC_C_TYPE				5			//   Thermocouple C type.
#define TC_E_TYPE				6			//   Thermocouple E type.
#define TC_J_TYPE				7			//   Thermocouple J type.
#define TC_K_TYPE				8			//   Thermocouple K type.
#define TC_N_TYPE				9			//   Thermocouple N type.
#define TC_R_TYPE				10			//   Thermocouple R type.
#define TC_S_TYPE				11			//   Thermocouple S type.
#define TC_T_TYPE				12			//   Thermocouple T type.

// ADC acquisition interval specifiers for S26_Sched2608_GetAins().
#define ADC_SNAPSHOT			0			// Fetch snapshot data that is acquired asynchronously by the 2608 every 2 msec.
#define ADC_INTEGRATED			1			// Fetch integrated data that is acquired asynchronously by the 2608 at the line frequency.

// Line frequency specifiers for S26_Sched2608_SetLineFreq().
#define LINEFREQ_50HZ			1			// Line frequency is 50 Hz.
#define LINEFREQ_60HZ			0			// Line frequency is 60 Hz.

// Constants for model 2612 high-resolution ADC module ---------------------------------------------

#define REF_OUT_1V			    0			// Output reference voltage 1V.
#define REF_OUT_2V			    1			// Output reference voltage 2V.
#define REF_OUT_3V			    2			// Output reference voltage 3V.
#define REF_OUT_5V			    3			// Output reference voltage 5V.

// A/D conversion modes  ---------------------------------------------------------------------------
											// Over Sample Ratio (OSR) / Conversion Rate in 1X mode:
#define OSR_64          0xA0100000			// 64        3.52kHz
#define OSR_128         0xA0200000          // 128       1.76kHz
#define OSR_256         0xA0300000          // 256       880Hz
#define OSR_512         0xA0400000          // 512       440Hz
#define OSR_1024        0xA0500000          // 1024      220Hz
#define OSR_2048        0xA0600000          // 2048      110Hz
#define OSR_4096        0xA0700000          // 4096      55Hz
#define OSR_8192        0xA0800000          // 8192      27.5Hz
#define OSR_16384       0xA0900000          // 16384     13.75Hz
#define OSR_32768       0xA0F00000          // 32768     6.875Hz

#define MODE_2X         0x00080000          // 2X speed mode

// Constants for model 2620 counter module ---------------------------------------

// Soft trigger commands.
#define LOAD_2620				1			// Copy preload register to counter core.
#define LATCH_2620				2			// Transfer counter core to data latch.

// Constants for model 2631 motor controller module -------------------------------

// Output mode register structure.
#define MODE_2631_REVERSE		1			// Reverse bit.
#define MODE_2631_RUN    		2			// Run bit. Reset if 0.

// Serial comport configuration constants -----------------------------------------

// Baud rate clock divisors.
#define SIO_BR_300				0x0300		//    300 bps.
#define SIO_BR_600				0x0180		//    600 bps.
#define SIO_BR_1200				0x00C0		//  1,200 bps.
#define SIO_BR_2400				0x0060		//  2,400 bps.
#define SIO_BR_4800				0x0030		//  4,800 bps.
#define SIO_BR_9600				0x0018		//  9,600 bps.
#define SIO_BR_19200			0x000C		// 19,200 bps.
#define SIO_BR_38400			0x0006		// 38,400 bps.
#define SIO_BR_57600			0x0004		// 57,600 bps.
#define SIO_BR_115200			0x0002		// 115,200 bps.

// Parity types.
#define SIO_PARITY_NONE			0x00		// Parity disabled.
#define SIO_PARITY_ODD			0x08		// Odd parity.
#define SIO_PARITY_EVEN			0x18		// Even parity.

// Character bit counts.
#define SIO_DATA_5				0x00		// 5 data bits.
#define SIO_DATA_6				0x01		// 6 data bits.
#define SIO_DATA_7				0x02		// 7 data bits.
#define SIO_DATA_8				0x03		// 8 data bits.

// Stop bits.
#define SIO_STOP_1				0x00		// 1 stop bit.
#define SIO_STOP_2				0x04		// 2 stop bits (databits>5), or 1.5 stop bits (databits=5).

// Interface type.
#define SIO_PHY_RS232			0x00		// RS-232.
#define SIO_PHY_RS422_IDLEON	0x40		// RS-422, transmitter always enabled.
#define SIO_PHY_RS485			0x80		// RS-485.
#define SIO_PHY_RS422_IDLEOFF	0xC0		// RS-422, transmitter tri-stated when idle.

// Flow control.
#define SIO_FLOW_OFF			0x00		// No flow control.
#define SIO_FLOW_ON				0x20		// XON-XOFF flow control.

// LED events that will cause the comport status LED to light.
#define SIO_LED_TRANSMIT		0x08		// Light upon char transmit.
#define SIO_LED_RECEIVE			0x04		// Light upon char receive.
#define SIO_LED_ERROR			0x02		// Light upon error or break.

// Error codes --------------------------------------------------------------------

// Gateway transactions.
////typedef unsigned long			GWERR;	// Gateway error codes:
#define GWERR_NONE				0x00000000	// No gateway errors detected.
#define GWERR_TOOLARGE			0x00000100	// Too many MCmds in a command packet, or cmd/rsp packet exceeds 1KB.
#define GWERR_XACTALLOC			0x00000300	// Couldn't allocate transaction object.
#define GWERR_IOMNORESPOND		0x00000400	// Bad module response, ModID is in LSB of ErrorCode.
#define GWERR_MMCLOSED			0x00000500	// MM is not open.
#define GWERR_IOMCLOSED			0x00000600	// Iom is not open.
#define GWERR_IOMTYPE			0x00000700	// Action is not supported by the registered iom type.
#define GWERR_IOMERROR			0x00000800	// Iom comm error flag (CERR) is asserted.  See IomStatus[] for details.
#define GWERR_IOMRESET			0x00000900	// Iom reset flag (RST) is asserted.  See IomStatus[] for details.
#define GWERR_MMNORESPOND		0x00000A00	// MM response timed out.
#define GWERR_PACKETSEND		0x00000B00	// Socket driver failed to send command packet.
#define GWERR_IOMSPECIFIC		0x00000C00	// Iom-specific status flag is asserted.  See IomStatus[] for details.
#define GWERR_BADVALUE			0x00000D00	// Illegal argument value (e.g., channel number does not exist).
#define GWERRMASK				0xFFFFFF00	// Mask for gateway error.  Truncates additional info.

// Returned by S26_DriverOpen().
#define DRVERR_NONE				0x00000000	// No errors.
#define DRVERR_MALLOC			0x00000001	// Memory allocation failed.
#define DRVERR_NETWORKOPEN		0x00000002	// Socket library problem (i.e., too many sockets, bad version).
#define DRVERR_CRITICALSECTION	0x00000003	// Can't create critical section.
#define DRVERR_REOPEN			0x00000004	// Attempted to open api again before closing it.

// Returned by S26_BoardOpen().
#define ERR_NONE				0x00000000	// No errors.
#define ERR_BADHANDLE			0x00000001	// Illegal MM handle.
#define ERR_CREATESOCKET		0x00000002	// Can't create socket.
#define ERR_BINDSOCKET			0x00000004	// Can't bind socket to NIC.

// Returned by S26_SchedExecuteIsResponded().
#define RESP_READY				0x00000000	// Response has been received without errors.
#define RESP_BUSY				0x00000001	// Response not yet received, but no errors detected.

// IOM status bits ----------------------------------------------------------------

// Common to all iom types.
#define STATUS_RST				0x80		// Module reset flag.
#define STATUS_CERR				0x40		// Communication error detected.

// Iom type-specific.
#define STATUS_2608_CALERR		0x10		// 2608: Calibration values out of tolerance, defaults applied.
#define STATUS_2610_STRM		0x01		// 2610: Serial control stream error.
#define STATUS_2612_OVERFLOW	0x01		// 2612: Out of range (positive).
#define STATUS_2612_UNDERFLOW	0x02		// 2612: Out of range (negative).
#define STATUS_2612_EEPROM		0x08		// 2612: EEPROM read/write error.
#define STATUS_2650_DRVR		0x02		// 2650: Relay coil driver fault.
#define STATUS_2650_STRM		0x01		// 2650: Serial control stream error.
#define STATUS_2652_STRM		0x01		// 2652: Serial control stream error.
#define STATUS_2653_STRM		0x01		// 2653: Serial control stream error.

// Serial comport status bits -----------------------------------------------------

#define COM_REJECTED			0x80		// Command rejected by the MM.
#define COM_FRAMINGERROR		0x08		// Framing error detected.
#define COM_PARITYERROR			0x04		// Parity error detected.
#define COM_OVERFLOWERROR		0x02		// UART or receive buffer overflowed.
#define COM_ISOPEN				0x01		// Comport is open.


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////  TYPES  ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


typedef u32						HBD;		// Handle to a MM.
typedef u32						GWERR;		// Error code.
typedef u8						IOMPORT;	// IOM port number on a MM.

#ifndef S26_BUILDING_MOD
typedef void *					HXACT;		// Transaction handle, from the application's perspective.
#endif


/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////  API FUNCTION DECLARATIONS  /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


#endif	// #ifndef _INC_APP2600_H_		These macros may be included multiple times:


// Initialization, shutdown and status.
API( int, 		S26_DriverOpen,						( int NumMMs ) );
API( void,		S26_DriverClose,					( void ) );
API( const char *, S26_DriverVersion,				( void ) );
API( u32,		S26_BoardOpen,						( HBD hbd, char *CLDotAdrs, char *MMDotAdrs ) );
API( void,  	S26_BoardClose,						( HBD hbd ) );
API( GWERR, 	S26_RegisterAllIoms,				( HBD hbd, u32 msec, u16 *NumBoards, u16 *IomTypeList, u8 *IomStatus, u32 retries ) );
API( GWERR, 	S26_ResetIom,						( HBD hbd, IOMPORT IomPort, u32 msec, u32 retries ) );
API( BOOL,  	S26_ResetNetwork,					( HBD hbd ) );

// Asynchronous communications.
API( GWERR,  	S26_ComClearFlags,					( HBD hbd, u8 LogDev, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComClose,						( HBD hbd, u8 LogDev, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComEndBreak,					( HBD hbd, u8 LogDev, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComFlush,						( HBD hbd, u8 LogDev, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComGetRxCount,					( HBD hbd, u8 LogDev, u16 *CharCount, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComGetTxCount,					( HBD hbd, u8 LogDev, u16 *CharCount, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComOpen,						( HBD hbd, u8 LogDev, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComReceive,						( HBD hbd, u8 LogDev, u8 *MsgBuf, u16 *MsgLen, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComSend,						( HBD hbd, u8 LogDev, u8 *MsgBuf, u16 MsgLen, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComSetMode,						( HBD hbd, u8 LogDev, u16 ClockDiv, u8 Attributes, u8 LEDs, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComSetBreakChar,				( HBD hbd, u8 LogDev, u8 BreakChar, u32 msec, u32 retries ) );
API( GWERR,  	S26_ComStartBreak,					( HBD hbd, u8 LogDev, u32 msec, u32 retries ) );

// Gateway transaction scheduling --------------------------------

// Transaction control.
API( HXACT, 	S26_SchedOpen,						( HBD hbd, u32 retries ) );
API( GWERR, 	S26_SchedExecute,					( HXACT x, u32 msec, u8 *IomStatus ) );
API( GWERR, 	S26_SchedExecuteStart,				( HXACT x ) );
API( int, 		S26_SchedExecuteIsResponded,		( HXACT x, u32 msec  ) );
API( GWERR, 	S26_SchedExecuteWait,				( HXACT x, u32 msec ) );
API( GWERR, 	S26_SchedExecuteFinish,				( HXACT x, u8 *IomStatus ) );

// General-purpose IOM transactions.
API( GWERR,		S26_Sched2600_Nop,					( HXACT x, IOMPORT IomPort ) );
API( GWERR,		S26_Sched2600_ClearStatus,			( HXACT x, IOMPORT IomPort, u8 BitMask ) );
API( GWERR, 	S26_Sched2600_IomGetProductID,		( HXACT x, IOMPORT IomPort, u16 *ProductID ) );
API( GWERR, 	S26_Sched2600_GetFirmwareVersion,	( HXACT x, IOMPORT IomPort, u16 *Version ) );
API( GWERR, 	S26_Sched2600_GetAddress,			( HXACT x, IOMPORT IomPort, u8 *adrs ) );

// MM.
API( GWERR, 	S26_Sched2601_GetLinkStatus,		( HXACT x, u16 *LinkFlags ) );
API( GWERR, 	S26_Sched2601_GetInterlocks,		( HXACT x, u8 *LockFlags ) );
API( GWERR, 	S26_Sched2601_SetWatchdog,			( HXACT x, u8 NumTenthSeconds ) );

// 2608 analog I/O.
API( DOUBLE,	S26_2608_AdcCorrect,				( HBD hbd, IOMPORT IomPort, s16 RawData, BOOL IsHighGain ) );
API( GWERR, 	S26_2608_WriteEeprom,				( HBD hbd, IOMPORT IomPort, u32 msec, u8 address, u8 value, u32 retries ) );
API( GWERR, 	S26_Sched2608_GetAins,				( HXACT x, IOMPORT IomPort, DOUBLE *volts, BOOL Integrated ) );
API( GWERR, 	S26_Sched2608_GetAinTypes,			( HXACT x, IOMPORT IomPort, u8 *types ) );
API( GWERR, 	S26_Sched2608_GetAout,				( HXACT x, IOMPORT IomPort, u8 chan, DOUBLE *volts ) );
API( GWERR, 	S26_Sched2608_GetCalData,			( HXACT x, IOMPORT IomPort, s16 *caldata ) );
API( GWERR, 	S26_Sched2608_ReadEeprom,			( HXACT x, IOMPORT IomPort, u8 address, u8 *value ) );
API( GWERR, 	S26_Sched2608_SetAinTypes,			( HXACT x, IOMPORT IomPort, const u8 *types ) );
API( GWERR, 	S26_Sched2608_SetAout,				( HXACT x, IOMPORT IomPort, u8 chan, DOUBLE volts ) );
API( GWERR, 	S26_Sched2608_SetTempUnits,			( HXACT x, IOMPORT IomPort, int DegreesF ) );
API( GWERR, 	S26_Sched2608_SetLineFreq,			( HXACT x, IOMPORT IomPort, u8 freq ) );

// 2610 digital I/O.
API( GWERR, 	S26_Sched2610_GetInputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR, 	S26_Sched2610_GetModes,				( HXACT x, IOMPORT IomPort, u8 *modes ) );
API( GWERR, 	S26_Sched2610_GetModes32,			( HXACT x, IOMPORT IomPort, u8 *modes ) );
API( GWERR, 	S26_Sched2610_GetOutputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR, 	S26_Sched2610_GetPwmRatio,			( HXACT x, IOMPORT IomPort, u8 chan, u8 *OnTime, u8 *OffTime ) );
API( GWERR, 	S26_Sched2610_SetModes,				( HXACT x, IOMPORT IomPort, u8 *modes ) );
API( GWERR, 	S26_Sched2610_SetModes32,			( HXACT x, IOMPORT IomPort, u8 *modes ) );
API( GWERR, 	S26_Sched2610_SetOutputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR, 	S26_Sched2610_SetPwmRatio,			( HXACT x, IOMPORT IomPort, u8 chan, u8 OnTime, u8 OffTime ) );

// 2612 analog input.
API( GWERR, 	S26_Sched2612_SetMode,				( HXACT x, IOMPORT IomPort, u8 channel, u32 mode ) );
API( GWERR, 	S26_Sched2612_SetVoltages,			( HXACT x, IOMPORT IomPort, u8 volts ) );
API( GWERR, 	S26_Sched2612_GetValues,			( HXACT x, IOMPORT IomPort, s32 *values, u8 *tstamps ) );
API( GWERR, 	S26_Sched2612_RefreshData,			( HXACT x, IOMPORT IomPort ) );
API( GWERR, 	S26_2612_RegisterZero,				( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan, u32 nsmp ) );
API( GWERR, 	S26_2612_RegisterSpan,				( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan, u32 nsmp, DOUBLE load ) );
API( GWERR, 	S26_2612_RegisterTare,				( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan, u32 nsmp ) );
API( GWERR, 	S26_2612_SaveCalibrations,			( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan ) );
API( GWERR, 	S26_2612_RestoreCalibrations,		( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan ) );
API( DOUBLE,	S26_2612_GetCalibratedValue,		( HBD hbd, IOMPORT IomPort, u8 chan, u32 *sample ) );
API( DOUBLE,	S26_2612_GetOffset,					( HBD hbd, IOMPORT IomPort, u8 chan ) );
API( DOUBLE,	S26_2612_GetScale,					( HBD hbd, IOMPORT IomPort, u8 chan ) );
API( DOUBLE,	S26_2612_GetTare,					( HBD hbd, IOMPORT IomPort, u8 chan ) );
API( GWERR, 	S26_2612_SetCalibrations,			( HBD hbd, IOMPORT IomPort, u8 chan, DOUBLE Offset, DOUBLE Scale, DOUBLE Tare ) );

// 2620 counter.
API( GWERR, 	S26_Sched2620_GetCommonControl,		( HXACT x, IOMPORT IomPort, u16 *period, u8 *tstamp ) );
API( GWERR, 	S26_Sched2620_GetCounts,			( HXACT x, IOMPORT IomPort, u8 chan, u32 *value, u16 *tstamp ) );
API( GWERR, 	S26_Sched2620_GetMode,				( HXACT x, IOMPORT IomPort, u8 chan, u16 *mode ) );
API( GWERR, 	S26_Sched2620_GetPreload,			( HXACT x, IOMPORT IomPort, u8 chan, u8 reg, u32 *value ) );
API( GWERR, 	S26_Sched2620_GetStatus,			( HXACT x, IOMPORT IomPort, u8 chan, u16 *status ) );
API( GWERR, 	S26_Sched2620_SetCommonControl,		( HXACT x, IOMPORT IomPort, u16 gperiod, u8 tstamp ) );
API( GWERR, 	S26_Sched2620_SetControlReg,		( HXACT x, IOMPORT IomPort, u8 chan, u8 DataVal ) );
API( GWERR, 	S26_Sched2620_SetMode,				( HXACT x, IOMPORT IomPort, u8 chan, u16 mode ) );
API( GWERR, 	S26_Sched2620_SetModeEncoder,		( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowX, u16 HardPreload, u16 ClkMode ) );
API( GWERR, 	S26_Sched2620_SetModeFreqMeas,		( HXACT x, IOMPORT IomPort, u8 chan, u16 InternalGate ) );
API( GWERR, 	S26_Sched2620_SetModePeriodMeas,	( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowX ) );
API( GWERR, 	S26_Sched2620_SetModePulseGen,		( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowX, u16 HardTrig, u16 ActLowOut ) );
API( GWERR, 	S26_Sched2620_SetModePulseMeas,		( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowX ) );
API( GWERR, 	S26_Sched2620_SetModePwmGen,		( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowOut ) );
API( GWERR, 	S26_Sched2620_SetPreload,			( HXACT x, IOMPORT IomPort, u8 chan, u8 reg, u32 value ) );

// 2631 motor controller.
API( GWERR, 	S26_Sched2631_SetControlReg,		( HXACT x, IOMPORT IomPort, u8 DataVal ) );
API( GWERR, 	S26_Sched2631_GetStatus,			( HXACT x, IOMPORT IomPort, u16 *status ) );
API( GWERR, 	S26_Sched2631_SetPreload,			( HXACT x, IOMPORT IomPort, u8 reg, u32 value ) );
API( GWERR, 	S26_Sched2631_GetPreload,			( HXACT x, IOMPORT IomPort, u8 reg, u32 *value ) );
API( GWERR, 	S26_Sched2631_GetCounts,			( HXACT x, IOMPORT IomPort, u32 *value, u16 *tstamp ) );
API( GWERR, 	S26_Sched2631_SetCommonControl,		( HXACT x, IOMPORT IomPort, u16 period, u8 tstamp ) );
API( GWERR, 	S26_Sched2631_GetCommonControl,		( HXACT x, IOMPORT IomPort, u16 *period, u8 *tstamp ) );
API( GWERR, 	S26_Sched2631_SetMode,				( HXACT x, IOMPORT IomPort, u16 mode ) );
API( GWERR, 	S26_Sched2631_GetMode,				( HXACT x, IOMPORT IomPort, u16 *mode ) );
API( GWERR, 	S26_Sched2631_SetModeEncoder,		( HXACT x, IOMPORT IomPort, u16 ActLowX, u16 HardPreload, u16 ClkMode ) );
API( GWERR, 	S26_Sched2631_SetModePulseGen,		( HXACT x, IOMPORT IomPort, u16 ActLowX, u16 HardTrig, u16 ActLowOut ) );
API( GWERR, 	S26_Sched2631_SetModePwmGen,		( HXACT x, IOMPORT IomPort, u16 ActLowOut ) );
API( GWERR, 	S26_Sched2631_SetModePulseMeas,		( HXACT x, IOMPORT IomPort, u16 ActLowX ) );
API( GWERR, 	S26_Sched2631_SetModePeriodMeas,	( HXACT x, IOMPORT IomPort, u16 ActLowX ) );
API( GWERR, 	S26_Sched2631_SetModeFreqMeas,		( HXACT x, IOMPORT IomPort, u16 igate ) );
API( GWERR, 	S26_Sched2631_OutEnable,			( HXACT x, IOMPORT IomPort, u8 enable ) );
API( GWERR, 	S26_Sched2631_SetOutMode,			( HXACT x, IOMPORT IomPort, u8 mode ) );
API( GWERR, 	S26_Sched2631_SetAmpl,				( HXACT x, IOMPORT IomPort, u32 amplitude ) );
API( GWERR, 	S26_Sched2631_SetOutputFrq,			( HXACT x, IOMPORT IomPort, u32 frequency ) );
API( GWERR, 	S26_Sched2631_SetPWMFrq,			( HXACT x, IOMPORT IomPort, u32 frequency ) );

// 2650 relay.
API( GWERR, 	S26_Sched2650_GetInputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR, 	S26_Sched2650_GetOutputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR, 	S26_Sched2650_SetOutputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );

// 2652 relay.
API( GWERR, 	S26_Sched2652_GetInputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR, 	S26_Sched2652_GetOutputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR, 	S26_Sched2652_SetOutputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR, 	S26_Sched2652_GetModes,				( HXACT x, IOMPORT IomPort, u8 *modes ) );
API( GWERR, 	S26_Sched2652_GetPwmRatio,			( HXACT x, IOMPORT IomPort, u8 chan, u8 *OnTime, u8 *OffTime ) );
API( GWERR, 	S26_Sched2652_SetModes,				( HXACT x, IOMPORT IomPort, u8 *modes ) );
API( GWERR,		S26_Sched2652_SetPwmRatio,			( HXACT x, IOMPORT IomPort, u8 chan, u8 OnTime, u8 OffTime ) );

// 2653 relay.
API( GWERR,		S26_Sched2653_SetModes,				( HXACT x, IOMPORT IomPort, u8 *modes ) );
API( GWERR,		S26_Sched2653_GetModes,				( HXACT x, IOMPORT IomPort, u8 *modes ) );
API( GWERR,		S26_Sched2653_SetPwmRatio,			( HXACT x, IOMPORT IomPort, u8 chan, u8 OnTime, u8 OffTime ) );
API( GWERR,		S26_Sched2653_GetPwmRatio,			( HXACT x, IOMPORT IomPort, u8 chan, u8 *OnTime, u8 *OffTime ) );
API( GWERR,		S26_Sched2653_GetInputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR,		S26_Sched2653_GetOutputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );
API( GWERR,		S26_Sched2653_SetOutputs,			( HXACT x, IOMPORT IomPort, u8 *states ) );

// VB utility functions.
API( u32 *,		S26_CastAsLong,						( void *pval ) );
API( u16 *,		S26_CastAsShort,					( void *pval ) );
API( void,		S26_CopyAny,						( void *dst, const void *src, int nbytes ) );
API( u32,		S26_Bitmask,						( u32 bitnum ) );
