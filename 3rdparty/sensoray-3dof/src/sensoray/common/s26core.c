/******************************************************************************************
Module    : s26core.c
Function  : Platform-independent middleware core for Sensoray 2600 system.
Author    : Jim Lamberson
Copyright : Copyright (C) 2008 Sensoray.

One of these preprocessor definitions must be declared to specify cpu endianness:
ENDIAN_LITTLE, ENDIAN_BIG

This code module:
  *	Constitutes the bulk of a comprehensive API for Sensoray's 2600 family.
  *	Encapsulates all of the communication protocols required to communicate with
	a Sensoray 2600 I/O system. Public functions are exposed directly to application
	programs as API calls.
  *	Is designed to work with any operating system or cpu. There are no
	references to any particular operating system (e.g., "#ifdef _WINDOZE_"). PLEASE
	KEEP IT THAT WAY! Don't pollute this file with any OS or CPU dependent code.
  *	Depends on OS-specific code that must be supplied by these external files:
		s26app.h	- Declarations shared by app/mod/core.	
		s26mod.h	- Declarations shared by mod/core.
		s26mod.c	- System interface functions needed by the core. 
	Your project must include these files, either by setting the include path or by
	collecting all necessary files together in one location. See the linux project
	for clues about implementing these files for other operating systems.

If you plan to develop your own API:
  *	Don't modify any files except the three 0S-specific source files mentioned above.
	The core and other OS-independent source files are designed to work with any operating
	system and with any cpu, WITHOUT MODIFICATION.
  *	Good luck! Sensoray has provided you with the foundation code (including a complete
	linux implementation) but Sensoray DOES NOT	PROVIDE SUPPORT for customization of the code.
	We will be happy to quote a fee for porting the API to another operating system or cpu.
*******************************************************************************************/


#define VERSION_STRING		"1.0.12"		// Middleware version string. This is changed by Sensoray for each middleware release.

// Other declarations.
#include <stddef.h>
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>

#define S26_BUILDING_MOD					// Configure include files for building the middleware core.

#include "s26app.h"							// OS-dependent defines used by app/mod/core.
#include "s26mod.h"							// OS-dependent defines used by mod/core.
#include "s26modf.h"						// Generic defines used by mod/core.

#define LOGERR(ERR)							// If desired, define error logger here.


///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////  CONSTANTS  ////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


#define TRUE					1
#define FALSE					0

#define PVOID					0						// Void pointer.
#define ZERO_SIZE				0						// Zero data size.

// Build constants.
#define MAX_MCMDS				100						// Max number of MCmd's allowed in a gateway command packet.
#define MAX_TRANSACTS			8						// Max number of concurrent transactions allowed for each MM.

// Physical resource limits.
#define NUM_IOMPORTS			16						// Number of iom ports on each MM.
#define NUM_LOGDEVS				5						// Number of logical devices on a MM (1 gateway, 4 comports).

// MM communication parameters.
#define MRSP_HEADER_SIZE		3						// Fixed size of MRsp header.
#define MCMD_HEADER_SIZE		2						// Fixed size of MCmd header.
#define ACT_HEADER_SIZE			1						// Fixed size of Action header, which is just the action opcode.
#define MAX_MCMDSIZE			254						// Maximum size of an MCmd, including its header.
#define MAX_PACKETSIZE			1440					// Maximum legal UDP payload size.
#define MAX_MMPAYLOAD_SIZE		1024					// Maximum UDP payload size supported by the MM.
#define MAX_PPSTEPS				( MAX_MCMDS * 2 )		// Max number of post-processing steps per gateway command packet.

// Address offsets of values within a MRsp.
#define MRSP_POSN_IOMPORT		0						// Iom port number that the MRsp was received on.
#define MRSP_POSN_MRSPLEN		1						// MRsp's byte count.
#define MRSP_POSN_IOMSTATUS		2						// Iom status byte.

// MM retry parameters.
#define SEQNUM_MIN				1						// Minimum valid sequence number.
#define SEQNUM_MAX				6						// Maximum valid sequence number.
#define SEQNUM_SHIFT			4						// Shift left the 3-bit transaction sequence number by this amount.
#define SEQNUM_MASK				(0x07 << SEQNUM_SHIFT)	// Sequence number mask, applied to first byte of gateway or comport command packet.

// Network parameters.
#define IPPORT_GATEWAY			10000					// Gateway IP port.
#define IPPORT_COM1				( IPPORT_GATEWAY + 1 )	// Serial COM1 IP port.
#define IPPORT_COM2				( IPPORT_GATEWAY + 2 )	// Serial COM2 IP port.
#define IPPORT_COM3				( IPPORT_GATEWAY + 3 )	// Serial COM3 IP port.
#define IPPORT_COM4				( IPPORT_GATEWAY + 4 )	// Serial COM4 IP port.

// Comport opcodes.
#define SIO_SETMODE				0x00					// Init baud rate, parity, etc.
#define SIO_SEND				0x01					// Transmit string.
#define SIO_RECEIVE				0x02					// Receive string.
#define SIO_CLEARFLAGS			0x03					// Reset all error flags.
#define SIO_OPEN				0x04					// Open ComPort.
#define SIO_CLOSE				0x05					// Close ComPort.
#define SIO_FLUSH				0x06					// Flush receive buffer.
#define SIO_GETRXCOUNT			0x07					// Return receive buffer character count.
#define SIO_GETTXCOUNT			0x08					// Return transmit buffer character count.
#define SIO_STARTBREAK			0x09					// Start break transmission.
#define SIO_ENDBREAK			0x0A					// Terminate break transmission.
#define SIO_SETBREAKCHAR		0x0B					// Declare the Break character.

// Iom status bits.										// Masks:
#define STATUS_NONE				0x00					//   No status bits are set.
#define STATUS_PHYSICAL			0xE7					//   Physical status flags that are asserted by the iom.
#define STATUS_SIMULATED		0x18					//   Middleware-simulated iom status flags.
#define STATUS_TYPESPECIFIC		0x1F					//   All iom type-specific flags, including both physical and simulated.
														// Physical iom status flags that are not manipulated by client:
#define STATUS_HRST				0x20					//   Module reset flag, gateway resettable.

// Iom status bit masks.
#define STATMASK_NONE			( ~STATUS_HRST )		// Pass through all status bits except HRST, which is only used the gateway, internally.
#define STATMASK_2650			( ~(STATUS_HRST | 3) )	// Pass through all except iom-specific, which are not reliable indicators of problems.
#define STATMASK_2652			( ~(STATUS_HRST | 3) )	// Pass through all except iom-specific, which are not reliable indicators of problems.
#define STATMASK_2653			( ~(STATUS_HRST | 3) )	// Pass through all except iom-specific, which are not reliable indicators of problems.

// Special ResponseLength indicator for Reset actions.
#define KILL_MRSP				0xFF					// Set RspLen to this to indicate action kills MRsp.

// Invalid handles.
#define INVALID_XACT			0						// Bad gateway transaction handle, due to memory allocation problem.

// Iom types.
#define IOMTYPE_UNREG			0						// Unregistered iom.

// Sizes of iom response buffers for supported iom types.
#define IOM_RSPBUF_SIZE_2601	250						// MM.
#define IOM_RSPBUF_SIZE_2608	38						// 2608.
#define IOM_RSPBUF_SIZE_2610	10						// 2610.
#define IOM_RSPBUF_SIZE_2612	26						// 2612.
#define IOM_RSPBUF_SIZE_2620	20						// 2620.
#define IOM_RSPBUF_SIZE_2631	20						// 2631.
#define IOM_RSPBUF_SIZE_2650	11						// 2650.
#define IOM_RSPBUF_SIZE_2652	11						// 2652.
#define IOM_RSPBUF_SIZE_2653	11						// 2653.
#define IOM_RSPBUF_SIZE_ANY		10						// Any iom.  This must be the size of the smallest buffer for all iom types.

// Sizes of iom command buffers for supported iom types.
#define IOM_CMDBUF_SIZE_2601	250						// MM.
#define IOM_CMDBUF_SIZE_2608	20						// 2608.
#define IOM_CMDBUF_SIZE_2610	18						// 2610.
#define IOM_CMDBUF_SIZE_2612	20						// 2612.
#define IOM_CMDBUF_SIZE_2620	20						// 2620.
#define IOM_CMDBUF_SIZE_2631	20						// 2631.
#define IOM_CMDBUF_SIZE_2650	13						// 2650.
#define IOM_CMDBUF_SIZE_2652	13						// 2652.
#define IOM_CMDBUF_SIZE_2653	14						// 2653.
#define IOM_CMDBUF_SIZE_ANY		13						// Any iom.  This must be the size of the smallest buffer for all iom types.

// Response argument specification byte: ( IsLittleEndian, ArgSize<2:0>, ArgCount<3:0> ).
#define MASK_LITTLEENDIANFLAG	0x8000
#define MASK_ARGCOUNT			0x00FF
#define MASK_ARGSIZE			0x7F00
#define SHFT_ARGSIZE			8
#define ASPEC(ETYPE,ATYPE,ACNT)	( (u16)( ETYPE | ( sizeof(ATYPE) << SHFT_ARGSIZE ) | ACNT ) )
#define LE(ARGTYPE,ARGCOUNT)	ASPEC( MASK_LITTLEENDIANFLAG, ARGTYPE, ARGCOUNT )
#define BE(ARGTYPE,ARGCOUNT)	ASPEC( 0, ARGTYPE, ARGCOUNT )
#define NE(ARGTYPE,ARGCOUNT)	ASPEC( 0, ARGTYPE, ARGCOUNT )

// Action opcodes -----------------------------------------------

// Opcodes specific to the 2601 iom gateway.
#define OP_GETLINKSTATUS		0x00					// Get Active Port List (APL).
#define OP_GETINTERLOCKS		0x01					// Get status of interlock contacts.

// Opcodes common to all module types.
#define OP_SOFTRESET			0xF0					// Invoke soft reset.
#define OP_HARDRESET			0xF1					// Invoke hard reset.
#define OP_RESETFLAGS			0xF2					// Clear specified status flag bits.
#define OP_SETWATCHDOG			0xF3					// Set communication watchdog interval.
#define OP_GETPRODUCTID			0xF5					// Get module identifier.
#define OP_GETVERSION			0xF6					// Get module's firmware version number.
#define OP_GETADDRESS			0xF7					// Return module's address shunt settings, if present.
#define OP_NOP					0xFF					// No operation.

// Opcodes common to all IOMs.
#define OP_GETADDRESS			0xF7					// Get address shunt settings.
#define OP_LINKQUERY			0xFE					// Query iom port to determine if module is present.

// Opcodes specific to model 2608 analog I/O module.
#define OP_AIO_SETOUTPUT		0x00					// Program dac setpoint.
#define OP_AIO_GETOUTPUT		0x01					// Get programmed dac setpoint.
#define OP_AIO_SETINPUTRANGES	0x02					// Set all adc gain flags.
#define OP_AIO_GETINPUTRANGES	0x03					// Get all adc gain flags.
#define OP_AIO_GETSNAPSHOTS		0x04					// Get all adc snapshot values.
#define OP_AIO_GETINTEGRATEDS	0x05					// Get all adc integrated values.
#define OP_AIO_LOCKCHANNEL		0x0A					// UNIMPLEMENTED - halt on specified channel.
#define OP_AIO_GETINTERNALS		0x0B					// Get all digitized references.
#define OP_AIO_SETFSTANDARD		0x0C					// Declare line frequency.
#define OP_AIO_READEPROM		0x0D					// Get eeprom value.
#define OP_AIO_WRITEEPROM		0x0E					// Program eeprom value.

// Opcodes specific to model 2610 48-channel DIO.
#define OP_DIO_SETMODES			0x00					// Set operating modes for channels 0-7.
#define OP_DIO_GETMODES			0x01					// Get operating modes for channels 0-7.
#define OP_DIO_SETPWMRATIO		0x02					// Set a channel's PWM ratio and period.
#define OP_DIO_GETPWMRATIO		0x03					// Get a channel's PWM ratio and period.
#define OP_DIO_GETINPUTS		0x04					// Get physical states of all channels.
#define OP_DIO_GETOUTPUTS		0x05					// Get programmed output states of all channels.
#define OP_DIO_SETOUTPUTS		0x06					// Set programmed output states of all channels.
#define OP_DIO_SETMODES32		0x07					// Set operating modes for channels 0-31. Valid only on firmware version 1.02 and higher.
#define OP_DIO_GETMODES32		0x08					// Get operating modes for channels 0-31. Valid only on firmware version 1.02 and higher.

// Opcodes specific to model 2612 analog input module.
#define OP_AI_SETMODE		    0x00					// Program ADC mode.
#define OP_AI_GETINPUTS		    0x01					// Return ADC values.
#define OP_AI_SETVOLTAGES	    0x02					// Program voltages for all channels.
#define OP_AI_READEEPROM 	    0x0D					// Get eeprom value.
#define OP_AI_WRITEEPROM		0x0E					// Program eeprom value.

// Opcodes specific to model 2620 4-channel counter.  Must be or'ed with (chanNum << 4).
#define OP_CNT_SOFTTRIGGER		0x00					// Generate soft trigger.
#define OP_CNT_SETMODE			0x01					// Write to counter mode register.
#define OP_CNT_GETSTATUS		0x02					// Read counter status register.
#define OP_CNT_SETPRELOAD0		0x03					// Write 32-bit value to preload 0.
#define OP_CNT_SETPRELOAD1		0x04					// Write 32-bit value to preload 1.
#define OP_CNT_GETCOUNTS		0x05					// Fetch core or latched counts, depending on counter operating mode.
#define OP_CNT_GETCOUNTS_TS		0x06					// Fetch core or latched counts plus time stamp, depending on counter operating mode.
#define OP_CNT_SETCOMMONCONTROL	0x0F					// Write 16-bit value to common control register.

// Opcodes specific to model 2631 motor controller.
#define OP_MC_SOFTTRIGGER		0x00					// Generate soft trigger.
#define OP_MC_SETMODE			0x01					// Write to counter mode register.
#define OP_MC_GETSTATUS         0x02					// Read counter status register.
#define OP_MC_SETPRELOAD0		0x03					// Write 32-bit value to preload 0.
#define OP_MC_SETPRELOAD1		0x04					// Write 32-bit value to preload 1.
#define OP_MC_GETCOUNTS         0x05					// Fetch core or latched counts, depending on counter operating mode.
#define OP_MC_GETCOUNTS_TS		0x06					// Fetch core or latched counts plus time stamp, depending on counter operating mode.
#define OP_MC_SETCOMMONCONTROL  0x0F					// Write 16-bit value to common control register.

#define OP_MC_OUTENABLE         0x10					// Enable output.
#define OP_MC_OUTDISABLE        0x11					// Disable output.
#define OP_MC_SETOUTMODE        0x12					// Write 8-bit value to output mode register.
#define OP_MC_SETAMPLITUDE      0x13					// Write 8-bit value to output amplitude register.
#define OP_MC_SETOUTFRQ         0x14					// Write output frequency value.
#define OP_MC_SETPWMFRQ  		0x15					// Write PWM frequency value.

// Opcodes specific to models 2650/2652/2653 relay racks.
#define OP_RLY_GETINPUTS		0x00					// Get physical states of all relay coil drivers.
#define OP_RLY_GETOUTPUTS		0x01					// Get programmed states of all relay coil drivers.
#define OP_RLY_SETOUTPUTS		0x02					// Set states of all relay coil drivers.

// Opcodes specific to model 2652/2653 relay racks.
#define OP_RLY_SETMODES			0x03					// Set operating modes for all 7 channels.
#define OP_RLY_GETMODES			0x04					// Get operating modes for all 7 channels.
#define OP_RLY_SETPWMRATIO		0x05					// Set a channel's PWM ratio and period.
#define OP_RLY_GETPWMRATIO		0x06					// Get a channel's PWM ratio and period.

// Model 2608 constants -----------------------------------------------

// Configuration constants.
#define ADC_SCALAR_TOL			0.02					// Maximum adc reference tolerance (e.g., 0.02 = 2 percent).
#define DAC_SCALAR_TOL			0.1						// Maximum dac scalar deviation from 1.0 (e.g., 0.1 = 10 percent).
#define DAC_OFFSET_TOL			40						// Maximum dac raw data offset deviation from 0x0000.
#define TREF_OFFSET_TOL			275						// Maximum temp ref raw data offset deviation from 0x0000.
#define MIN_CLB					32000					// Minimum adc counts permitted for full-scale reference standards.
#define ANA_SCALAR_NORM			1000000					// Normalization scalar; enables use of integer calibration constants.

// Channel capacity.
#define NUM_AINS				16						// Number of analog input channels.

// Digitized values for internal references are stored in this standardized order:
#define INDEX_REF_LG_0V			0						// Low gain, 0 Volt standard.
#define INDEX_REF_LG_10V		1						// Low gain, 10 Volt standard.
#define INDEX_REF_HG_0V			2						// High gain, 0 Volt standard.
#define INDEX_REF_HG_100MV		3						// High gain, 100 mV standard.
#define INDEX_REF_T0			4						// Junction temperature sensor 0.  >= v3.00: for adc chans 0 to 1, < v3.00: for adc chans 0 to 7.
#define INDEX_REF_T1			5						// Junction temperature sensor 1.  >= v3.00: for adc chans 2 to 3, < v3.00: for adc chans 8 to 15.
#define INDEX_REF_T2			6						// Junction temperature sensor 2.  >= v3.00: for adc chans 4 to 5, < v3.00: not used.
#define INDEX_REF_T3			7						// Junction temperature sensor 3.  >= v3.00: for adc chans 6 to 7, < v3.00: not used.
#define INDEX_REF_T4			8						// Junction temperature sensor 4.  >= v3.00: for adc chans 8 to 9, < v3.00: not used.
#define INDEX_REF_T5			9						// Junction temperature sensor 5.  >= v3.00: for adc chans 10 to 11, < v3.00: not used.
#define INDEX_REF_T6			10						// Junction temperature sensor 6.  >= v3.00: for adc chans 12 to 13, < v3.00: not used.
#define INDEX_REF_T7			11						// Junction temperature sensor 7.  >= v3.00: for adc chans 14 to 15, < v3.00: not used.

// Number of internal reference standards returned by OP_AIO_GETINTERNALS.
#define MAX_REFS				12						// Max number of values returned by OP_AIO_GETINTERNALS for any firmware version.
#define MAX_NUM_TREFS			8						// Max number of onboard temperature sensors for any firmware version.
#define NUM_REFS_PRE_V300		8						// Number of values returned by OP_AIO_GETINTERNALS for firmware versions below 3.00.
#define NUM_REFS_POST_V300		MAX_REFS				// Number of values returned by OP_AIO_GETINTERNALS for firmware versions 3.00 and higher.

// EEPROM address offsets.  All multi-byte EEPROM values (e.g., shorts and longs) are stored in EEPROM in little-endian order.
#define EEADRS_NUMDACS			0						// u8: Number of populated dacs: 0, 4 or 8.
#define EEADRS_STD10V			12						// s32: 10V reference voltage (DOUBLE), times 1e6.
#define EEADRS_STD100MV			16						// s32: 100mV reference voltage, times 1e6.
#define EEADRS_DACOFFSET		20						// s16: Dac0 offset.  Other dac offsets spaced at address intervals of EESIZE_DACINFO bytes.
#define EEADRS_DACSCALAR		22						// s32: Dac0 scalar (DOUBLE), times 1e6.  Other dac scalars spaced at address intervals of EESIZE_DACINFO bytes.
#define EESIZE_DACINFO			6						// Number of eeprom bytes required for each dac offset/scalar pair.
#define EEADRS_TREFOFFSET		68						// s16's: List of 8 reference temperature sensor offsets, 1 per sensor.
#define EESIZE_TREFINFO			2						// Number of eeprom bytes required for each tref offset value.
#define EEADRS_CHECKSUM			84						// s8: Last byte: checksum of all eeprom bytes from address 0 through ( EEADRS_CHECKSUM - 1 ).

// Min/max ratios of EEPROM values:
// e.g., eeprom-stored 10V reference standard is invalid if it exceeds 10*ADC_SCALAR_MAX or is less than 10*ADC_SCALAR_MIN.
#define ADC_SCALAR_MIN			( (s32)( ( 1.0 - ADC_SCALAR_TOL ) * (DOUBLE)ANA_SCALAR_NORM ) )
#define ADC_SCALAR_MAX			( (s32)( ( 1.0 + ADC_SCALAR_TOL ) * (DOUBLE)ANA_SCALAR_NORM ) )
// Eeprom-stored dac scalar is invalid if it exceeds DAC_SCALAR_MAX or is less than DAC_SCALAR_MIN.
#define DAC_SCALAR_MIN			( (s32)( ( 1.0 - DAC_SCALAR_TOL ) * (DOUBLE)ANA_SCALAR_NORM ) )
#define DAC_SCALAR_MAX			( (s32)( ( 1.0 + DAC_SCALAR_TOL ) * (DOUBLE)ANA_SCALAR_NORM ) )

// Code-simplifying macros for eeprom values.
#define EE_NUMDACS				( image[EEADRS_NUMDACS] )
#define EE_10V					( *(s32 *)( &image[EEADRS_STD10V] ) )
#define EE_100MV				( *(s32 *)( &image[EEADRS_STD100MV] ) )
#define EE_DACOS(X)				( *(s16 *)( &image[EEADRS_DACOFFSET + EESIZE_DACINFO * X] ) )
#define EE_DACSC(X)				( *(s32 *)( &image[EEADRS_DACSCALAR + EESIZE_DACINFO * X] ) )
#define EE_TREFOS(X)			( *(s16 *)( &image[EEADRS_TREFOFFSET + EESIZE_TREFINFO * X] ) )

// Model 2610 constants --------------------------------------------------------------------------------------

#define DIO_BYTECOUNT			6						// Enough bytes for 48 DIO channels, 1 bit per channel.

// Model 2612 constants --------------------------------------------------------------------------------------

#define SG_DEFAULT_SCALE        1.0		                // Scale ADC's range to -1...+1.

// Model 2620/2631 constants ---------------------------------------------------------------------------------

// Status bits associated with the OP_CNT_GETSTATUS action:
#define CNT_STATUS_NULL 		0x01					// Counter core contains zero counts.
#define CNT_STATUS_OVR			0x02					// Overflow detected (0xFFFFFFFF -> 0x00000000); reset by "read status" command
#define CNT_STATUS_UNR			0x04					// Underflow detected (0x00000000 -> 0xFFFFFFFF); reset by "read status" command
#define CNT_STATUS_EX32 		0x08					// Counter extension bit 32.
#define CNT_STATUS_LD			0x10					// Counter was loaded; reset by "read status" command
#define CNT_STATUS_RESET		0x20					// Counter was reset; reset by "read status" command
#define CNT_STATUS_LATCH		0x40					// Counter was latched; reset by "read latch" command
#define CNT_STATUS_QERR 		0x80					// Quadrature decoder error detected; reset by "read status" command

// Command bits associated with the OP_CNT_SOFTTRIGGER action:
#define CNT_TRIG_LOAD			0x01					// Transfer preload register to counter core.
#define CNT_TRIG_LATCH			0x02					// Transfer counter core and timestamp to data latch.

// Preload register identifiers.
#define CNT_REG_PRELOAD0		0						// Preload0 register.
#define CNT_REG_PRELOAD1		1						// Preload1 register.

// Counter channel's operating mode used by OP_CNT_SETMODE action:
#define CT_RUN_SHFT				15						// Channel:
#define CT_RUN_DISABLE			( 0 << CT_RUN_SHFT )	//   Disable.
#define CT_RUN_ENABLE			( 1 << CT_RUN_SHFT )	//   Enable.

#define CT_OM_SHFT				13						// Output pin mode:
#define CT_OM_BIT31				( 0 << CT_OM_SHFT )		//   Counter bit 31.
#define CT_OM_ZEROTOGGLE		( 1 << CT_OM_SHFT )		//   Counter bit 32 (toggles at zero counts).
#define CT_OM_ZEROACTIVE		( 2 << CT_OM_SHFT )		//   Active at zero counts.
#define CT_OM_ROLLOVER			( 3 << CT_OM_SHFT )		//   Active during rollover.
#define CT_OM_DONTCARE			( 0 << CT_OM_SHFT )

#define CT_XP_SHFT				12						// Index input polarity:
#define CT_XP_ACTHIGH			( 0 << CT_XP_SHFT )		//   Active high.
#define CT_XP_ACTLOW			( 1 << CT_XP_SHFT )		//   Active low.
#define CT_XP_DONTCARE			( 0 << CT_XP_SHFT )

#define CT_PL_SHFT				10						// Preload trigger:
#define CT_PL_SOFTONLY			( 0 << CT_PL_SHFT )		//   Soft only.
#define CT_PL_INDEX				( 1 << CT_PL_SHFT )		//   Soft or index leading edge.
#define CT_PL_ZERO				( 2 << CT_PL_SHFT )		//   Soft or zero counts reached.

#define CT_LAT_SHFT				9						// Latch trigger:
#define CT_LAT_SOFTONLY			( 0 << CT_LAT_SHFT )	//   Soft only.
#define CT_LAT_INDEX			( 1 << CT_LAT_SHFT )	//   Soft or index leading edge.

#define CT_CET_SHFT				8						// Count enable trigger:
#define CT_CET_CONFIG			( 0 << CT_CET_SHFT )	//   Upon configuration.
#define CT_CET_INDEX			( 1 << CT_CET_SHFT )	//   Upon index leading edge.

#define CT_OP_SHFT				7						// Output pin polarity:
#define CT_OP_ACTHIGH			( 0 << CT_OP_SHFT )		//   Active high.
#define CT_OP_ACTLOW			( 1 << CT_OP_SHFT )		//   Active low.
#define CT_OP_DONTCARE			( 0 << CT_OP_SHFT )

#define CT_M_SHFT				4						// Clock mode.
#define CT_M_QUADX1ARISE		( 0 << CT_M_SHFT )		//   Quadrature x1 clock on A rising.
#define CT_M_QUADX1AFALL		( 1 << CT_M_SHFT )		//   Quadrature x1 clock on A falling.
#define CT_M_QUADX2				( 2 << CT_M_SHFT )		//   Quadrature x2 clock on A both.
#define CT_M_QUADX4				( 3 << CT_M_SHFT )		//   Quadrature x4.
#define CT_M_MONOARISE			( 4 << CT_M_SHFT )		//   Mono clock on A rising.
#define CT_M_MONOAFALL			( 5 << CT_M_SHFT )		//   Mono clock on A falling.
#define CT_M_MONOABOTH			( 6 << CT_M_SHFT )		//   Mono clock on A both.
#define CT_M_INTERNAL			( 7 << CT_M_SHFT )		//   Internal 10 MHz, A gate, B direction.

#define CT_CD_SHFT				2						// Count disable trigger:
#define CT_CD_NEVER				( 0 << CT_CD_SHFT )		//   Never.
#define CT_CD_INDEX				( 1 << CT_CD_SHFT )		//   Upon index trailing edge (if enabled).
#define CT_CD_ZERO				( 2 << CT_CD_SHFT )		//   Upon zero counts reached.

#define CT_PLM_SHFT				1						// Preload register usage:
#define CT_PLM_SINGLE			( 0 << CT_PLM_SHFT )	//   Only preload 0.
#define CT_PLM_BOTH				( 1 << CT_PLM_SHFT )	//   Both preload 0 and 1.

														// Index source:
#define CT_XC_EXTERNAL			0						//   External index pin.
#define CT_XC_INTERNAL			1						//   Internal free-running gate generator.


///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////  TYPES  //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


// Simple types.
typedef u16					ARGSPEC;					// Response argument specification: ( IsLittleEndian, ArgSize<6:0>, ArgCount<7:0> ).

// Complex type forward declarations.
typedef struct MM_OBJ		MM_OBJ;						// MM object.
typedef struct IOM			IOM;						// Iom object.
typedef struct POSTPROC		POSTPROC;					// Postprocessing step.
typedef struct XACT			XACT;						// Transaction object.
typedef XACT				*HXACT;						// Pointer to (i.e., app handle to) transaction object.

// Define constants that are shared by core/app and functions that are exposed through the api.
#include "app2600.h"

// Function types.
typedef	void				(* PPFUNC)( POSTPROC * );	// Response postprocessing function.
typedef void				(* VFUNC)( IOM * );			// Virtual module initialization function.


// Structure and union declarations //////////////////////////////////////////////////////////////////////

typedef struct IOM_ATTR {								// CONSTANT MODULE ATTRIBUTES ------------------
	u16					IomType;						//   Registered module type (e.g., 2601, 2608, etc.).  Set to IOMTYPE_UNREG if iom is closed.
	VFUNC				IomReset;						//   Function to be called upon module reset.
	VFUNC				IomRegister;					//   Function to be called upon module registration.
	u8					IomCmdBufSize;					//   Maximum permitted module command size for this module type.
	u8					IomRspBufSize;					//   Maximum permitted module response size for this module type.
} IOM_ATTR;

struct POSTPROC {										// POST-PROCESSING STEP --------------------
	u16					SrcSize;						//   Byte count of unprocessed data source.
	u8					*pSrcData;						//   Pointer to unprocessed source data, if SrcData value is not used directly.
	u8					*pDstData;						//   Pointer to processed data destination buffer.
	PPFUNC				pPProcFn;						//   Pointer to post-processing callback function.
	const ARGSPEC		*pArgSpec;						//   Pointer to response arguments specifications.  Last spec is always 0.
	IOM					*iom;							//   Pointer to iom object associated with this step.
	XACT				*x;								//   Pointer to transaction object associated with this step.
};

typedef struct MCMD {									// MODULE COMMAND -----------------------------
	u8					ModID;							//   Iom port number for I/O modules, or 0xFF for main module.
	u8					MRspLen;						//   Expected size of MRsp.
	const IOM_ATTR		*pIomAttr;						//   Constant attributes of the module type.
	u8					IomCmdBufMargin;				//   Maximum remaining iom command size for this MCmd.
	u8					IomRspBufMargin;				//   Maximum remaining iom response size for this MCmd.
	BOOL				KillMRsp;						//   Indicates no MRsp is expected in response to MCmd.
	u8					StatMask;						//   Bit mask for iom's MRsp status byte.
	u8					*pMRspBase;						//   Pointer to MRsp base in packet response buffer.
} MCMD;

typedef struct MRSP {									// MODULE RESPONSE --------------------
	u8					ModID;							//   Source module identifier.
	u8					Length;							//   Length of module response, including header.
	u8					Status;							//   Source module status.
	u8					RspList[256];					//   Action responses.
} MRSP;

typedef struct DACREF {									// MODEL 2608 ANALOG OUTPUT CALIBRATION VALUE PAIR ---------------
	s16					std_offset;						//   Dac correction offset.
	DOUBLE				std_scalar;						//   Dac correction scalar.
} DACREF;

typedef struct IOM_2608 {								// MODEL 2608 CUSTOM ATTRIBUTES ---------------------
	u8					SensType[NUM_AINS];				//   Analog type declarations for the 16 external ain channels.
	u16					GainFlags;						//   High Gain flags, one bit per external ain channel.
	int					UseDegreesF;					//   Temperature units selector: 0 = degrees C, 1 = degrees F.
														//   Values that depend on firmware version number:
	int					NumTemps;						//     Number of onboard temperature sensors: 8 for v3.00 and higher, 2 if firmware < v3.00.
	int					NumRefs;						//     Number of word values returned by calibrate action.
	const int			*pRefIndex;						//     Pointer to reference standards ordering list: differences start at firmware v3.00.
														//   Individual board profile from 2608 eeprom:
	int					NumDacs;						//     Number of populated analog output channels: 0 | 4 | 8.
	DOUBLE				RefVolts_10v;					//     Exact value of 10V adc low-gain reference standard (in Volts).
	DOUBLE				RefVolts_100mv;					//     Exact value of 100mV adc high-gain reference standard (in Volts).
														//   Real-time standardization parameters:
	DOUBLE				RefTemp[NUM_AINS];				//     Equivalent junction temperatures, one per external ain channel.
	s16					RawRefData[MAX_REFS];			//     Raw digitized reference standards.
	DOUBLE				NormScalar10v;					//     Normalization factor for 10V range:   RawAdc * NormScalar is in range -1.0 to +1.0.
	DOUBLE				NormScalar100mv;				//     Normalization factor for 100mV range: RawAdc * NormScalar is in range -1.0 to +1.0.
	DACREF				dac_std[MAX_NUM_AOUTS];			//     DAC standardization offset/scalar pairs.
	s16					TRefOffset[MAX_NUM_TREFS];		//     Reference temperature calibration offsets, in raw adc counts.
} IOM_2608;

typedef struct IOM_2612 {								// MODEL 2612 CUSTOM ATTRIBUTES ---------------------
	u32 			    Sample[4];   					//   Sample number.
	u8  			    Tstamp[4];   					//   Tstamp of the last sample.
	u8  			    TstampOld[4];  					//   Tstamp of the sample before the last.
	s32 			    Value[4];   					//   Value in ADC's units.
	DOUBLE				Offset[4];     					//   Zero offset as ADC_value on zero load.
	DOUBLE			    Scale[4];   					//   Scale as users_value / ADC_value.
	DOUBLE			    Tare[4];       					//   Tare in user's units on tare load.
} IOM_2612;

typedef struct CH2620 {									// MODEL 2620 CHANNEL ATTRIBUTES IMAGE -------------------
	u16					Mode;							//   Operating mode control bits.
	s32					Preload[2];						//   Preload register images.
} CH2620;

typedef struct IOM_2620 {								// MODEL 2620 CUSTOM ATTRIBUTES ---------------------
	u16					CommonControl;					//   Image of Common Control register (gate divisor and time stamp resolution).
	CH2620				ChanAttr[4];					//   Image of channel settings.
} IOM_2620;

typedef struct CH2631IN {								// MODEL 2631 INPUT CHANNEL ATTRIBUTES IMAGE -------------------
	u16					CommonControl;					//   Image of Common Control register (gate divisor and time stamp resolution).
	u16					Mode;							//   Operating mode control bits.
	s32					Preload[2];						//   Preload register images.
} CH2631IN;

typedef struct CH2631OUT {								// MODEL 2631 OUTPUT CHANNEL ATTRIBUTES IMAGE -------------------
    u8                  outenable;                      //   Output enable.
    u8                  outmode;                        //   Output mode.
    u32                 outfreq;                        //   Output frequency.
    u16                 outampl;                        //   Output amplitude.
    u32                 pwmfreq;                        //   PWM frequency.
} CH2631OUT;

typedef struct IOM_2631 {								// MODEL 2631 CUSTOM ATTRIBUTES ---------------------
	u16					CommonControl;					//   Image of Common Control register (gate divisor and time stamp resolution).
	CH2631IN			ChanAttrIn; 					//   Image of input channel settings.
	CH2631OUT			ChanAttrOut; 					//   Image of output channel settings.
} IOM_2631;

typedef union IOM_SPECIFIC {							// TYPE-SPECIFIC STORAGE OVERLAY; ADD NEW TYPES AS REQUIRED ----------------------
	IOM_2608			Obj2608;						//   Model 2608 analog i/o.
	IOM_2612			Obj2612;						//   Model 2612 analog input.
	IOM_2620			Obj2620;						//   Model 2620 counter/timer.
	IOM_2631			Obj2631;						//   Model 2631 motor controller.
} IOM_SPECIFIC;

struct IOM {											// IOM OBJECT -------------------------------
	MM_OBJ				*mm;							//   Pointer to this iom's parent MM.
	const IOM_ATTR		*pIomAttr;						//   Constant attributes of the module type (model number, buffer sizes, etc.).
	IOMPORT				IomPort;						//   MM's iom port number to which this iom is attached.
	u16					FwVersion;						//   Iom's firmware version number.
	u32					MaxInitTime;					//   Max time, in msec, to allow for gateway transactions in IomReset().
	u32					MaxRetries;						//   Max retries to allow for gateway transactions in IomReset().
	u8					SimStatus;						//   Simulated iom status flags, generated by the middleware.
	IOM_SPECIFIC		Custom;							//   Type-specific storage overlay.
};

struct XACT {											// TRANSACTION OBJECT -------------------------
	GWERR				GWErrCode;						//   Last gateway error.  MUST BE FIRST STRUCTURE MEMBER!!!
	MM_OBJ				*mm;							//   Pointer to this transaction's affiliated MM.
	u8					LogDev;							//   Logical device number (0 = gateway, 1 to 4 = COM1 to COM4).
	S26_HSOCKET			hSock;							//   Handle to this transaction's private socket.
	BOOL				IsSent;							//   Command packet has been sent to the MM.
	u8					Cmd[MAX_PACKETSIZE];			//   Command packet payload buffer.

	u8					Rsp[MAX_PACKETSIZE];			//   Response packet payload buffer.
	u16					CmdLen;							//   Size of data in Cmd[].
														//   Retry control:
	BOOL				UseRetries;						//     MM's retry mechanism is enabled for this transaction.	
	u8					SeqNext;						//     Next available transaction sequence number.
	u8					SeqThis;						//     Sequence number for the current transaction.  Applicable only if UseRetries is true.
	int					RemainingRetries;				//     Remaining available retry count.  Applicable only if UseRetries is true.
	u32					TxTimestamp;					//     Time command was sent.
														//   Parameters that apply only to comport transactions:
	u16					RspLen;							//     Size of data in Rsp[].
														//   Parameters that apply only to gateway transactions:
	BOOL				KillRspPacket;					//     Response packet is not expected.
	u16					NumPProcs;						//     Number of post-processing steps for this transaction.
	POSTPROC			PProc[MAX_PPSTEPS];				//     List of post-processing steps for this transaction.
	u8					*pCmd;							//     NextWrite pointer for Cmd[].
	u8					*pRsp;							//     NextRead pointer for Rsp[].
	u16					NumMCmds;						//     Number of MCmds in command packet.
	u8					*pMCmdBase;						//     Pointer to base of active MCmd in Cmd[].
	MCMD				*pMCmd;							//     Pointer to active MCmd[] element.
	MCMD				MCmd[MAX_MCMDS];				//     Attributes of MCmds belonging to active gateway command packet.
	MRSP				*pMRsp[MAX_MCMDS];				//     Pointers to MRsp base locations in gateway response packet.
	IOMPORT				SelectedIom;					//     The iom port number whose mcmd is under construction.
	u8					IomStatus[NUM_IOMPORTS];		//     The returned iom status bytes.
	u8					MMStatus;						//     The returned mm status byte.
	u8					PPRspBuf[256];					//     Intermediate response scratchpad buffer shared by all postprocessing functions.
	u8					PPCmdBuf[512];					//     Stacked command arguments that are needed for postprocessing, shared by all pp functions.
	u8					*pPPCmdBuf;						//     NextWrite pointer for PPCmdBuf[];
};

typedef struct XACT_POOL {								// TRANSACTION OBJECT POOL ---------------------------
	XACT				ObjPool[MAX_TRANSACTS];			//   Pool of transaction objects.
	XACT				*FreeList[MAX_TRANSACTS];		//   List of free objects in the pool.
	int					FreeCount;						//   Number of free objects in the pool.
	int					TotalCount;						//   Total number of objects in the pool.
	S26_CRITICALSECTION	hCriticalSection;				//   Handle to critical section used for pool management.
} XACT_POOL;

struct MM_OBJ {											// MAIN MODULE OBJECT --------------------------
	BOOL				IsOpen;							//   MM object is open.
	IOM					IomObj[NUM_IOMPORTS];			//   Iom objects.
	int					IomCount;						//   Number of registered iom objects.
	u32					nordMMAdrs;						//   MM's IP address, in network byte order.
	XACT_POOL			XactPool;						//   Transaction object pool.
														//   Diagnostics info:
	u32					PacketCountSent;				//     Number of packets sent by this MM.
	u32					PacketCountRcvd;				//     Number of packets received by this MM.
	u32					RetryCount;						//     Number of transaction retry attempts.
};


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////  FORWARD DECLARATIONS  ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////


static void endianfix_pp( POSTPROC *p );
static void s2608_IomReset( IOM *iom );
static void s2612_IomReset( IOM *iom );
static void s2620_IomReset( IOM *iom );
static void s2631_IomReset( IOM *iom );
static void s2608_IomRegister( IOM *iom );
static void s2612_IomRegister( IOM *iom );
static void s2620_IomRegister( IOM *iom );
static void s2631_IomRegister( IOM *iom );
static void NullVFUNC( IOM *iom );
static GWERR RegisterAllIoms( MM_OBJ *mm, u32 msec, u16 *IomCount, u16 *IomTypeList, u8 *IomStatus, u32 retries );
static GWERR Sched2600_ClearStatus( HXACT x, IOMPORT IomPort, u8 BitMask );


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////  STORAGE  /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////


// MM objects.
static u32		MMCount = 0;		// Number of MM's in the system.
static MM_OBJ	*obj;				// Pointer to array of MM (2601) objects.

// Dummy transaction object used in cases where object allocation fails.
static GWERR	InvalidXact = GWERR_XACTALLOC;

// Request to simulate a transaction error.
//static GWERR	SimErr = GWERR_NONE;

// Mapping list:  LogicalDeviceNumber -> IpPortNumber.
static const u16 IPPortList[] = { IPPORT_GATEWAY, IPPORT_COM1, IPPORT_COM2, IPPORT_COM3, IPPORT_COM4 };

// Constant attributes for all module types.
static const IOM_ATTR IomAttr2601	= { 2601,			NullVFUNC,		NullVFUNC,			IOM_CMDBUF_SIZE_2601,	IOM_RSPBUF_SIZE_2601 };
static const IOM_ATTR IomAttr2608	= { 2608,			s2608_IomReset,	s2608_IomRegister,	IOM_CMDBUF_SIZE_2608,	IOM_RSPBUF_SIZE_2608 };
static const IOM_ATTR IomAttr2610	= { 2610,			NullVFUNC,		NullVFUNC,			IOM_CMDBUF_SIZE_2610,	IOM_RSPBUF_SIZE_2610 };
static const IOM_ATTR IomAttr2612	= { 2612,			s2612_IomReset,	s2612_IomRegister,  IOM_CMDBUF_SIZE_2612,	IOM_RSPBUF_SIZE_2612 };
static const IOM_ATTR IomAttr2620	= { 2620,			s2620_IomReset,	s2620_IomRegister,	IOM_CMDBUF_SIZE_2620,	IOM_RSPBUF_SIZE_2620 };
static const IOM_ATTR IomAttr2631	= { 2631,			s2631_IomReset,	s2631_IomRegister,	IOM_CMDBUF_SIZE_2631,	IOM_RSPBUF_SIZE_2631 };
static const IOM_ATTR IomAttr2650	= { 2650,			NullVFUNC,		NullVFUNC,			IOM_CMDBUF_SIZE_2650,	IOM_RSPBUF_SIZE_2650 };
static const IOM_ATTR IomAttr2652	= { 2652,			NullVFUNC,		NullVFUNC,			IOM_CMDBUF_SIZE_2652,	IOM_RSPBUF_SIZE_2652 };
static const IOM_ATTR IomAttr2653	= { 2653,			NullVFUNC,		NullVFUNC,			IOM_CMDBUF_SIZE_2653,	IOM_RSPBUF_SIZE_2653 };
static const IOM_ATTR IomAttrUnreg	= { IOMTYPE_UNREG,	NullVFUNC,		NullVFUNC,			IOM_CMDBUF_SIZE_ANY,	IOM_RSPBUF_SIZE_ANY  };		// Unregistered type.

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////  ENDIAN CONVERSION SUPPORT  ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

static u16 SwapBytes16( u16 val )
{
	u16 rtnval;
	*( (u8 *)&rtnval     ) = *( (u8 *)&val + 1 );
	*( (u8 *)&rtnval + 1 ) = *( (u8 *)&val     );
	return rtnval;
}

static u32 SwapBytes32( u32 val )
{
	u32 rtnval;
	*( (u8 *)&rtnval     ) = *( (u8 *)&val + 3 );
	*( (u8 *)&rtnval + 1 ) = *( (u8 *)&val + 2 );
	*( (u8 *)&rtnval + 2 ) = *( (u8 *)&val + 1 );
	*( (u8 *)&rtnval + 3 ) = *( (u8 *)&val     );
	return rtnval;
}


#if defined( ENDIAN_BIG )
 #define HtoLE_16( X )		SwapBytes16( X )
 #define HtoBE_16( X )		( X )
 #define LEtoH_16( X )		SwapBytes16( X )
 #define BEtoH_16( X )		( X )
 #define LEtoH_32( X )		SwapBytes32( (u32)(X) )
 #define HtoLE_32( X )		SwapBytes32( (u32)(X) )
 #define BEtoH_32( X )		( X )
 #define HtoBE_32( X )		( X )
#elif defined( ENDIAN_LITTLE )
 #define HtoLE_16( X )		( X )
 #define HtoBE_16( X )		SwapBytes16( X )
 #define LEtoH_16( X )		( X )
 #define BEtoH_16( X )		SwapBytes16( X )
 #define LEtoH_32( X )		( X )
 #define HtoLE_32( X )		( X )
 #define BEtoH_32( X )		SwapBytes32( (u32)(X) )
 #define HtoBE_32( X )		SwapBytes32( (u32)(X) )
#else
 #error Symbol ENDIAN_x (e.g., ENDIAN_BIG, ENDIAN_LITTLE) is undefined
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////  TRANSACTION OBJECT POOL MANAGEMENT  /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// Allocate a transaction object from the pool and initialize it.
// Returns NULL if there are no free objects in the pool.

static XACT *XactAlloc( MM_OBJ *mm, u8 LogDev, int MaxRetries )
{
	XACT_POOL	*xp = &mm->XactPool;
	XACT		*x;
	int			i;

	// Start critical section.
	S26MOD_CriticalSectionBegin( xp->hCriticalSection );

	// Allocate a transaction object from the pool.
	///s x = ( ( xp->FreeCount > 0 ) ? xp->FreeList[ --( xp->FreeCount ) ] : NULL );
	if ( xp->FreeCount <= 0 )
    {
        LOGERR( xp->FreeCount );
        x = NULL;
    }
    else
	    x = xp->FreeList[ --xp->FreeCount ];

	// End critical section.
	S26MOD_CriticalSectionEnd( xp->hCriticalSection );

	// If the object was successfully allocated ...
	if ( x != NULL )
	{
		// Init the transaction object's error flags.
		if ( !mm->IsOpen )
		{
            x->GWErrCode = GWERR_MMCLOSED;
            LOGERR( x->GWErrCode );
            return NULL;
        }
		x->GWErrCode = GWERR_NONE;

		// Init transaction object as required by both comport and gateway transactions.
		x->LogDev			= LogDev;				// Target device (0 = gateway, 1 to 4 = COM1 to COM4).
		x->IsSent			= FALSE;				// Command packet has not been sent to the MM.
		x->UseRetries		= ( MaxRetries > 0 );	// Indicate whether MM retry mechanism is enabled.
		x->RemainingRetries	= MaxRetries;			// Init the number of unused retries (applies only if UseRetries is true).

		// Init gateway-specific values if this is a gateway transaction.
		if ( LogDev == LOGDEV_GATEWAY )
		{
			x->SelectedIom		= NUM_IOMPORTS;		// No MCmd is being constructed for any iom.
			x->KillRspPacket	= FALSE;			// Rsp packet is expected.
			x->pMCmd			= x->MCmd;			// Pointer to MCmd[] under construction.
			x->pCmd				= x->Cmd;			// Cmd packet NextWrite pointer.
			x->pRsp				= x->Rsp;			// Rsp packet NextRead pointer.
			x->NumMCmds			= 0;				// Number of MCmds in cmd packet.
			x->NumPProcs		= 0;				// No post-processing steps yet.
			x->pPPCmdBuf		= x->PPCmdBuf;		// No command arguments have been stored in PPCmdBuf[].
			x->MMStatus			= STATUS_NONE;		// No MM errors (RST) have been detected.

			// Init all iom status info in case one or more ioms fail to respond or are not spoken to.
			for ( i = 0; i < NUM_IOMPORTS; i++ )
				x->IomStatus[i] = STATUS_NONE;
		}
	}

	// Return pointer to transaction object.
	return x;
}

/////////////////////////////////////////////////////////////
// Release a transaction object to its affiliated MM's pool.

static void XactFree( HXACT x )
{
	XACT_POOL *xp;

	// If the transaction handle is legal ...
	if ( ( x != NULL ) && ( x != (XACT *)InvalidXact ) )
	{
		// Cache pointer to MM's transaction pool.
		xp = &x->mm->XactPool;

		// Start critical section.
		S26MOD_CriticalSectionBegin( xp->hCriticalSection );


		// Release transaction object.
        if ( xp->FreeCount < MAX_TRANSACTS )            /// +/- limited
    		xp->FreeList[ ( xp->FreeCount )++ ] = x;

		// End critical section.
		S26MOD_CriticalSectionEnd( xp->hCriticalSection );
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////  GATEWAY UTILITY FUNCTIONS  /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Flush to Cmd[] the MCmd that is currently being constructed.
// This must not be called if gateway errors are pending.

static void FlushMCmdToPacket( HXACT x )
{
	// Compute the size of the MCmd that is under construction.
	u8 MCmdLen = (u8)( x->pCmd - x->pMCmdBase );

	// The current MCmd may be empty.  This can happen if (1) the MCmd's first action specifies an iom status mask
	// that differs from the MCmd's iom status mask, or (2) only postprocessing steps (i.e., no actions) have been
	// scheduled.  If no actions have been scheduled into the current MCmd ...
	if ( MCmdLen == MCMD_HEADER_SIZE )
	{
		// Drop the MCmd.
		x->pCmd = x->pMCmdBase;				// Roll back the command packet next-write pointer.
		x->pRsp = x->pMCmd->pMRspBase;		// Roll back the response packet buffer pointer.
		x->NumMCmds--;						// Pretend the MCmd never existed.
	}
	else
	{
		// Insert MCmd length into header of MCmd that is being constructed.
		x->pMCmdBase[1] = MCmdLen;

		// If the target module is not expected to produce an MRsp ...
		if ( x->pMCmd->KillMRsp )
		{
			// If the target iom is the gateway, indicate that we expect no response packet at all.  Otherwise, roll back the
			// response packet buffer pointer to the beginning of the target module's MRsp in the response packet buffer.
			if ( x->pMCmd->ModID == MODID_GATEWAY )
				x->KillRspPacket = TRUE;
			else
				x->pRsp = x->pMCmd->pMRspBase;
		}
	}
}

////////////////////////////////////////////////////////////////////////////
// Open a new MCmd.
// Call this only if no gateway errors are pending.

static void GWBeginMCmd( HXACT x, IOMPORT IomPort, const IOM_ATTR *pIomAttr, u8 StatMask )
{
	// If we are already constructing an MCmd, unwind it into Cmd[] before starting the new MCmd.
	if ( x->NumMCmds > 0 )
		FlushMCmdToPacket( x );

	// If too many MCmds have been declared, set an error indicator.  Otherwise, initialize the new MCmd's structure.
	if ( x->NumMCmds >= MAX_MCMDS )
		x->GWErrCode = GWERR_TOOLARGE;
	else
	{
		// Initialize MCmd's structure.
		x->pMCmd					= &( x->MCmd[x->NumMCmds++] );							// Bump MCmd count and get pointer to new MCmd structure.
		x->pMCmd->ModID				= IomPort;												// Cache iom port number for response checking.
		x->pMCmd->KillMRsp			= FALSE;												// At this point, we expect to get a MRsp.
		x->pMCmd->MRspLen			= MRSP_HEADER_SIZE;										// So far, we expect only the MRsp header in response.
		x->pMCmd->StatMask			= StatMask;												// Mask for iom status byte: 1=passthru, 0=forcezero.
		x->pMCmd->pIomAttr			= pIomAttr;												// Cache pointer to module's constant attributes.
		x->pMCmd->IomCmdBufMargin	= (u8)( pIomAttr->IomCmdBufSize - MCMD_HEADER_SIZE );	// No actions yet, so almost the entire iom command buffer is available.
		x->pMCmd->IomRspBufMargin	= (u8)( pIomAttr->IomRspBufSize - MRSP_HEADER_SIZE );	// No actions yet, so almost the entire iom response buffer is available.

		// Set a pointer to where the MRsp header should appear in the response packet buffer, then advance
		// response buffer NextLoc pointer to start of action responses.
		x->pMCmd->pMRspBase			= x->pRsp;
		x->pRsp						+= MRSP_HEADER_SIZE;

		// Copy the MCmd header to the command packet and advance NextWrite pointer to start of actions.
		x->pMCmdBase				= x->pCmd;							// Cache pointer to base of MCmd under construction in Cmd[].
		x->pMCmdBase[0]				= IomPort;
		x->pCmd						+= MCMD_HEADER_SIZE;
	}
}

////////////////////////////////////////////////////////////////////////////////////////
// Schedule a post-processing action for the command packet that is being constructed.
// Imports:
//	pSrc     - pointer to the unprocessed source data, or the data value itself (which must have same size as pointer).
//	pDst     - pointer to buffer that will receive the post-processed data.
//	nbytes   - optional data byte count, in case it is not implicitly known by the post-processing function.
//	pPProcFn - pointer to post-processing callback function, or zero if no pp function is to be called.
//  ArgSpec  - pointer to list of response argument specifications, or zero if there are no response values.
// Returns GWErrCode.

static GWERR GWAddPost( HXACT x, void *pSrc, void *pDst, u8 nbytes, PPFUNC pPProcFn, const ARGSPEC *pArgSpec )
{
	// If no gateway error is pending and a response is expected ...
	if ( x->GWErrCode == GWERR_NONE )
	{
		if ( !x->pMCmd->KillMRsp )
		{
			// Append this post-process step to the list of steps to be performed for this transaction.
			POSTPROC *p = &( x->PProc[ x->NumPProcs ] );		// Cache pointer to this post-process step.
            if ( x->NumPProcs < MAX_PPSTEPS - 1 )      /// +/- limited
			    x->NumPProcs++;		 							// Bump the ppstep count.
			p->SrcSize	= nbytes;								// Source data size.  This is already implicitly known to most virtual action postprocessors.
			p->pSrcData	= pSrc;									// Source data (actual data or pointer to data value) that has not yet been post-processed.
			p->pDstData	= pDst;									// Destination buffer for post-processed data.
			p->pPProcFn	= pPProcFn;								// Post-processing callback function, which takes a single argument: pointer to POSTPROC struct.
			p->pArgSpec	= pArgSpec;								// Pointer to response argument specifications.
			p->iom		= &( x->mm->IomObj[ x->SelectedIom ] );	// Pointer to iom object associated with this action.
			p->x		= x;									// Pointer to transaction object associated with this action.
		}
	}

	// Return last gateway error, if any.
	return x->GWErrCode;
}

/////////////////////////////////////////////////////////////////
// Append an iom action to the MCmd that is being constructed.
// Imports:
//  x        - pointer to transaction object.
//	Opcode   - action opcode for the target iom.
//	pActArg  - pointer to action arguments, if any.
//	ArgSize  - byte count of action arguments.
//	pActRsp  - pointer to buffer that will receive the action response.
//  StatMask - mask for MRsp status byte: 1's = pass through, 0's = force to zero.
//  ArgSpec  - pointer to list of response argument specifications.  Set to KILL_MRSP if module response is killed by the action.
// Returns GWErrCode.

static GWERR GWAddAction( HXACT x, u8 Opcode, void *pActArg, u8 ArgSize, void *pActRsp, u8 StatMask, const ARGSPEC *ArgSpec )
{
	u8	RspSize = 0;	// Action's response byte count.
	int	KillMRsp;		// Boolean: no MRsp is expected.

	// Abort if gateway error is pending.
	if ( x->GWErrCode != GWERR_NONE )
		return x->GWErrCode;

	// Determine whether the current MRsp will be killed by this action (e.g., due to a module Reset command).
	KillMRsp = ( ArgSpec == (ARGSPEC *)KILL_MRSP );

	// If a module response is expected, compute the action's response size by parsing its argument specification list.
	if ( !KillMRsp )
	{
		// For each response spec, which specifies the attributes of a contiguous group of identically-sized values,
		// totalize the byte count of all values in this group.
		const ARGSPEC *pSpec = ArgSpec;
		for ( ; *pSpec != 0; pSpec++ )
			RspSize = (u8)( RspSize + ( ( *pSpec & MASK_ARGCOUNT ) * ( ( *pSpec & MASK_ARGSIZE ) >> SHFT_ARGSIZE ) ) );
	}

	// IF the current MRsp was marked as "killed" by the previously scheduled action,
	// OR the current MCmd's maximum length would be exceeded by appending the action to it,
	// OR the target iom has insufficient command buffer space for the action's opcode and arguments,
	// OR the this new action doesn't kill its MRsp AND
	//   the target iom has insufficient response buffer space for the action's response,
	//   OR the current MCmd employs an iom status mask that differs from the mask required by this action ...
	if ( x->pMCmd->KillMRsp																			// IF current mrsp previously killed
		||	( ( ( x->pCmd - x->pMCmdBase ) + (u16)( ArgSize + ACT_HEADER_SIZE ) ) > MAX_MCMDSIZE )	// OR mcmd would overflow
		||	( ( ArgSize + ACT_HEADER_SIZE ) > x->pMCmd->IomCmdBufMargin )							// OR iom cmdbuf would overflow
		||	( !KillMRsp && (																		// OR action doesn't kill mrsp AND
			( RspSize > x->pMCmd->IomRspBufMargin )													//   ( iom rspbuf would overflow
			|| ( StatMask != x->pMCmd->StatMask ) ) ) )												//   OR modified status mask )
	{
		// Force a new MCmd start.  The new MCmd inherits the previous MCmd's iom port and iom command and
		// response buffer sizes because the new MCmd's target iom is the same as for the previous MCmd.
		GWBeginMCmd( x, x->SelectedIom, x->pMCmd->pIomAttr, StatMask );
	}

	// If no gateway error is pending ...
	if ( x->GWErrCode == GWERR_NONE )
	{
		// Copy the action's opcode and command arguments to the command packet buffer.
		int argcount;
		u8  *pArg = (u8 *)pActArg;
		*(x->pCmd)++ = (u8)Opcode;
		for ( argcount = (int)ArgSize; argcount > 0; argcount-- )
			*(x->pCmd)++ = *pArg++;

		// Compute the space that will remain in the target iom's command buffer after it has received the action opcode and command arguments.
		x->pMCmd->IomCmdBufMargin = (u8)( x->pMCmd->IomCmdBufMargin - ArgSize - ACT_HEADER_SIZE );

		// Mark the enclosing MRsp as having no response if this action kills the MRsp.
		x->pMCmd->KillMRsp |= KillMRsp;

		// If a response is expected from the iom (e.g., Reset action is not scheduled) ...
		if ( !x->pMCmd->KillMRsp )
		{
			// Modify the current MCmd's status mask as required by this action.  The modified status mask will
			// apply to this action's response as well as all following action responses in the current MCmd.
			x->pMCmd->StatMask &= StatMask;

			// If this action produces a response ...
			if ( RspSize > 0 )
			{
				// Schedule the first (and possibly only) post-processing step for this action:  perform endian correction
				// on all multi-byte values within the action response.  This endian-correction pp step is always scheduled
				// as the first pp step for all actions.  The pp step is scheduled by appending it to the currently empty
				// list of pp steps that are to be performed for this action.
				GWAddPost( x, x->pRsp, pActRsp, RspSize, endianfix_pp, ArgSpec );

				// Compute the space that will remain in the target iom's response buffer after it has executed this action.
				x->pMCmd->IomRspBufMargin = (u8)( x->pMCmd->IomRspBufMargin - RspSize );

				// Advance response packet pointer so it points to beginning of next command packet's response item.
				x->pRsp += RspSize;

				// Increase the MRsp's length by the size of the iom's action response.
				x->pMCmd->MRspLen = (u8)( x->pMCmd->MRspLen + RspSize );
			}
		}

		// Log error if command packet size or anticipated response packet size exceeds the maximum UDP payload
		// size supported by the MM.
		if ( ( x->pCmd - x->Cmd > MAX_MMPAYLOAD_SIZE ) || ( x->pRsp - x->Rsp > MAX_MMPAYLOAD_SIZE ) )
			x->GWErrCode = GWERR_TOOLARGE;
	}

	// Return last gateway error, if any.
	return x->GWErrCode;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Prepare a com or gateway packet for MM retries if retries are enabled for the transaction.
// When this is called, the completed packet should already be stored in x->Cmd[].

static void CommandPacketRetryPrep( HXACT x )
{
	// If MM retries are to be enabled ...
	if ( x->UseRetries )
	{
		// Obtain a unique sequence number for this transaction.
		x->SeqThis = x->SeqNext;

		// Bump the next available sequence number.
		x->SeqNext = ( ( x->SeqNext < SEQNUM_MAX ) ? ( x->SeqNext + 1 ) : SEQNUM_MIN );

		// Replace the first MCmd byte's sequence field with the transaction sequence number.
		x->Cmd[0] = ( x->Cmd[0] & ~SEQNUM_MASK ) | ( x->SeqThis << SEQNUM_SHIFT );
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
// Send a pre-built gateway command packet that resides in x.  Don't wait for a response.

// Helper function: finish construction of gateway command packet before sending it.
static GWERR SchedExecuteFinishBuild( HXACT x )
{
	// Abort if transaction error is already pending.
	if ( x->GWErrCode != GWERR_NONE )
		return x->GWErrCode;

	// Log error if the MM is closed.
	if ( !x->mm->IsOpen )
		x->GWErrCode = GWERR_MMCLOSED;

	// If there are no pending errors and the command packet is not empty ...
	else if ( ( x->GWErrCode == GWERR_NONE ) && ( x->NumMCmds > 0 ) )
	{
		// We were constructing an MCmd, so flush the MCmd into Cmd[] before starting the transaction.
		FlushMCmdToPacket( x );

		// Cache size of gateway command packet in Cmd[].
		x->CmdLen = (u16)( x->pCmd - x->Cmd );

		// Configure the packet for retries, if required.
		CommandPacketRetryPrep( x );
	}

	// Return last error, if any.
	return x->GWErrCode;
}

// Public function.
EXPORT(GWERR) S26_SchedExecuteStart( HXACT x )
{
	// Abort if transaction object doesn't exist.
	if ( x == INVALID_XACT )
		return GWERR_XACTALLOC;

	// Finish command packet construction, and if successful ...
	if ( SchedExecuteFinishBuild( x ) == GWERR_NONE )
	{
		// If the command packet is not empty ...
		if ( x->NumMCmds > 0 )
		{
			// Attempt to transmit the command packet to the MM; log error if failed.
			if ( !S26MOD_SendPacket( x->hSock, x->mm->nordMMAdrs, IPPORT_GATEWAY, x->Cmd, x->CmdLen, 0 ) )
				x->GWErrCode = GWERR_PACKETSEND;
			else
				x->mm->PacketCountSent++;
		}
	}

	// Indicate that the application made an effort to send the command packet.
	x->IsSent = TRUE;

	// Log the time at which the command was sent.
	x->TxTimestamp = S26MOD_CurrentTime();

	// Return last error, if any.
	return x->GWErrCode;
}

/////////////////////////////////////////////////////////////////
// Non-blocking check for a received response packet.
// Returns one of these values:
//  RESP_READY	- Response has been received.
//  RESP_BUSY	- Response not yet received.
//	GWERR_x		- Error detected, no response forthcoming.

EXPORT(int) S26_SchedExecuteIsResponded( HXACT x, u32 msec )
{
	// Abort if transaction object doesn't exist.
	if ( x == INVALID_XACT )
		return GWERR_XACTALLOC;

	// Abort if gateway error is pending.
	if ( x->GWErrCode != GWERR_NONE )
		return x->GWErrCode;

	// Abort if the MM is closed.
	if ( !x->mm->IsOpen )
		x->GWErrCode = GWERR_MMCLOSED;

	// Abort if the application never tried to send the command packet; log it as a PacketSendFailed error.
	else if ( !x->IsSent )
		x->GWErrCode = GWERR_PACKETSEND;

	// If the command packet is empty, notify caller that the transaction completed.
	else if ( ( x->NumMCmds == 0 ) && ( x->LogDev == LOGDEV_GATEWAY ) )
		return RESP_READY;

	// Do a non-blocking check for received response packet. If response packet not yet received ...
	if ( !S26MOD_IsPacketRcvd( x->hSock, 0 ) )
	{
		// If response has not timed out, notify caller that no packet has been received.
		// Note: this correctly handles time wraparound for elapsed times smaller than approximately 25 days.
		if ( ( S26MOD_CurrentTime() - x->TxTimestamp ) < msec )
			return RESP_BUSY;

		// Response timed out. If retries are disabled for this transaction, notify caller that transaction timed out
		if ( !x->UseRetries )
			x->GWErrCode = GWERR_MMNORESPOND;

		// Current retry has timed out. If we have not exhausted the maximum number of allowed retries ...
		else if ( x->RemainingRetries-- )
		{
			// Attempt to retransmit the command packet to the MM; log error if failed.
			if ( !S26MOD_SendPacket( x->hSock, x->mm->nordMMAdrs, IPPortList[x->LogDev], x->Cmd, x->CmdLen, 0 ) )
				x->GWErrCode = GWERR_PACKETSEND;
			else
			{
				x->TxTimestamp = S26MOD_CurrentTime();		// log send time for new retry
				x->mm->RetryCount++;
				x->mm->PacketCountSent++;
				return RESP_BUSY;							// still waiting for response
			}
		}

		// Log error if we exhausted all authorized retry attempts.
		else
			x->GWErrCode = GWERR_MMNORESPOND;
	}

	// If response packet was received ...
	else
	{
		// Copy the response packet into the transaction object.
		x->RspLen = S26MOD_RecvPacket( x->hSock, x->Rsp, sizeof(x->Rsp) );

		// Another packet has been received.
		x->mm->PacketCountRcvd++;

		// If transaction retries are disabled, this is the expected response packet and so we notify caller that transaction completed.
		if ( !x->UseRetries )
			return RESP_READY;

		// Retries are enabled. If received packet and command packet have matching transaction sequence numbers ...
		if ( x->SeqThis == ( ( x->Rsp[0] & SEQNUM_MASK ) >> SEQNUM_SHIFT ) )
		{
			// Correct the first udp payload byte by replacing the sequence number with the byte's "sign-extended" msb.
			if ( x->Rsp[0] & 0x80 )	x->Rsp[0] |= SEQNUM_MASK;
			else					x->Rsp[0] &= ~SEQNUM_MASK;

			// Notify caller that transaction completed.
			return RESP_READY;
		}

		// Retries are enabled but sequence numbers don't match, so notify caller that we are still waiting for a response to the current retry.
		else
			return RESP_BUSY;
	}

	// Return last error.
	return x->GWErrCode;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Wait for a response packet or a timeout, whichever occurs first.  If a packet is received
// before timing out, copy it into x->Rsp[] and check for the proper sequence number; if the
// sequence number is invalid for the current transaction, drop the packet and restart the wait.

EXPORT(GWERR) S26_SchedExecuteWait( HXACT x, u32 msec )
{
	// Abort if transaction object doesn't exist.
	if ( x == INVALID_XACT )
		return GWERR_XACTALLOC;

	// Abort if gateway error is pending.
	if ( x->GWErrCode != GWERR_NONE )
		return x->GWErrCode;

	// Abort if the MM is closed.
	if ( !x->mm->IsOpen )
		x->GWErrCode = GWERR_MMCLOSED;

	// Abort if the application never tried to send the command packet; log it as a PacketSendFailed error.
	else if ( !x->IsSent )
		x->GWErrCode = GWERR_PACKETSEND;

	// If the command packet is not empty ...
	else if ( ( x->NumMCmds > 0 ) || ( x->LogDev != LOGDEV_GATEWAY ) )
	{
		// Wait for valid response packet and copy it into transaction object -----------------------------------------------

		// If retries are enabled for this transaction ...
		if ( x->UseRetries )
		{
			// Repeat ...
			do
			{
				// Wait for response packet.  If wait timed out ...
				if ( !S26MOD_IsPacketRcvd( x->hSock, msec ) )
				{
					// If we have not done all authorized retries ...
					if ( x->RemainingRetries-- )
					{
						// Attempt to retransmit the command packet to the MM; log error if failed.
						if ( !S26MOD_SendPacket( x->hSock, x->mm->nordMMAdrs, IPPortList[x->LogDev], x->Cmd, x->CmdLen, 0 ) )
							x->GWErrCode = GWERR_PACKETSEND;
						else
						{
							x->mm->RetryCount++;
							x->mm->PacketCountSent++;
						}
					}

					// Log error if we have exhausted all authorized retry attempts.
					else
						x->GWErrCode = GWERR_MMNORESPOND;
				}

				// If packet received ...
				else
				{
					// Copy the response packet into the transaction object.
					x->RspLen = S26MOD_RecvPacket( x->hSock, x->Rsp, sizeof(x->Rsp) );

					// Another packet has been received.
					x->mm->PacketCountRcvd++;

					// If received packet and command packet have matching transaction sequence numbers ...
					if ( x->SeqThis == ( ( x->Rsp[0] & SEQNUM_MASK ) >> SEQNUM_SHIFT ) )
					{
						// Correct the first udp payload byte by replacing the sequence number with the byte's "sign-extended" msb.
						if ( x->Rsp[0] & 0x80 )	x->Rsp[0] |= SEQNUM_MASK;
						else					x->Rsp[0] &= ~SEQNUM_MASK;

						// The transaction was successful, so break out of the retry loop.
						break;
					}
				}
			}
			while ( x->GWErrCode == GWERR_NONE );
		}

		// If retries are disabled ...
		else
		{
			// Wait for the response packet or its timeout, whichever occurs first.  Log error upon timeout.
			if ( !S26MOD_IsPacketRcvd( x->hSock, msec ) )
				x->GWErrCode = GWERR_MMNORESPOND;

			// If packet was received ...
			else
			{
				// Copy packet into transaction object.
				x->RspLen = S26MOD_RecvPacket( x->hSock, x->Rsp, sizeof(x->Rsp) );
				
				// Another packet has been received.
				x->mm->PacketCountRcvd++;
			}
		}
	}

	// Return last error, if any.
	return x->GWErrCode;
}




////////////////////////////////////////////////////////////////////////////////////////////
// Parse response packet for errors.  If any errors are detected, the transaction object's
// gateway error code is set accordingly.  Assumed: no gateway errors upon entry.

static void ParseResponsePacket( HXACT x )
{
	u8		mcmd;		// MCmd counter.
	MCMD	*pMC;		// Pointer to current MCmd in the command packet.

	// For each MCmd in the command packet ...
	for ( mcmd = 0, pMC = x->MCmd; mcmd < x->NumMCmds; mcmd++, pMC++ )
	{
		// If an MRsp is expected from the iom ...
		if ( !pMC->KillMRsp )
		{
			// If any of the following errors are detected in the received MRsp ...
			if (	( pMC->pMRspBase[MRSP_POSN_IOMPORT] != pMC->ModID )		// MRsp module ID differs from MCmd module ID.
				||	( pMC->pMRspBase[MRSP_POSN_MRSPLEN] != pMC->MRspLen )	// MRspLen byte differs from expected value.
				||	( pMC->pMRspBase >= ( x->Rsp + x->RspLen ) ) )			// MRsp pointer is beyond end of response pkt buffer.
			{
																	// ... then
				x->GWErrCode = pMC->ModID | GWERR_IOMNORESPOND;		// Set the error code and associate the error with the ID of the offending module.
				return;												// Terminate processing of the response packet.
			}

			// If the MCmd is addressed to an iom ...
			if ( pMC->ModID < NUM_IOMPORTS )
			{
				// After masking off any non-relevant bits, extract and log any asserted bits in the iom's status byte and the
				// middleware-simulated iom status byte.  The middleware-simulated iom status flags are asserted by the middleware
				// when it detects an iom event that can't be reported by the iom itself (e.g., a 2608's eeprom calibration values
				// are found to be out of tolerance and so the middleware will use default values for all cal constants).
				x->IomStatus[pMC->ModID] |= 					// Combine all previously logged status flags with
					( pMC->StatMask & ~STATUS_HRST &			//   the masked versions of
					( pMC->pMRspBase[MRSP_POSN_IOMSTATUS]		//     iom's masked status byte, and
					| x->mm->IomObj[pMC->ModID].SimStatus ) );	//     middleware-simulated iom status byte.
			}
			else	// If the MCmd is addressed to the MM ...
			{
				// Extract and save the mm's status byte.
				x->MMStatus |= pMC->pMRspBase[MRSP_POSN_IOMSTATUS];
			}
		}
	}

	// If we are not at the end of the response packet (as we should be) ...
	if ( x->pRsp != ( x->Rsp + x->RspLen ) )
	{
		// An error has been detected in the final MRsp and so we must indicate the error.
		// This can happen if the last MCmd has an incorrectly specified Action.
		x->GWErrCode = x->MCmd[x->NumMCmds - 1].ModID | GWERR_IOMNORESPOND;
	}
}

///////////////////////////////////////////////////////////////////
// Execute all postprocessing steps for a transaction.

static void PostprocessAll( HXACT x )
{
	POSTPROC	*p;			// Pointer to post-processing step.
	int			i;

	// For each scheduled postprocessing step, execute the step by invoking the specified callback function.
	for ( i = 0, p = x->PProc; i < x->NumPProcs; i++, p++ )
		( (PPFUNC)( p->pPProcFn ) )( p );

	// Abort if the mm's RST flag was asserted.
	if ( ( x->MMStatus & STATUS_RST ) != STATUS_NONE )
	{
		x->GWErrCode = GWERR_IOMRESET | MODID_GATEWAY;
		return;
	}

	// Determine whether any iom status flags (e.g., STATUS_RST or STATUS_CERR) were asserted in any masked MRsp status byte.
	// For each iom port ...
	for ( i = 0; i < NUM_IOMPORTS; i++ )
	{
		// If any of the iom's application status flags were asserted ...
		if ( ( x->IomStatus[i] & ( STATUS_CERR | STATUS_RST | STATUS_TYPESPECIFIC ) ) != STATUS_NONE )
		{
			// Report if any iom CERR error flag was asserted.
			if ( ( x->IomStatus[i] & STATUS_CERR ) != STATUS_NONE )
				x->GWErrCode = GWERR_IOMERROR | i;

			// Else, report if any iom RST flag was asserted.
			else if ( ( x->IomStatus[i] & STATUS_RST ) != STATUS_NONE )
				x->GWErrCode = GWERR_IOMRESET | i;

			// Else report that an iom-specific status flag was asserted.
			else
				x->GWErrCode = GWERR_IOMSPECIFIC | i;

			// Exit loop to "latch" the error code.
			break;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fetch and process the response packet, which is assumed to already be received.  If no errors are
// detected in the response packet, copy all action responses to their application buffers and copy
// all iom status bytes to the application's iom status byte buffer.  If response packet errors are
// detected, parse the response packet to determine the location of the error and set the error code
// accordingly.

EXPORT(GWERR) S26_SchedExecuteFinish( HXACT x, u8 *IomStatus )
{
	GWERR	rtnval;		// Function return value.

	// Abort if transaction object doesn't exist.
	if ( x == INVALID_XACT )
        return GWERR_XACTALLOC;

	// If there are no pending gateway errors ...
	if ( x->GWErrCode == GWERR_NONE )
	{
		// Abort if the MM is closed.
		if ( !x->mm->IsOpen )
			x->GWErrCode = GWERR_MMCLOSED;

		// Abort if the application never tried to send the command packet; log it as a timeout error.
		else if ( !x->IsSent )
			x->GWErrCode = GWERR_MMNORESPOND;

		// Otherwise ...
		else
		{
			// If the command packet is not empty then parse the response packet.
			if ( x->NumMCmds > 0 )
				ParseResponsePacket( x );

			// If no errors have been detected thus far then perform all postprocessing steps.
			if ( x->GWErrCode == GWERR_NONE )
				PostprocessAll( x );
		}
	}

	// Populate the application's iom status buffer if one is specified.
	if ( IomStatus != NULL )
		memcpy( IomStatus, x->IomStatus, NUM_IOMPORTS );

	// Copy transaction error flags so we will still have them after the transaction object is released.
	rtnval = x->GWErrCode;

	// Release transaction object to the free transaction pool.
	XactFree( x );

	// Return transaction error flags.
	return rtnval;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Execute all scheduled iom and postprocessor actions, then destroy the transaction object.
// IomStatus[] will be populated if explicitly specified (non-zero).

EXPORT(GWERR) S26_SchedExecute( HXACT x, u32 msec, u8 *IomStatus )
{
    GWERR err;              // Error code.

	// Send the command packet.
	err = S26_SchedExecuteStart( x );
    if ( err != GWERR_NONE )
    {
        // Release the transaction object to the free pool.
        XactFree( x );

		// Abort with error.
        return err;
    }

	// Wait for response packet (which will be put into the transactions Rsp buffer) or timeout.
	err = S26_SchedExecuteWait( x, msec );

    if ( err != GWERR_NONE )
    {
        // Release the transaction object to the free pool.
        XactFree( x );

		// Abort with error.
        return err;
    }

	// Execute all scheduled postprocesses, release transaction object, return any logged errors.
	return S26_SchedExecuteFinish( x, IomStatus );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////  MM COMMUNICATION INITIALIZATION  ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Transmit a command packet to the gateway and wait for either a response packet or a timeout, whichever
// occurs first.  The calling thread is blocked while the transmit/receive transaction is in progress.
// The received response, if any, is placed in the specified response packet buffer.
// Imports:
//	pCmd	= Pointer to command packet.
//	CmdLen	= Byte count of command packet.
//	msec	= Max time to wait for a response, in milliseconds.
// Returns byte count of received response packet via transaction object.

static void MMTransact( HXACT x, const u8 *pCmd, u16 CmdLen, u32 msec )
{
	// No response has been received yet.
	x->RspLen = 0;

	// Attempt to transmit the command packet to the MM, and if successful ...
	if ( S26MOD_SendPacket( x->hSock, x->mm->nordMMAdrs, IPPORT_GATEWAY, pCmd, CmdLen, 0 ) )
	{
		// Another packet has been sent.
		x->mm->PacketCountSent++;

		// Wait for the response packet or its timeout, whichever occurs first.  During the wait, the
		// current thread is blocked.  If packet was successfully received ...
		if ( S26MOD_IsPacketRcvd( x->hSock, msec ) )
		{
			// Fetch the response packet and return its byte count.
			x->RspLen = S26MOD_RecvPacket( x->hSock, x->Rsp, sizeof(x->Rsp) );

			// Another packet has been received.
			x->mm->PacketCountRcvd++;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// MM communication init.  This function blocks the calling thread while the MM is being initialized.
// The function is implemented as a virtual state machine which performs the following functions:
//  1. Reset the MM and all attached IOMs.
//  2. Synchronize communications with the MM.
//  3. Clear the MM's RST status flag.
//  4. Set IsActive true if the MM is ready to communicate.
// Imports:
//	mm			: pointer to MM object.
//	NetLatency	: network latency in milliseconds.  Specify 1 msec for typical dedicated LAN.
//	LinkLatency	: client delay (msec) in detecting active MM network link.
// Returns true if successful.

// Initialization sequencer states.
typedef enum MMSTATE {					// MM initialization states:
	STATE_RESET,						//   Invoking HardReset on the MM.
	STATE_SYNC,							//   Verifying MM has undergone a hard reset.
	STATE_INIT,							//   Clearing the MM's hard reset indicator.
	STATE_ONLINE						//   Init completed: MM is online.
} MMSTATE;

// Maximum number of reset retries.  Rather than limit the number of retries, this value actually
// limits only the total time allowed for the reset sequencer to run before forcing a timeout.
#define NUM_RESET_RETRIES	1			// Maximum number of reset retries.

// Maximum allowable execution times in milliseconds, excluding network latency.
#define MSEC_MMRESET		29			// MM HardReset.
#define MSEC_MMNOP			29			// MM NOP.
#define MSEC_MMINIT			29			// MM ResetFlags.
#define MSEC_RECONNECT		1400		// Total time from MM reset to network link generation (excludes client's link detection delay).

// Code clarification aliases.
#define MRSP_MODID			x->Rsp[0]	// MRsp source module identifier.
#define MRSP_LENGTH			x->Rsp[1]	// MRsp length byte.
#define MRSP_STATUS			x->Rsp[2]	// MRsp status byte.
#define RSP					( ( x->RspLen == MRSP_HEADER_SIZE ) && ( MRSP_MODID == MODID_GATEWAY ) && ( MRSP_LENGTH == MRSP_HEADER_SIZE ) )
#define RST					( ( MRSP_STATUS & STATUS_RST ) != 0 )

#define NO_RETRIES			0			// Disable the MM's retry mechanism.
#define TWO_RETRIES			2			// Perform two retries.

// MM command packets used for MM communication initialization.
static const u8 RstCmd[]  = { MODID_GATEWAY, 3, OP_HARDRESET				};		// Hard reset.
static const u8 SyncCmd[] = { MODID_GATEWAY, 3, OP_NOP						};		// No operation.
static const u8 InitCmd[] = { MODID_GATEWAY, 4, OP_RESETFLAGS, STATUS_RST	};		// Clear RST status flag.

static BOOL ResetNetwork( MM_OBJ *mm, u32 NetLatency, u32 LinkLatency )
{
    GWERR err;              // Error code.
	IOMPORT	i;
	BOOL	rsp;
	BOOL	rst;
	XACT	*x;		// Transaction object.

	// Calculate time constants for individual states within the reset sequence.
	u32	msecMMRESET	= MSEC_MMRESET	+ NetLatency;
	u32	msecMMNOP	= MSEC_MMNOP	+ NetLatency;
	u32	msecMMINIT	= MSEC_MMINIT	+ NetLatency;

	// Calculate maximum time to wait for the reset sequencer to progress to STATE_ONLINE.
	u32 msecMAXTIME	= ( NUM_RESET_RETRIES + 1 ) * ( MSEC_RECONNECT + LinkLatency + msecMMRESET + MSEC_MMNOP + MSEC_MMINIT );

	// Start the timer.  Instead of tracking real time here we will add time increments of known execution time
	// as each sequencer state executes.
	u32 msecElapsed = 0;

	// Set the initial state of the virtual state machine that is responsible for initializing the MM.  When the state machine has
	// fully initialized MM, the MM will be ready to communicate with the client (i.e., STATE_READY) and this function will terminate.
	MMSTATE State = STATE_RESET;

	// Abort if MM is closed.
    if ( !mm->IsOpen )
        return FALSE;

	// Loop until the MM is initialized or we timeout ...
	do
	{
		switch ( State )
		{
		case STATE_RESET:

			// Send a HardReset action to the MM and wait long enough to ensure MM has rebooted.  If MM
			// executes the HardReset, it will not respond.  If we transmit while MM is rebooting (i.e.,
			// it just had a communication watchdog timeout), MM will not respond.  If MM has failed or
			// is not connected, it will not respond.  In any of these cases, something is wrong if we
			// receive a response.  If this happens, we will remain in STATE_RESET and retry until the
			// MM successfully reboots (i.e., it fails to respond) or the gateway thread is terminated.

			// Attempt to obtain a transaction object from the free pool.  Abort if failed.
			if ( ( x = XactAlloc( mm, LOGDEV_GATEWAY, NO_RETRIES ) ) == NULL )
                return FALSE;

			// Send a RESET command to the gateway.
			MMTransact( x, (u8 *)RstCmd, sizeof(RstCmd), msecMMRESET );

			// Cache the transaction results.
			rsp = ( x->RspLen != 0 );

			// Release the transaction object to the free pool.
			XactFree( x );

			// Advance to the SYNC state if there was no reply.
			if ( !rsp )
			{
				msecElapsed += msecMMRESET;
				State = STATE_SYNC;
			}
			else
				msecElapsed += NetLatency;

			break;

		case STATE_SYNC:

			// We have sent a HardReset to MM and it does not answer back (which is the proper reaction
			// when MM executes a HardReset), so we will now verify that MM has undergone a hard reset.
			// We accomplish this by transmitting MM NOPs until MM responds, then examine the MM's RST
			// status flag.  If RST is not set, we must switch back to STATE_RESET to force another
			// HardReset.  If RST is set, we can advance to STATE_INIT.

			// Attempt to obtain a transaction object from the free pool.  Abort if failed.
			if ( ( x = XactAlloc( mm, LOGDEV_GATEWAY, NO_RETRIES ) ) == NULL )
                return FALSE;

			// Send a NOP command to the gateway.
			MMTransact( x, (u8 *)SyncCmd, sizeof(SyncCmd), msecMMNOP );

			// Cache the transaction results.
			rsp = RSP;

			// Release the transaction object to the free pool.
			XactFree( x );

			// Change state if we received a MM reply.
			if ( rsp )
			{
				msecElapsed += NetLatency;
				State = ( RST ? STATE_INIT : STATE_RESET );
			}
			else
				msecElapsed += msecMMNOP;

			break;

		case STATE_INIT:

			// MM has successfully undergone a HardReset and is responding, so we now attempt to clear its
			// RST status flag.  If we can clear RST, we will advance to STATE_ONLINE.  If RST is not
			// cleared, we will retry until it is cleared.  If MM does not respond, we will switch back
			// to STATE_SYNC to re-establish communication with MM.

			// Attempt to obtain a transaction object from the free pool.  Abort if failed.
			if ( ( x = XactAlloc( mm, LOGDEV_GATEWAY, NO_RETRIES ) ) == NULL )
                return FALSE;

			// Send a Clear RST Status Flag command to the MM.
			MMTransact( x, (u8 *)InitCmd, sizeof(InitCmd), msecMMINIT );

			// Cache the transaction results.
			rsp = RSP;
			rst = RST;

			// Release the transaction object to the free pool.
			XactFree( x );

			// If MM responded ...
			if ( rsp )
			{
				// If the MM's RST flag has been cleared ...
				if ( !rst )
					State = STATE_ONLINE;			// MM is now online.  Mark it accordingly and ignore elapsed time.
				else
					msecElapsed += NetLatency;		// RST flag still asserted, so bump elapsed time due to transaction and retry.
			}
			else
			{
				msecElapsed += msecMMINIT;			// Elapsed time due to timeout.
				State = STATE_SYNC;					// MM didn't respond, so switch back to the SYNC state.
			}

			break;

		case STATE_ONLINE:

			// The MM is back online and so we can exit.  Starting from the time of the preceding transaction (Clear RST
			// Status Flag), the client has approximately ten seconds to communicate with the MM to avoid a communication
			// timeout.  All previously registered ioms remain registered because we assume that no iom connection changes
			// have resulted from the module system reset.  All ioms are assumed to have been reset, so we will now clear
			// all iom error flags (both simulated and real).

			// Restart the MM's diagnostics counters.
			mm->PacketCountRcvd = 0;
			mm->PacketCountSent = 0;
			mm->RetryCount	= 0;

			// Attempt to obtain a transaction object from the free pool.  Abort if failed.
			if ( ( x = XactAlloc( mm, LOGDEV_GATEWAY, TWO_RETRIES ) ) == NULL )
				return FALSE;

			// For each iom port ...
			for ( i = 0; i < NUM_IOMPORTS; i++ )
			{
				// If the port was previously registered (connected to a known iom type) then assume it is still
				// connected and schedule negation of all of the iom's error status bits (both real and simulated).
				if ( mm->IomObj[i].pIomAttr->IomType != IOMTYPE_UNREG )
				{
					err = Sched2600_ClearStatus( x, i, ~0 );
					if ( err != GWERR_NONE )
					{
						// Release the transaction object to the free pool.
						XactFree( x );

						// Report error and exit with Fail code.
						return FALSE;
					}
				}
			}

			// Execute transaction and release the transaction object. Indicate success/fail and exit.
			return ( S26_SchedExecute( x, NetLatency + 100, NULL ) == GWERR_NONE );
		}
	}
	while ( msecElapsed < msecMAXTIME );

	// Indicate failure (due to timeout).
	return FALSE;
}	

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////  BOARD & DRIVER INITIALIZATION AND STATUS  ////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
// Return pointer to middleware version string. First available in middleware version 1.0.10.

EXPORT(const char *) S26_DriverVersion( void )
{
	return VERSION_STRING;
}

///////////////////////////////////
// Helper function: close a MM.

static void BoardClose( MM_OBJ *mm )
{
	int i;

	// If the MM is open ...
	if ( mm->IsOpen )
	{
		// For each transaction object belonging to this MM ...
		for ( i = 0; i < MAX_TRANSACTS; i++ )
		{
			// In case it is active, force an error to terminate this transaction.  We do this by forcing a
			// GWERR_MMCLOSED error, but don't release it to the free transaction object pool now.  This will
			// be done later when the transaction is "executed."
			mm->XactPool.ObjPool[i].GWErrCode = GWERR_MMCLOSED;

			// Destroy the socket associated with the transaction object.  Sockets are no longer needed for open
			// transactions because they now have pending GWERR_MMCLOSED errors.
			S26MOD_CloseSocket( mm->XactPool.ObjPool[i].hSock );
			mm->XactPool.ObjPool[i].hSock = 0;
		}

		// Mark the MM as Closed.
		mm->IsOpen = FALSE;
	}
}

/////////////////////////////////////////////////////////////////////////
// Initialize the core.  This must be the first core function called.
// Imports:
//  NumMMs = Number of MMs (Model 2601 Main Modules) in the system.
// Returns:
//  0						= successful.
//  DRVERR_MALLOC			= failed due to memory allocation problem.
//  DRVERR_NETWORKOPEN		= failed due to network initialization problem.
//  DRVERR_CRITICALSECTION	= failed due to critical section creation problem.
//	DRVERR_REOPEN			= failed due to already open.

EXPORT(int) S26_DriverOpen( int NumMMs )
{
	u32		hbd;	// Logical board number.
	MM_OBJ	*mm;	// Pointer to core's MM objects.
	XACT	*x;		// Pointer to transaction control object.
	int		i;

    // Reopen protection.
    if ( MMCount != 0 )
        return DRVERR_REOPEN;

	// Allocate storage for all MM objects.  Abort if the allocation failed.
	if ( ( obj = malloc( NumMMs * sizeof(MM_OBJ) ) ) == NULL )
		return DRVERR_MALLOC;

	// Open the network interface.  Abort if the network open failed.
	if ( S26MOD_OpenNetwork( NumMMs * MAX_TRANSACTS ) != 0 )
	{
		// Free storage used by the MM objects
		free( obj );

		// Indicate network could not be opened.
		return DRVERR_NETWORKOPEN;
	}

	// Initialize the MM objects.  For each MM object ...
	for ( hbd = 0, mm = obj; hbd < (u32)NumMMs; hbd++, mm++ )
	{
		// Create critical section used for managing the transaction object pool.  If failed ...
		if ( ( mm->XactPool.hCriticalSection = S26MOD_CriticalSectionCreate() ) == S26_INVALID_CRITICALSECTION )
		{
			// Close the network interface.
			S26MOD_CloseNetwork();

			// Destroy any previously created critical sections.
			for ( i = 0; i < (int)MMCount; i++ )
				S26MOD_CriticalSectionDestroy( mm->XactPool.hCriticalSection );

			// Free storage used by the MM objects
			free( obj );

			// No MM objects exist.
			MMCount = 0;

			// Indicate critical section could not be created.
			return DRVERR_CRITICALSECTION;
		}

		// Mark MM as closed.
		mm->IsOpen = FALSE;

		// There are no registered iom's.
		mm->IomCount = 0;

		// For each of this MM's transaction objects ...
		for ( i = 0; i < MAX_TRANSACTS; i++ )
		{
			// Cache pointer to the transaction control object, and add object handle to the free object pool.
			x = mm->XactPool.FreeList[i] = &mm->XactPool.ObjPool[i];

			// Mark the transaction object's socket as Not Created.
			x->hSock = S26_INVALID_SOCKET;

			// Set pointer to the mm that owns this transaction object.
			x->mm = mm;

			// Init the sequence number.
			x->SeqNext = SEQNUM_MIN;
		}

		// Register the object count and mark the transaction object pool as Full.
		mm->XactPool.FreeCount = mm->XactPool.TotalCount = MAX_TRANSACTS;

		// Initialize "constants" for each IOM port on this MM ...
		for ( i = 0; i < NUM_IOMPORTS; i++ )
		{
			mm->IomObj[i].mm		= mm;				// Init pointer to this iom's parent mm.
			mm->IomObj[i].IomPort	= (u8)i;			// MM port number to which this iom is attached.
		}

		// Another MM has been successfully created.
		MMCount++;
	}

	// Indicate success.
	return DRVERR_NONE;
}

/////////////////////////////////////////////////////////////////////////
// Shutdown the core.  This must be the last function called.

EXPORT(void) S26_DriverClose( void )
{
	u32		hbd;			// Logical board number.
	MM_OBJ	*mm;//	= obj;		// Pointer to core's MM object.

	// For each MM object ...
	for ( hbd = 0; hbd < MMCount; hbd++ )
	{
		// Cache pointer to the MM object.
		mm = &obj[hbd];

		// Close the MM if it is open.
		BoardClose( mm );

		// Destroy the critical section used to manage the transaction object pool.
		S26MOD_CriticalSectionDestroy( mm->XactPool.hCriticalSection );
	}

	// Close the network interface.
	S26MOD_CloseNetwork();

	// Free storage used by the MM objects
    if ( obj != NULL )
    {
        free( obj );
        obj = NULL;
    }
	// Mark objects as unallocated.
	MMCount = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Open logical MM.
// Imports:
//	hbd			= logical board number.
//	CLDotAdrs	= local IP address.  Set to zero to use default NIC, or to specific address
//					to use a specific NIC in a multi-homed host.
//	MMDotAdrs	= MM's IP address.
// Returns zero if successful, otherwise error flags (ERR_BADHANDLE, ERR_CREATESOCKET, ERR_BINDSOCKET).

EXPORT(u32) S26_BoardOpen( HBD hbd, char *CLDotAdrs, const char *MMDotAdrs )
{
	MM_OBJ	*mm;								// Pointer to core's MM object.
	u32		ClientAdrs;							// Client's address in network byte order.
	u32		FaultFlags = ERR_NONE;			// No errors detected yet.
	int		i;

	// Return error if board handle is illegal.
	if ( hbd >= MMCount )
		return ERR_BADHANDLE;

	// Cache pointer to core's MM object.
	mm = &obj[hbd];

	// Close the board if it is already open.
	BoardClose( mm );

	// Get the MM's IP address in network byte order.
	mm->nordMMAdrs = S26MOD_inet_addr( MMDotAdrs );

	// Get the Client's network byte ordered IP address.  Use default NIC if no client address is specified (i.e., INADDR_ANY).
	ClientAdrs = ( CLDotAdrs != NULL ) ? S26MOD_inet_addr( CLDotAdrs ) : 0;
	
	// Create and bind a dedicated socket to each transaction object ---------------------------------
	// If any of these create or bind attempts fail then we will delete any sockets that have already
	// been created for this MM, the MM will be marked as Closed and an error code will be returned.

	// For each transaction object ...
	for ( i = 0; i < MAX_TRANSACTS; i++ )
	{
		// Create a socket and assign it to its associated transaction object.  Abort if socket could not be created.
		if ( ( mm->XactPool.ObjPool[i].hSock = S26MOD_CreateSocket() ) == S26_INVALID_SOCKET )
		{
			FaultFlags = ERR_CREATESOCKET;
			break;
		}
		
		// Bind the socket to the specified MM network interface's IP port and address.  Close socket and abort if bind failed.
		else if ( !S26MOD_BindSocket( mm->XactPool.ObjPool[i].hSock, ClientAdrs ) )
		{
			// Close the socket.
			S26MOD_CloseSocket( mm->XactPool.ObjPool[i].hSock );
			mm->XactPool.ObjPool[i].hSock = 0;

			FaultFlags = ERR_BINDSOCKET;
			break;
		}
	}

	// Destroy all sockets and transaction objects if a socket create/bind problem was encountered.
	if ( FaultFlags != ERR_NONE )
	{
		// For each transaction object belonging to this MM ...
		for ( i = 0; i < MAX_TRANSACTS; i++ )
		{
			// Exit loop if all sockets have been closed.
			if ( mm->XactPool.ObjPool[i].hSock == S26_INVALID_SOCKET )
				break;

			// Close the socket associated with the transaction object.
			S26MOD_CloseSocket( mm->XactPool.ObjPool[i].hSock );
			mm->XactPool.ObjPool[i].hSock = 0;
		}

		// Return error code.  The board was never opened.
		return FaultFlags;
	}

	// For each iom port ...
	for ( i = 0; i < NUM_IOMPORTS; i++ )
	{
		// Mark the iom type as undefined.
		mm->IomObj[i].pIomAttr	= &IomAttrUnreg;

		// No middleware-simulated iom error flags are asserted for this iom.
		mm->IomObj[i].SimStatus = STATUS_NONE;
	}

	// There are no registered iom's.
	mm->IomCount = 0;

	// Init the MM's diagnostics info.
	mm->PacketCountRcvd	= 0;
	mm->PacketCountSent	= 0;
	mm->RetryCount	= 0;

	// Mark MM as open.
	mm->IsOpen = TRUE;

	// Indicate success.
	return ERR_NONE;
}

////////////////////////////////////////////////////
// Close logical MM.

EXPORT(void) S26_BoardClose( HBD hbd )
{
	// If legal MM handle was specified then close the MM.
	if ( hbd < MMCount )
		BoardClose( &obj[hbd] );
}

////////////////////////////////////////////////////////////
// MM init function.

EXPORT(BOOL) S26_ResetNetwork( HBD hbd )
{
	// Execute MM reset if legal board handle was specified.  We assume operation on a dedicated
	// lan, so NetLatency is set to 1ms and LinkLatency is set to a nominal 2 seconds.
	return ( hbd < MMCount ) ? ResetNetwork( &obj[hbd], 1, 2000 ) : 0;
}	


////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////  COMPORT FUNCTIONS  /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////
// Send the contents of the transaction command buffer to the MM and wait for either a response packet
// or a timeout, whichever occurs first.  The calling thread is blocked while the transaction is in
// progress.  The received response, if any, is placed in the transaction response packet buffer.

static GWERR ComTransact( HXACT x, u8 LogDev, u16 CmdLen, u16 MaxRspLen, u32 msec )
{
	// Abort if the MM is closed.
	if ( !x->mm->IsOpen )
		x->GWErrCode = GWERR_MMCLOSED;

	// Abort if comport is illegal.
	else if ( ( LogDev - 1 ) >= LOGDEV_COM4 )
		x->GWErrCode = GWERR_BADVALUE | 2;

	// Execute the transaction.
	else
	{
		// Prepare the command packet for MM retries, if retries are enabled for this transaction.
		CommandPacketRetryPrep( x );

		// Attempt to transmit the command packet to the MM, and if successful ...
		if ( S26MOD_SendPacket( x->hSock, x->mm->nordMMAdrs, IPPortList[LogDev], x->Cmd, CmdLen, 0 ) )
		{
			// Another packet has been sent.
			x->mm->PacketCountSent++;
			
			// Cache the packet size in case we must do retries.
			x->CmdLen = CmdLen;

			// Indicate command packet is sent.
			x->IsSent = TRUE;

			// Wait for the response packet or its timeout.  During the wait, the current thread is blocked.  If
			// a packet is received before a timeout, the packet is placed into x->Rsp[].
			S26_SchedExecuteWait( x, msec );
		}
		else
			x->GWErrCode = GWERR_PACKETSEND;
	}

	// Return error code.
	return x->GWErrCode;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Release transaction object to the free pool and return combined GWERR and comport status byte.

static GWERR ComXactFree( HXACT x )
{
	// Cache the return value, which is the combined transaction error code and comport status byte.
	GWERR rtnval = x->GWErrCode;
	if ( x->GWErrCode == GWERR_NONE )
		rtnval |= (GWERR)x->Rsp[0];

	// Release the transaction object to the free pool.
	XactFree( x );

	// Return combined transaction error and comport status byte.
	return rtnval;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// PUBLIC COMPORT FUNCTIONS
// Imports:
//	hbd			= board handle in range 0:nBoards-1.
//	LogDev		= logical device id in range 1:4 (COM1:COM4).
// Returns transaction error (GWERR) or'ed with comport status byte (REJ,OPN,0,BRK,FRM,PAR,OVR,WRP).
/////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
// Program a serial ComPort's operating mode.  Returns the transaction error code,
// combined with ComPort's status byte if response packet was received.

EXPORT(GWERR) S26_ComSetMode( HBD hbd, u8 LogDev, u16 ClockDiv, u8 Attributes, u8 LEDs, u32 msec, u32 retries )
{
	HXACT x;		// Pointer to transaction object.

	// Abort if board handle is illegal.
	if ( hbd >= MMCount )
		return GWERR_BADVALUE | 1;

	// Attempt to obtain a transaction object from the free pool.  Abort if failed.
	if ( ( x = XactAlloc( &obj[hbd], LogDev, retries ) ) == NULL )
		return GWERR_XACTALLOC;

	// Populate the command packet.
	x->Cmd[0] = SIO_SETMODE;								// ComPort opcode: set operating mode.
	*( (u16 *)( &( x->Cmd[1] ) ) ) = HtoLE_16( ClockDiv );	// Baud rate clock divisor.
	x->Cmd[3] = Attributes;									// Size (bit count) of each serially transmitted value.
	x->Cmd[4] = LEDs;										// LED usage:  ( 0, 0, 0, 0, XMT, RCV, ERR/BRK, 0 ).

	// Transmit the command packet to the MM and wait for a response packet or a timeout.
	ComTransact( x, LogDev, 5, sizeof(x->Rsp), msec );

	// Release transaction object to free pool, and return combined transaction error and comport status byte.
	return ComXactFree( x );
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Program a serial comport's Break character.  This is the char that is to be inserted into
// the comport's receive buffer upon detection of an incoming break condition.  Returns the
// transaction error code, combined with ComPort's status byte if response packet was received.

EXPORT(GWERR) S26_ComSetBreakChar( HBD hbd, u8 LogDev, u8 BreakChar, u32 msec, u32 retries )
{
	HXACT x;		// Pointer to transaction object.

	// Abort if board handle is illegal.
	if ( hbd >= MMCount )
		return GWERR_BADVALUE | 1;

	// Attempt to obtain a transaction object from the free pool.  Abort if failed.
	if ( ( x = XactAlloc( &obj[hbd], LogDev, retries ) ) == NULL )
		return GWERR_XACTALLOC;

	// Populate the command packet.
	x->Cmd[0] = SIO_SETBREAKCHAR;		// ComPort opcode: set break character.
	x->Cmd[1] = BreakChar;				// The Break character.

	// Transmit the command packet to the MM and wait for a response packet or a timeout.
	ComTransact( x, LogDev, 2, sizeof(x->Rsp), msec);

	// Release transaction object to free pool, and return combined transaction error and comport status byte.
	return ComXactFree( x );
}

//////////////////////////////////////////////////////////////////////////////////////////
// Transmit a byte string on the specified ComPort.  Returns the ComPort's status byte
// if a response packet was received, or an error code if the transaction failed.

EXPORT(GWERR) S26_ComSend( HBD hbd, u8 LogDev, u8 *MsgBuf, u16 MsgLen, u32 msec, u32 retries )
{
	HXACT x;		// Pointer to transaction object.

	// Abort if message size (plus opcode byte) is too large to fit into MM's ComPort receive buffer.
	if ( MsgLen > ( MAX_MMPAYLOAD_SIZE - 1 ) )
		return GWERR_TOOLARGE;

	// Abort if board handle is illegal.
	if ( hbd >= MMCount )
		return GWERR_BADVALUE | 1;

	// Attempt to obtain a transaction object from the free pool.  Abort if failed.
	if ( ( x = XactAlloc( &obj[hbd], LogDev, retries ) ) == NULL )
		return GWERR_XACTALLOC;

	// Build a command packet by prepending the ComPort opcode to the data string.
	x->Cmd[0] = SIO_SEND;						// Cmd[0]   gets the opcode.
	memcpy( &x->Cmd[1], MsgBuf, MsgLen );		// Cmd[1..] gets the character string.

	// Transmit the command packet to the MM and wait for a response packet or a timeout.
	ComTransact( x, LogDev, (u16)( MsgLen + 1 ), sizeof(x->Rsp), msec );

	// Release transaction object to free pool, and return combined transaction error and comport status byte.
	return ComXactFree( x );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Fetch all available bytes, or MsgLen bytes, whichever is smaller, from the specified ComPort's
// serial receive queue into MsgBuf[].  The number of bytes transferred to MsbBuf[] is returned in
// MsgLen.  Returns the ComPort's status byte if a response packet was received, or an error code
// if the MM transaction failed.

EXPORT(GWERR) S26_ComReceive( HBD hbd, u8 LogDev, u8 *MsgBuf, u16 *MsgLen, u32 msec, u32 retries )
{
	HXACT x;		// Pointer to transaction object.

	// Abort if board handle is illegal.
	if ( hbd >= MMCount )
		return GWERR_BADVALUE | 1;

	// Attempt to obtain a transaction object from the free pool.  Abort if failed.
	if ( ( x = XactAlloc( &obj[hbd], LogDev, retries ) ) == NULL )
		return GWERR_XACTALLOC;

	// Populate the command packet.
	x->Cmd[0] = SIO_RECEIVE;				// Opcode: request queued receive data.
	x->Cmd[1] = (u8)( *MsgLen & 0xFF);		// Max number of bytes to receive, low byte.
	x->Cmd[2] = (u8)( *MsgLen >> 8 );		// Max number of bytes to receive, high byte.

	// Transmit the command packet to the MM and wait for a response packet or a timeout.
	// If a response packet was received ...
	if ( ComTransact( x, LogDev, 3, *MsgLen, msec ) == GWERR_NONE )
	{
		// Compute the data payload size by discounting the ComPort status byte, then copy the
		// payload to the application's receive buffer if it has a non-zero byte count.
		if ( --( x->RspLen ) )
			memcpy( MsgBuf, &x->Rsp[1], x->RspLen );
	}
	else	// If receive timed out ...
	{
		// Received data size is zero.
		x->RspLen = 0;
	}

	// Return received char count to application.
	*MsgLen = x->RspLen;

	// Release transaction object to free pool, and return combined transaction error and comport status byte.
	return ComXactFree( x );
}

////////////////////////////////////////////////////////////////////////////////////////
// Helper function: send a one-byte command to ComPort and return the response packet,
// which consists of a single status byte.

static GWERR ComSimpleCmd( HBD hbd, u8 LogDev, u8 Opcode, u32 msec, u32 retries )
{
	HXACT x;		// Pointer to transaction object.

	// Abort if board handle is not legal.
	if ( hbd >= MMCount )
		return GWERR_BADVALUE | 1;

	// Attempt to obtain a transaction object from the free pool.  Abort if failed.
	x = XactAlloc( &obj[hbd], LogDev, retries );
	if ( x == INVALID_XACT )
		return GWERR_XACTALLOC;

	// Build a command packet consisting of a single opcode byte.
	x->Cmd[0] = Opcode;

	// Transmit the command packet to the MM and wait for a response packet or a timeout.
	ComTransact( x, LogDev, 1, sizeof(x->Rsp), msec );

	// Release transaction object to free pool, and return combined transaction error and comport status byte.
	return ComXactFree( x );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper function: return the number of characters pending in a comport's transmit or receive buffer.

static GWERR ComGetBufCount( HBD hbd, u8 LogDev, u16 *CharCount, u8 Opcode, u32 msec, u32 retries )
{
	HXACT x;		// Pointer to transaction object.

	// Abort if board handle is illegal.
	if ( hbd >= MMCount )
		return GWERR_BADVALUE | 1;

	// Attempt to obtain a transaction object from the free pool.  Abort if failed.
	if ( ( x = XactAlloc( &obj[hbd], LogDev, retries ) ) == NULL )
		return GWERR_XACTALLOC;

	// Build a command packet consisting of a single opcode byte.
	x->Cmd[0] = Opcode;

	// Transmit the command packet to the MM and wait for a response packet or a timeout.
	ComTransact( x, LogDev, 1, sizeof(x->Rsp), msec );

	// If a valid response packet was received then skip over the ComPort status byte, convert char count
	// to host byte order and copy it to app storage.  Otherwise, force the character count to zero.
	*CharCount = ( ( x->RspLen == 3 ) ? LEtoH_16( *( (u16 *)( &( x->Rsp[1] ) ) ) ) : 0 );

	// Release transaction object to free pool, and return combined transaction error and comport status byte.
	return ComXactFree( x );
}

///////////////////////////////////////////////////////////////////////////////////
// Reset all error flags on the specified ComPort.

EXPORT(GWERR) S26_ComClearFlags( HBD hbd, u8 LogDev, u32 msec, u32 retries )
{
	return ComSimpleCmd( hbd, LogDev, SIO_CLEARFLAGS, msec, retries );
}

///////////////////////////////////////////////////////////////////////////////////
// Open the specified ComPort.

EXPORT(GWERR) S26_ComOpen( HBD hbd, u8 LogDev, u32 msec, u32 retries )
{
	return ComSimpleCmd( hbd, LogDev, SIO_OPEN, msec, retries );
}

///////////////////////////////////////////////////////////////////////////////////
// Close the specified ComPort.

EXPORT(GWERR) S26_ComClose( HBD hbd, u8 LogDev, u32 msec, u32 retries )
{
	return ComSimpleCmd( hbd, LogDev, SIO_CLOSE, msec, retries );
}

///////////////////////////////////////////////////////////////////////////////////
// Flush the receiver and reset all error flags on the specified ComPort.

EXPORT(GWERR) S26_ComFlush( HBD hbd, u8 LogDev, u32 msec, u32 retries )
{
	return ComSimpleCmd( hbd, LogDev, SIO_FLUSH, msec, retries );
}

///////////////////////////////////////////////////////////////////////////////////
// Start break transmission.

EXPORT(GWERR) S26_ComStartBreak( HBD hbd, u8 LogDev, u32 msec, u32 retries )
{
	return ComSimpleCmd( hbd, LogDev, SIO_STARTBREAK, msec, retries );
}

///////////////////////////////////////////////////////////////////////////////////
// Terminate break transmission.

EXPORT(GWERR) S26_ComEndBreak( HBD hbd, u8 LogDev, u32 msec, u32 retries )
{
	return ComSimpleCmd( hbd, LogDev, SIO_ENDBREAK, msec, retries );
}

///////////////////////////////////////////////////////////////////////////////
// Return the number of characters pending in the receive buffer.

EXPORT(GWERR) S26_ComGetRxCount( HBD hbd, u8 LogDev, u16 *CharCount, u32 msec, u32 retries )
{
	return ComGetBufCount( hbd, LogDev, CharCount, SIO_GETRXCOUNT, msec, retries );
}

///////////////////////////////////////////////////////////////////////////////
// Return the number of characters pending in the transmit buffer.

EXPORT(GWERR) S26_ComGetTxCount( HBD hbd, u8 LogDev, u16 *CharCount, u32 msec, u32 retries )
{
	return ComGetBufCount( hbd, LogDev, CharCount, SIO_GETTXCOUNT, msec, retries );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////  GATEWAY SCHEDULER FUNCTIONS  //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

#define IMPLICIT_SIZE	0		// Byte count specification for postprocessor functions that have implicit knowledge of the byte count.

//////////////////////////////////////////////////////////////////////////////
// Verify IomType is registered.  If so, the target iom is selected and, if
// the target iom is not the previously addressed iom, a new mcmd is started.
// Imports:
//	x             : Pointer to transaction object.
//	IomPort       : Target iom's port number on the mm, or MODID_GATEWAY if target is the gateway.
//  IomType       : Iom type.  This must be the registered type, or zero if any iom type is allowed.
// Returns gateway error code if validation or selection fails, or zero if successful.

static GWERR SelectIom( XACT **px, IOMPORT IomPort, u16 IomType )
{
	XACT			*x;					// Pointer to transaction object.
	const IOM_ATTR	*pIomAttr;			// Pointer to module's constant attributes.

	// Redirect transaction to the "invalid transaction object" if the specified transaction object doesn't exist.
	if ( *px == INVALID_XACT )
		*px = (XACT *)&InvalidXact;

	// Cache pointer to transaction object.
	x = *px;

	// If no gateway errors have been detected ...
	if ( x->GWErrCode == GWERR_NONE )
	{
		// If target is an iom (vs. the gateway) ...
		if ( IomPort < NUM_IOMPORTS )
		{
			// Cache pointer to iom's constant attributes.
			pIomAttr = x->mm->IomObj[IomPort].pIomAttr;

			// If a valid iom type is declared (and therefore required to be registered by the action that is being scheduled) ...
			if ( IomType != IOMTYPE_UNREG )
			{
				// Set error flag if the iom type is not registered.
				if ( pIomAttr == &IomAttrUnreg )
					x->GWErrCode = GWERR_IOMCLOSED | (u32)IomPort;

				// Set error flag if iom type does not match the registered type.
				else if ( pIomAttr->IomType != IomType )
					x->GWErrCode = GWERR_IOMTYPE | IomPort;
			}
		}

		// Otherwise target must be gateway, so set error flag if iom port number is illegal.
		else 
		{
			// The target is the gateway.
			pIomAttr = &IomAttr2601;

			// Set error code if iom port number is illegal.
			if ( IomPort != MODID_GATEWAY )
				x->GWErrCode = GWERR_BADVALUE | 2;

			// Set error code if iom type is illegal.
			else if ( ( IomType != 2601 ) && ( IomType != IOMTYPE_UNREG ) )
				x->GWErrCode = GWERR_BADVALUE | 3;
		}

		// If the specified iom is not connected to the transaction's active iom port, and there are no transaction errors yet ...
		if ( ( IomPort != x->SelectedIom ) && ( x->GWErrCode == GWERR_NONE ) )
		{
			// Designate this iom port as the active one.
			x->SelectedIom = IomPort;

			// Start new module command.  All iom status bits are relevant until an action scheduler indicates otherwise.
			GWBeginMCmd( x, IomPort, pIomAttr, STATMASK_NONE );
		}
	}

	// Return 0 if validation was successful.
	return x->GWErrCode;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Add a command argument value to PPCmdBuf[] that is needed for postprocessing.
// Returns pointer to argument in PPCmdBuf[].

static void *PushCmdArg( HXACT x, const void *src, int nbytes )
{
	u8 *dst = x->pPPCmdBuf;			// Get buffer NextWrite pointer.
	memcpy( dst, src, nbytes );		// Copy value to buffer.
	x->pPPCmdBuf += nbytes;			// Bump NextWrite pointer.
	return dst;						// Return value's location in buffer.
}

/////////////////////////////////////////////////////////////////////////////////////////
// Allocate and initialize a transaction object.
// Returns handle to transaction (pointer to XACT structure), or zero if error detected.

EXPORT(XACT *) S26_SchedOpen( HBD hbd, u32 retries )
{
	// Abort if illegal MM number.
	if ( hbd >= MMCount )
        return NULL;

    return XactAlloc( &obj[hbd], LOGDEV_GATEWAY, retries );
}

///////////////////////////////////////
// Common postprocessor functions.

// Copy block of data to application buffer.
static void memcpy_pp( POSTPROC *p )
{
	memcpy( p->pDstData, p->pSrcData, p->SrcSize );
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Null VFUNC for module registration and/or reset.  This is used by all modules that do not
// require middleware image maintenance or physical initialization upon registration or reset.

static void NullVFUNC( IOM *iom )
{
	// do nothing
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Endian correct all response values as the first step during post-processing.  A list of response
// specification items are contained in pArgSpec[].  Each item specifies the attributes of a group
// of contiguous values in the MRsp: endianness, size in bytes of each value (which must all be same
// size), and number of contiguous values.  The next item specifies the next contiguous value group
// in the MRsp.  Enough items are supplied to specify all values in the MRsp.  The attribute value
// zero indicates there are no more values in the MRsp.

static void endianfix_pp( POSTPROC *p )
{
	int				i;
	int				isLE;
	int				argsize;
	int				argcount;
	int				specSize;
	u8				*pSrc = p->pSrcData;
	u8				*pDst = p->pDstData;
	const ARGSPEC	*s;

	// For each group of response values that belong to a single specification ...
	for ( s = p->pArgSpec; *s != 0; s++ )
	{
		// Extract the group's specification.
		isLE		= ( ( *s & MASK_LITTLEENDIANFLAG ) != 0 );	// TRUE if little endian.
		argsize		= ( *s & MASK_ARGSIZE ) >> SHFT_ARGSIZE;	// Size of each value.
		argcount	= *s & MASK_ARGCOUNT;						// Number of consecutive values in response packet.
		specSize	= argsize * argcount;						// Byte count of all values in this specification.

		// Endian-correct (in place in response packet buffer) this response packet group's values based on the specification.
		switch ( argsize )
		{
		case 2:		// Words -----------------------

			if ( isLE )		// Little endian.
			{
				u16 *src = (u16 *)pSrc;
				for ( i = 0; i < argcount; i++, src++ )
					*src = LEtoH_16( *src );
			}
			else			// Big endian.
			{
				u16 *src = (u16 *)pSrc;
				for ( i = 0; i < argcount; i++, src++ )
					*src = BEtoH_16( *src );
			}

			break;

		case 4:		// DWords -----------------------

			if ( isLE )		// Little endian.
			{
				u32 *src = (u32 *)pSrc;
				for ( i = 0; i < argcount; i++, src++ )
					*src = LEtoH_32( *src );
			}
			else			// Big endian.
			{
				u32 *src = (u32 *)pSrc;
				for ( i = 0; i < argcount; i++, src++ )
					*src = BEtoH_32( *src );
			}

			break;
		}

		// Copy all corrected response values to their target locations.
		memcpy( pDst, pSrc, specSize );

		// Bump pointers for next specification.
		pSrc += specSize;
		pDst += specSize;
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////  GATEWAY ACTIONS  //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


// Specifications for response values ------------

#define NoEndian_KillMRsp						(const ARGSPEC *)KILL_MRSP

static const ARGSPEC BigEndian_16x1[]			= { BE( u16, 1 ), 0 };

static const ARGSPEC NoEndian_0x0[]				= { 0 };
static const ARGSPEC NoEndian_8x1[]				= { NE( u8, 1 ), 0 };
static const ARGSPEC NoEndian_8x2[]				= { NE( u8, 2 ), 0 };
static const ARGSPEC NoEndian_8x4[]				= { NE( u8, 4 ), 0 };
static const ARGSPEC NoEndian_DioStates[]		= { NE( u8, DIO_BYTECOUNT ), 0 };

static const ARGSPEC LittleEndian_16x1[]		= { LE( u16, 1 ), 0 };
static const ARGSPEC LittleEndian_32x1[]		= { LE( u32, 1 ), 0 };
static const ARGSPEC LittleEndian_32x1_16x1[]	= { LE( u32, 1 ), LE( u16, 1 ), 0 };
static const ARGSPEC LittleEndian_32x4_8x4[]	= { LE( u32, 4 ), LE( u8, 4 ), 0 };
static const ARGSPEC LittleEndian_AinData[]		= { LE( s16, NUM_AINS ), 0 };


////////////////////////////////
// Fetch active port flags.

// Scheduler.
EXPORT(GWERR) S26_Sched2601_GetLinkStatus( HXACT x, u16 *LinkFlags )
{
	// Select the iom.
	SelectIom( &x, MODID_GATEWAY, 2601 );

	// Schedule: Fetch the link status.
	return GWAddAction( x, OP_GETLINKSTATUS, NULL, ZERO_SIZE, LinkFlags, STATMASK_NONE, BigEndian_16x1 );
}

//////////////////////////////////
// Fetch safety interlock flags.

// Scheduler.
EXPORT(GWERR) S26_Sched2601_GetInterlocks( HXACT x, u8 *LockFlags )
{
	// Select the iom.
	SelectIom( &x, MODID_GATEWAY, 2601 );

	// Schedule: Fetch interlock status.
	return GWAddAction( x, OP_GETINTERLOCKS, NULL, ZERO_SIZE, LockFlags, STATMASK_NONE, NoEndian_8x1 );
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Set communication watchdog interval to the specified number of 100 millisecond increments.

// Scheduler.
EXPORT(GWERR) S26_Sched2601_SetWatchdog( HXACT x, u8 NumTenthSeconds )
{
	// Select the iom.
	SelectIom( &x, MODID_GATEWAY, 2601 );

	// Schedule: Set watchdog interval.
	return GWAddAction( x, OP_SETWATCHDOG, &NumTenthSeconds, sizeof(NumTenthSeconds), NULL, STATMASK_NONE, NoEndian_0x0 );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////  COMMON ACTIONS  /////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// Module NOP.  Can be used to fetch just the module status.

// Scheduler.
EXPORT(GWERR) S26_Sched2600_Nop( HXACT x, IOMPORT IomPort )
{
	// Select the iom.
	SelectIom( &x, IomPort, 0 );

	// Schedule the nop action for execution.
	return GWAddAction( x, OP_NOP, NULL, ZERO_SIZE, NULL, STATMASK_NONE, NoEndian_0x0 );
}

////////////////////////////////////////////////////////////////////
// Negate the specified MM or iom status flag bits (except HRST).
// Negated bits are masked off (ignored) in the enclosing MRsp.

// Postprocessor: reset specified flags in middleware-simulated iom status byte.
static void ClearSimStatus_pp( POSTPROC *p )
{
	// Negate the specified flags.
	p->iom->SimStatus &= ( ~( *p->pSrcData ) );
}

// Private scheduler.
static GWERR Sched2600_ClearStatus( HXACT x, IOMPORT IomPort, u8 BitMask )
{
	// Client is not permitted to reset the iom's HRST flag bit, so adjust the mask accordingly.
	u8 mask = (u8)( BitMask & ~STATUS_HRST );

	// Select the iom.  If there were no errors ...
	if ( SelectIom( &x, IomPort, 0 ) == GWERR_NONE )
	{
		// If middleware-simulated iom flags are to be negated ...
		if ( BitMask & STATUS_SIMULATED )
		{
			// Schedule post-processing step: reset specified flags in middleware-simulated iom status byte.  The destination
			// pointer and data size is not specified here as it is implicitly known to the post-processing function.
			GWAddPost( x, PushCmdArg( x, &BitMask, sizeof(BitMask) ), NULL, ZERO_SIZE, ClearSimStatus_pp, NULL );
		}

		// If physical iom flags are to be negated ...
		if ( BitMask & STATUS_PHYSICAL )
		{
			// Schedule: Issue the Clear Status command.  Ignore the masked flag(s) that will be returned in the iom status byte.
			GWAddAction( x, OP_RESETFLAGS, &mask, sizeof(mask), NULL, (u8)~mask, NoEndian_0x0 );
		}
	}

	// Return error code, if any.
	return x->GWErrCode;
}

// Public wrapper.
EXPORT(GWERR) S26_Sched2600_ClearStatus( HXACT x, IOMPORT IomPort, u8 BitMask )
{
	// Schedule the specified status bits.
	return Sched2600_ClearStatus( x, IomPort, BitMask );
}

////////////////////////////////////////
// Return iom product identifier.

// Scheduler.
EXPORT(GWERR) S26_Sched2600_IomGetProductID( HXACT x, IOMPORT IomPort, u16 *ProductID )
{
	// Select the iom.
	SelectIom( &x, IomPort, 0 );

	// Schedule the GetProductID action for execution.
	return GWAddAction( x, OP_GETPRODUCTID, NULL, ZERO_SIZE, ProductID, STATMASK_NONE, BigEndian_16x1 );
}

////////////////////////////////////////////////
// Return firmware revision for MM or any iom.

// Scheduler.
EXPORT(GWERR) S26_Sched2600_GetFirmwareVersion( HXACT x, IOMPORT IomPort, u16 *Version )
{
	// Select the iom.
	SelectIom( &x, IomPort, 0 );

	// Schedule the GetFirmwareVersion action for execution.
	return GWAddAction( x, OP_GETVERSION, NULL, ZERO_SIZE, Version, STATMASK_NONE, BigEndian_16x1 );
}

///////////////////////////////////////////////////////////////////////
// Return iom address shunt settings (if iom type has address shunts.

// Scheduler.
EXPORT(GWERR) S26_Sched2600_GetAddress( HXACT x, IOMPORT IomPort, u8 *adrs )
{
	// Select the iom.
	SelectIom( &x, IomPort, 0 );

	// Schedule the GetAddressShunts action for execution.
	return GWAddAction( x, OP_GETADDRESS, NULL, ZERO_SIZE, adrs, STATMASK_NONE, NoEndian_8x1 );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////  REGISTRATION, DETECTION AND RESET FUNCTIONS  ////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////
// Schedule detect-upon-reset actions for an iom.

static GWERR SchedDetectIom( HXACT x, IOMPORT IomPort, u16 *IomType, u16 *FwVersion )
{
	// Schedule negation of the iom's RST and CERR status bits (in case either or both are set).
	S26_Sched2600_ClearStatus( x, IomPort, STATUS_RST | STATUS_CERR );

	// Schedule a request for the iom's product identifier and firmware version.
	S26_Sched2600_IomGetProductID( x, IomPort, IomType );
	return S26_Sched2600_GetFirmwareVersion( x, IomPort, FwVersion );
}

////////////////////////
// Register one iom.

static void RegisterOneIom( IOM *iom, const u32 msec, const u16 IomType, const u16 FwVersion, u32 retries )
{
	// If the target iom port is not already registered for the detected iom type ...
	if ( iom->pIomAttr->IomType != IomType )
	{
		// Set the iom's ConstantAttributes pointer based on the declared iom type, and adjust the mm's iom count.
		switch ( IomType )
		{
		case 2608:	iom->pIomAttr = &IomAttr2608;	iom->mm->IomCount++;	break;
		case 2610:	iom->pIomAttr = &IomAttr2610;	iom->mm->IomCount++;	break;
		case 2612:	iom->pIomAttr = &IomAttr2612;	iom->mm->IomCount++;	break;
		case 2620:	iom->pIomAttr = &IomAttr2620;	iom->mm->IomCount++;	break;
		case 2631:	iom->pIomAttr = &IomAttr2631;	iom->mm->IomCount++;	break;
		case 2650:	iom->pIomAttr = &IomAttr2650;	iom->mm->IomCount++;	break;
		case 2652:	iom->pIomAttr = &IomAttr2652;	iom->mm->IomCount++;	break;
		case 2653:	iom->pIomAttr = &IomAttr2653;	iom->mm->IomCount++;	break;
		default:	iom->pIomAttr = &IomAttrUnreg;	iom->mm->IomCount--;	break;
		}

		// No middleware-simulated iom error flags are asserted for this iom.
		iom->SimStatus = STATUS_NONE;

		// Save the iom's firmware version number.
		iom->FwVersion = FwVersion;

		// Save the max allowed transaction time and retry count for the iom's initialization vfunc to execute.
		iom->MaxInitTime = msec;
		iom->MaxRetries  = retries;

		// Initialize the iom and its middleware image.
		iom->pIomAttr->IomRegister( iom );
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Invoke a hard reset on an iom.

#define NUM_RECONN_RETRIES	10		// Max retries to reconnect to iom after hard reset.

EXPORT(GWERR) S26_ResetIom( HBD hbd, IOMPORT IomPort, u32 msec, u32 retries )
{
    GWERR err;              // Error code.
	MM_OBJ	*mm;			// Pointer to the mm object to which the target iom is connected.
	IOM		*iom;			// Pointer to the target iom object.
	XACT	*x;				// Pointer to transaction control objects used for invoking reset and restoring iom communication.
	u16		InfoType;		// Detected iom type upon communication restoration.
	u16		InfoFwVersion;	// Detected iom firmware version upon communication restoration.
	u8		i;

	// Abort if board handle is not legal.
	if ( hbd >= MMCount )
        return GWERR_BADVALUE | 1;

	// Abort if IomPort is not legal.
	if ( IomPort >= NUM_IOMPORTS )
        return GWERR_BADVALUE | 2;

	// Cache pointers to the MM and IOM objects.
	mm  = &obj[hbd];
	iom = &mm->IomObj[IomPort];

	// Invoke the iom reset --------------------------------

	// Allocate a transaction object.
	x = XactAlloc( mm, LOGDEV_GATEWAY, retries );

	// Select target module.
	SelectIom( &x, IomPort, 0 );

	// Schedule: Invoke the iom hard reset.
	GWAddAction( x, OP_HARDRESET, NULL, ZERO_SIZE, NULL, STATMASK_NONE, NoEndian_KillMRsp );

	// Execute transaction.
	err = S26_SchedExecute( x, msec, NULL );
    if ( err != GWERR_NONE )
        return err;

	// Unconditionally unregister the iom if it is registered.
	RegisterOneIom( iom, 0, IOMTYPE_UNREG, 0, retries );

	// Re-establish communication with the target iom ---------------------

	// Repeat up to the max allowed retries ...
	for ( i = 0; i < NUM_RECONN_RETRIES; i++ )
	{
		// Attempt to fetch the iom's info and negate its RST flag.
		x = XactAlloc( mm, LOGDEV_GATEWAY, retries );				// Allocate a transaction object.

		SchedDetectIom( x, IomPort, &InfoType, &InfoFwVersion );	// Schedule: Negate RST, fetch product id and fw version.

		err = S26_SchedExecute( x, msec, NULL );				// Execute transaction.

		// If info was successfully fetched (thereby indicating restored communications) ...
		if ( err == GWERR_NONE )
		{
			// Re-register the iom that was unregistered by the reset, then exit loop.
			RegisterOneIom( iom, msec, InfoType, InfoFwVersion, retries );
			break;
		}
	}

	// Return error code, if any.
	return err;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////  MODEL 2608 ANALOG I/O BOARD ACTIONS  ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////
// Read eeprom byte value.

// Scheduler.
EXPORT(GWERR) S26_Sched2608_ReadEeprom( HXACT x, IOMPORT IomPort, u8 address, u8 *value )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2608 );

	// Schedule the ReadEeprom action for execution.
	return GWAddAction( x, OP_AIO_READEPROM, &address, sizeof(u8), value, STATMASK_NONE, NoEndian_8x1 );
}

//////////////////////////////////////////////////////////////////////////////////
// Write eeprom byte value.
// Note: this executes in a private transaction, vs. scheduling an action.

EXPORT(GWERR) S26_2608_WriteEeprom( HBD hbd, IOMPORT IomPort, u32 msec, u8 address, u8 value, u32 retries )
{
	XACT	*x;
	u8		arg[2];

	// Abort if board handle is not legal.
	if ( hbd >= MMCount )
		return GWERR_BADVALUE | 1;

	// Abort if IomPort is not legal.
	if ( IomPort >= NUM_IOMPORTS )
		return GWERR_BADVALUE | 2;

	// Populate the action argument list.
	arg[0] = address;
	arg[1] = value;

	// Open a new transaction object.
	x = XactAlloc( &obj[hbd], LOGDEV_GATEWAY, retries );

	// Select the iom.
	SelectIom( &x, IomPort, 2608 );

	// Schedule the WriteEeprom action for execution.
	GWAddAction( x, OP_AIO_WRITEEPROM, &arg, sizeof(arg) / sizeof(arg[0]), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Execute and release the transaction object.  Abort if transaction failed.
	return S26_SchedExecute( x, msec, NULL );
}

///////////////////////////////////////////////////
// Read all eeprom values from 2608 board.
// Returns TRUE if successful.

static BOOL s2608_GetEepromImage( IOM *iom, u8 *image, u32 msec, u32 retries )
{
	u8		sum;
	u8		*eedata;
	XACT	*x;
	int		i;

	// Read all eeprom values from the 2608 into image buffer ------------------

	// Open new transaction.
	x = XactAlloc( iom->mm, LOGDEV_GATEWAY, retries );

	// Schedule eeprom reads for all calibration and info values.
	for ( i = 0, eedata = image; i <= EEADRS_CHECKSUM; i++ )
		S26_Sched2608_ReadEeprom( x, iom->IomPort, (u8)i, eedata++ );

	// Execute and release the transaction object.  Abort if transaction failed.
	if ( S26_SchedExecute( x, msec, NULL ) != GWERR_NONE )
		return FALSE;
	
	// Validate the eeprom checksum ------------------------------------------

	// Compute the checksum.
	for ( i = 0, sum = 0, eedata = image; i < EEADRS_CHECKSUM; i++ )
		sum = (u8)( sum + *eedata++ );

	// Abort if checksum is not valid.
	if ( sum != *eedata )
		return FALSE;

	// Correct byte ordering for all image values -----------------------------

	// Voltage reference values.
	EE_10V		= LEtoH_32( EE_10V );
	EE_100MV	= LEtoH_32( EE_100MV );

	// Dac standardization offset/scalar pairs.
	for ( i = 0; i < MAX_NUM_AOUTS; i++ )
	{
		EE_DACOS(i)	= LEtoH_16( EE_DACOS(i) );
		EE_DACSC(i)	= LEtoH_32( EE_DACSC(i) );
	}

	// TempRef calibration offsets.
	for ( i = 0; i < MAX_NUM_TREFS; i++ )
		EE_TREFOS(i) = LEtoH_16( EE_TREFOS(i) );

	// Abort if any value is out of tolerance ------------------------------

	// Abort if dac count is not valid.
	if ( ( EE_NUMDACS != 0 ) && ( EE_NUMDACS != 4 ) &&  ( EE_NUMDACS != 8 ) )
		return FALSE;

	// Abort if any adc reference value is out of tolerance.
	if ( ( EE_10V < ADC_SCALAR_MIN * 10 )
		|| ( EE_10V > ADC_SCALAR_MAX * 10 )
		|| ( EE_100MV < ADC_SCALAR_MIN / 10 )
		|| ( EE_100MV > ADC_SCALAR_MAX / 10 ) )
	{
		return FALSE;
	}

	// Abort if any dac value is out of tolerance.
	for ( i = 0; i < EE_NUMDACS; i++ )
	{
		if ( ( EE_DACOS(i) < -DAC_OFFSET_TOL )
			|| ( EE_DACOS(i) > DAC_OFFSET_TOL )
			|| ( EE_DACSC(i) < DAC_SCALAR_MIN )
			|| ( EE_DACSC(i) > DAC_SCALAR_MAX ) )
		{
			return FALSE;
		}
	}

	// Abort if any tref offset value is out of tolerance.
	for ( i = 0; i < MAX_NUM_TREFS; i++ )
	{
		if ( ( EE_TREFOS(i) < -TREF_OFFSET_TOL ) || ( EE_TREFOS(i) > TREF_OFFSET_TOL ) )
			return FALSE;
	}

	// Indicate success.
	return TRUE;
}

////////////////////////////////////////////////////
// Return analog output channel setpoint in volts.

// Postprocessor: Convert raw binary dac value to volts.
// Assumes:
//   p->pSrcData points to the normalization parameters for the target dac.
//   x->PPRspBuf contains raw dac setpoint in iom byte order.
static void GetAout_pp( POSTPROC *p )
{
	// Normalize the raw setpoint, convert to volts and store to application buffer.
	*(DOUBLE *)p->pDstData = (DOUBLE)( *(s16 *)p->x->PPRspBuf - ( (DACREF *)p->pSrcData )->std_offset ) / ( ( (DACREF *)p->pSrcData )->std_scalar * 3276.8 );
}

// Scheduler.
EXPORT(GWERR) S26_Sched2608_GetAout( HXACT x, IOMPORT IomPort, u8 chan, DOUBLE *volts )
{
	// Select the iom.
	if ( SelectIom( &x, IomPort, 2608 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal command values.
	if ( chan >= x->mm->IomObj[IomPort].Custom.Obj2608.NumDacs )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Schedule the iom action: fetch dac setpoint (which is a u16) into transaction buffer.
	GWAddAction( x, OP_AIO_GETOUTPUT, &chan, sizeof(u8), x->PPRspBuf, STATMASK_NONE, LittleEndian_16x1 );

	// Schedule postprocess: adjust byte order, convert to volts, store to app buffer.  Since the data source address
	// is implicitly known to GetAout_pp(), we will set this GWAddPost()'s data source so it points to the normalization
	// parameters for this dac channel.
	return GWAddPost( x, &x->mm->IomObj[IomPort].Custom.Obj2608.dac_std[chan], volts, IMPLICIT_SIZE, GetAout_pp, 0 );
}

////////////////////////////////////////////////////
// Program analog output channel setpoint in volts.

#define SIZEOF_AOUTACTION	3

// Scheduler: Program a dac setpoint.
EXPORT(GWERR) S26_Sched2608_SetAout( HXACT x, IOMPORT IomPort, u8 chan, DOUBLE volts )
{
	DOUBLE	dblval;
	s16		binval;
	u8		a[SIZEOF_AOUTACTION];
	DACREF	*dacref = &x->mm->IomObj[IomPort].Custom.Obj2608.dac_std[chan];

	// Select the iom.
	if ( SelectIom( &x, IomPort, 2608 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal command values.
	if ( chan >= x->mm->IomObj[IomPort].Custom.Obj2608.NumDacs )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Compute the corrected setpoint value for the target dac.
	dblval = volts * 3276.8 * dacref->std_scalar + dacref->std_offset;

	// Populate the corrected dac value.
	if ( dblval < -32768.0 )		binval = -32768;
	else if ( dblval > 32767.0 )	binval = 32767;
	else							binval = (s16)dblval;

	// Populate the action's argument list.
	a[0]			= chan;					// Analog input channel number
	*(s16 *)&a[1]	= HtoLE_16( binval );	// Corrected dac data value in iom byte order.

	// Schedule the SetAnalogOutput action for execution.
	return GWAddAction( x, OP_AIO_SETOUTPUT, a, SIZEOF_AOUTACTION, NULL, STATMASK_NONE, NoEndian_0x0 );
}

////////////////////////////////////////////////////////////////////////////////////////
// Get digitized reference standards.  The raw standards values are stored internally
// (for later use) and are also stored to app buffer if the host requested them.

// Helper: compute real-time calibration "constants".  This is used by both the GetCalData postprocessor
// and the 2608 initialization function (to set up default cal constants).
static void ComputeCalValues( IOM_2608 *iom2608 )
{
	int		i;
	int		ChansPerRefTemp;					// Number of analog input channels associated with each reference temperature sensor.
	short	raw10v;								// Raw adc counts for 10V reference.
	short	raw100mv;							// Raw adc counts for 100mV reference.
	DOUBLE	RefTemp[MAX_NUM_TREFS];				// The onboard reference temperatures in degrees C.
	s16		*raw = iom2608->RawRefData;			// Standards raw adc data source buffer.

	// Cache raw adc counts.
	raw10v		= raw[ INDEX_REF_LG_10V]   - raw[ INDEX_REF_LG_0V];
	raw100mv	= raw[ INDEX_REF_HG_100MV] - raw[ INDEX_REF_HG_0V];

	// Compute and store adc normalization scalars.  These values, when multiplied by an external channel's raw adc value
	// (minus the appropriate offset value), will produce a corrected voltage.  A scalar is set to zero if there is a
	// reference voltage digitization problem.
	iom2608->NormScalar10v		= ( raw10v   == 0 ) ? 0.0 : ( iom2608->RefVolts_10v   / (DOUBLE)raw10v );
	iom2608->NormScalar100mv	= ( raw100mv == 0 ) ? 0.0 : ( iom2608->RefVolts_100mv / (DOUBLE)raw100mv );

	// Compute and store all physical reference junction temperatures.
	for ( i = 0; i < iom2608->NumTemps; i++ )
		RefTemp[i] = (DOUBLE)( raw[ INDEX_REF_T0 + i] - iom2608->TRefOffset[i] - raw[ INDEX_REF_LG_0V] ) * iom2608->NormScalar10v * 100.0;

	// Compute the distribution of reference temperature sensors to external input channels.  The references
	// are evenly distributed among the external inputs regardless of the number of reference sensors.
	ChansPerRefTemp = NUM_AINS / iom2608->NumTemps;

	// Store all logical reference junction temperatures.
	for ( i = 0; i < NUM_AINS; i++ )
		iom2608->RefTemp[i] = RefTemp[ i / ChansPerRefTemp ];
}

// Postprocessor.
static void GetCalData_pp( POSTPROC *p )
{
	int			i;
	s16			*src		= (s16 *)( p->pSrcData );		// Pointer to source data.
	IOM_2608	*iom2608	= &p->iom->Custom.Obj2608;		// Pointer to iom object.
	s16			*raw		= iom2608->RawRefData;			// Internal destination buffer for correction values.
	const int	*px			= iom2608->pRefIndex;			// List of offsets of raw data values within RawRefData[].

	// Copy raw voltage reference data values to internal storage in standardized order (independent of firmware version).
	raw[INDEX_REF_LG_0V]		= src[ px[INDEX_REF_LG_0V] ];
	raw[INDEX_REF_HG_0V]		= src[ px[INDEX_REF_HG_0V] ];
	raw[INDEX_REF_LG_10V]		= src[ px[INDEX_REF_LG_10V] ];
	raw[INDEX_REF_HG_100MV]		= src[ px[INDEX_REF_HG_100MV] ];

	// Copy raw onboard temperature sensor values to internal storage in standardized order (independent of firmware version).
	for ( i = 0; i < iom2608->NumTemps; i++ )
		raw[INDEX_REF_T0 + i] = src[ px[INDEX_REF_T0 + i] ];

	// If necessary, increase full-scale reference standards raw adc counts to their minimum legal values.
	if ( raw[INDEX_REF_LG_10V]   < MIN_CLB )		raw[INDEX_REF_LG_10V]   = MIN_CLB;
	if ( raw[INDEX_REF_HG_100MV] < MIN_CLB )		raw[INDEX_REF_HG_100MV] = MIN_CLB;

	// Copy raw data values to application buffer if one is declared.  Values are copied in standardized order (independent of firmware version).
	if ( p->pDstData )
		memcpy( p->pDstData, raw, iom2608->NumRefs * sizeof(s16) );

	// Compute and store all real-time calibration values.
	ComputeCalValues( iom2608 );
}

// Scheduler.  If caldata points to app buffer (i.e., non-zero), raw cal data will be stored to the buffer.
EXPORT(GWERR) S26_Sched2608_GetCalData( HXACT x, IOMPORT IomPort, s16 *caldata )
{
	static ARGSPEC as[2];

	// Select the iom.
	if ( SelectIom( &x, IomPort, 2608 ) != GWERR_NONE )
		return x->GWErrCode;

	// Specify attributes of response arguments.
	as[0] = LE( s16, x->mm->IomObj[IomPort].Custom.Obj2608.NumRefs );
	as[1] = 0;

	// Schedule: Fetch raw digitized standards data into temporary buffer.
	GWAddAction( x, OP_AIO_GETINTERNALS, NULL, ZERO_SIZE, x->PPRspBuf, STATMASK_NONE, as );

	// Schedule: Convert raw adc values to host byte order, then convert to floating point form and store in
	// the MM object for later use (e.g., when measuring external channels).
	return GWAddPost( x, x->PPRspBuf, caldata, IMPLICIT_SIZE, GetCalData_pp, 0 );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set the temperature units to degrees C or F for all analog input channels that are connected to thermocouples.

EXPORT(GWERR) S26_Sched2608_SetTempUnits( HXACT x, IOMPORT IomPort, int DegreesF )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2608 );

	// Schedule: Store the units spec to middleware image.
	return GWAddPost( x, PushCmdArg( x, &DegreesF, sizeof(int) ), &x->mm->IomObj[ IomPort ].Custom.Obj2608.UseDegreesF, sizeof(int), memcpy_pp, 0 );
}

///////////////////////////////////////////////////////////////////////////////////////
// Convert a raw adc value to volts.  This is executed immediately (vs. scheduled).

EXPORT(DOUBLE) S26_2608_AdcCorrect( HBD hbd, IOMPORT IomPort, s16 RawData, int IsHighGain )
{
	IOM_2608 *aio;

	// If board handle and iom port are both legal ...
	if ( ( hbd < MMCount ) && ( IomPort < NUM_IOMPORTS ) )
	{
		// If registered iom type is 2608 ...
		if ( obj[hbd].IomObj[IomPort].pIomAttr->IomType == 2608 )
		{
			// Cache pointer to iom object.
			aio = &obj[hbd].IomObj[IomPort].Custom.Obj2608;

			// Convert raw adc data to a corrected voltage and return it.
			return (DOUBLE)( RawData - aio->RawRefData[ IsHighGain ? INDEX_REF_HG_0V : INDEX_REF_LG_0V ] )	// Subtract raw offset counts for this range.
				* ( IsHighGain ? aio->NormScalar100mv : aio->NormScalar10v );								// Normalize against reference voltage standard and convert to volts.
		}
	}

	// Return default value in case of error.
	return 0.0;
}

///////////////////////////////////////////////////////////
// Get adc data from all external channels.

typedef struct TC_PARAM		TC_PARAM;
typedef struct SENSOR_ATTR	SENSOR_ATTR;

struct TC_PARAM {					// THERMOCOUPLE LINEARIZATION TABLE ---------
	int				npoints;		//   Number of entries in table.
	DOUBLE			uvstep;			//   Distance between table entries, in volts.
	DOUBLE			startv;			//   Voltage (in V) of first table entry.
	DOUBLE			endv;			//   Voltage (in V) of last table entry.
	const DOUBLE	*table;			//   Pointer to table.  Table values are expressed in degrees C.
};

#include "tctable.h"				// Import the thermocouple linearization tables.

struct SENSOR_ATTR {				// ANALOG INPUT TYPE ATTRIBUTES ----------
	u16				gain;			//   High Gain bit (in msb).
	u32				OffsetIndex;	//   Index to iom's RawRefData[] for the 0V/0mV standard, depending on input range.
	u32				GainPosn;		//   Byte offset from base of IOM2608 where the normalization scalar can be found for this input range.
	DOUBLE			multiplier;		//   Multiplier for conversion to output units.
	TC_PARAM		*tcparam;		//   Pointer to Thermocouple Parameter Table, or zero if not a thermocouple type.
};

#define TC_BASE_TYPE	TC_B_TYPE	// First enumerated thermocouple type.

// Map enumerated analog input type to its associated attributes.
static const struct SENSOR_ATTR sensor_attr[] =
{ //  gain,   offset           scalar								         mult     tctable
	{ 0x0000, INDEX_REF_LG_0V, offsetof( IOM_2608, NormScalar10v   ), 3276.8,  0												},	// RAW_LG_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 3276.8,  0												},	// RAW_HG_TYPE
	{ 0x0000, INDEX_REF_LG_0V, offsetof( IOM_2608, NormScalar10v   ), 1.0,     0												},	// V_10_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 1.0,     0												},	// V_001_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 100e-2,  &tc_params_table[ TC_B_TYPE - TC_BASE_TYPE ]	},	// TC_B_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 100e-2,  &tc_params_table[ TC_C_TYPE - TC_BASE_TYPE ]	},	// TC_C_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 100e-2,  &tc_params_table[ TC_E_TYPE - TC_BASE_TYPE ]	},	// TC_E_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 100e-2,  &tc_params_table[ TC_J_TYPE - TC_BASE_TYPE ]	},	// TC_J_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 100e-2,  &tc_params_table[ TC_K_TYPE - TC_BASE_TYPE ]	},	// TC_K_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 100e-2,  &tc_params_table[ TC_N_TYPE - TC_BASE_TYPE ]	},	// TC_N_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 100e-2,  &tc_params_table[ TC_R_TYPE - TC_BASE_TYPE ]	},	// TC_R_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 100e-2,  &tc_params_table[ TC_S_TYPE - TC_BASE_TYPE ]	},	// TC_S_TYPE
	{ 0x8000, INDEX_REF_HG_0V, offsetof( IOM_2608, NormScalar100mv ), 100e-2,  &tc_params_table[ TC_T_TYPE - TC_BASE_TYPE ]	}	// TC_T_TYPE
};

// Postprocessor.
static void GetAins_pp( POSTPROC *p )
{
	int			chan;										// Analog input channel number.
	DOUBLE		volts;										// Corrected analog input voltage.
	DOUBLE		degreesC;									// Temperature in degrees C.
	s16			*src		= (s16 *)p->pSrcData;			// Raw adc source data buffer.
	DOUBLE		*dest		= (DOUBLE *)p->pDstData;		// Application buffer that will receive the post-processed adc data.
	IOM_2608	*iom2608	= &p->iom->Custom.Obj2608;		// Cached pointer to iom object.

	// For each analog input channel ...
	for ( chan = 0; chan < NUM_AINS; chan++, dest++ )
	{
		// Cache pointer to attributes for this input type.
		const SENSOR_ATTR *s = &sensor_attr[ iom2608->SensType[chan] ];

		// Convert raw adc data to host byte order (if necessary), and then to a corrected voltage.
		volts = (DOUBLE)( src[chan] - iom2608->RawRefData[ s->OffsetIndex ] )	// Subtract raw offset counts for this range.
			* ( *(DOUBLE *)( (u8 *)iom2608 + s->GainPosn ) )					// Normalize against standard and convert to volts.
			* s->multiplier;													// Scale to specialized output units (if necessary).

		// If analog input type is thermocouple, convert measured voltage to temperature and store temperature to application buffer.
		if ( s->tcparam )
		{
			int				index;					// Linearization table discrete index.
			DOUBLE			cindex;					// Linearization table continuous index.
			DOUBLE			utemp;					// Uncompensated tc temperature.
			const TC_PARAM	*tcp = s->tcparam;		// Pointer to thermocouple attributes.

			// Compute continuous table index.
			cindex = ( volts - tcp->startv ) / tcp->uvstep;

			// Compute discrete table index.
			index = (int)cindex;

			// Return table boundary temperature if table index is too large or small, otherwise return interpolated temperature.
			if ( index >= tcp->npoints - 1 )	utemp = tcp->table[ tcp->npoints - 1 ];
			else if ( index < 0 )				utemp = tcp->table[0];
			else								utemp = tcp->table[index] + ( tcp->table[index + 1] - tcp->table[index] ) * ( cindex - index );

			// Cold-junction compensate.
			degreesC = utemp + iom2608->RefTemp[chan];

			// Store temperature to application buffer.
			*dest = iom2608->UseDegreesF ? ( 1.8 * degreesC + 32.0 ) : degreesC;
		}

		// If analog input type is voltage, store measured voltage to application buffer.
		else
			*dest = volts;
	}
}

// Scheduler.
EXPORT(GWERR) S26_Sched2608_GetAins( HXACT x, IOMPORT IomPort, DOUBLE *volts, BOOL Integrated )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2608 );

	// Schedule: Fetch raw binary adc data for all chans.
	GWAddAction( x, (u8)( Integrated ? OP_AIO_GETINTEGRATEDS : OP_AIO_GETSNAPSHOTS ), NULL, ZERO_SIZE, x->PPRspBuf, STATMASK_NONE, LittleEndian_AinData );
	
	// Schedule: Convert binary adc values to host byte order and map to engineering units, then copy to application buffer
	return GWAddPost( x, x->PPRspBuf, volts, IMPLICIT_SIZE, GetAins_pp, 0 );
}

//////////////////////////////////////////////////////////////
// Set input types for all external channels.  Illegal input
// type will force the associated channel to the default type.

// Scheduler.
EXPORT(GWERR) S26_Sched2608_SetAinTypes( HXACT x, IOMPORT IomPort, const u8 *types )
{
	int			chan;					// Analog input channel number.
	const u8	*ptypes		= types;	// Analog input types.
	u16			gain		= 0;		// Gain flags in host byte order.
	u16			nordGain;				// Gain flags in 2608 byte order.
	IOM_2608	*iom2608;

	// Select the iom.
	if ( SelectIom( &x, IomPort, 2608 ) != GWERR_NONE )
		return x->GWErrCode;

	// Cache pointer to the 2608 object.
	iom2608 = &x->mm->IomObj[ IomPort ].Custom.Obj2608;

	// For each analog input channel ...
	for ( chan = 0; chan < NUM_AINS; chan++, ptypes++ )
	{
		// Validate the input type.  Abort if type is not legal.
		if ( *ptypes >= ( sizeof(sensor_attr) / sizeof(sensor_attr[0]) ) )
		{
			x->GWErrCode = GWERR_BADVALUE | 3;
			return x->GWErrCode;
		}

		// Shift the previously processed gain bits to make space for this new one, then set this channel's
		// Gain flag if it is to be measured at high gain.
		gain = (u16)( ( gain >> 1 ) | sensor_attr[ *ptypes ].gain );
	}

	// Arrange gain flags in 2608 byte order.
	nordGain = HtoLE_16( gain );

	// Schedule the SetAnalogInputTypes action for execution.
	GWAddAction( x, OP_AIO_SETINPUTRANGES, &nordGain, sizeof(nordGain), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Schedule: Store gain flags to middleware image.
	GWAddPost( x, PushCmdArg( x, &gain, sizeof(gain) ), &iom2608->GainFlags, sizeof(gain), memcpy_pp, 0 );

	// Schedule: Store the types to middleware image.
	return GWAddPost( x, PushCmdArg( x, types, NUM_AINS ), iom2608->SensType, NUM_AINS, memcpy_pp, 0 );
}

//////////////////////////////////////////////////////////////////////////////
// Get input types for all external adc channels.  These are the type values
// that are stored by the dll, which are validated against the gain flags
// that are returned from the 2608 in response to a OP_AIO_GETINPUTRANGES
// action.  There are two possible reasons for the returned input type
// differing from the programmed type: (1) an illegal type was specified,
// or (2) the 2608 was unexpectedly reset.  In either case, all stored dll
// types will be reset to their default values when this function is invoked.

// Postprocessor.
static void GetAinTypes_pp( POSTPROC *p )
{
	int		chan;
	u16		mask;

	// No InputType/GainFlags discrepancies detected yet.
	int nTypeErrors = 0;

	// Cache pointer to the 2608 object.
	IOM_2608 *iom2608 = &p->iom->Custom.Obj2608;

	// Cache gain flags.
	u16 GainFlags = *(u16 *)p->pSrcData;

	// For each analog input channel ...
	for ( chan = 0, mask = 1; chan < NUM_AINS; chan++, mask <<= 1 )
	{
		// Bump error count if input type doesn't agree with the gain flag.
		if ( sensor_attr[ iom2608->SensType[chan] ].gain != ( ( GainFlags & mask ) << ( 15 - chan ) ) )
			nTypeErrors++;
	}

	// If an input type error was detected then reset all input channels to the default type.
	if ( nTypeErrors > 0 )
	{
		for ( chan = 0; chan < NUM_AINS; chan++ )
			iom2608->SensType[chan] = 0;
	}

	// Copy all input types to the application buffer.
	memcpy( p->pDstData, p->iom->Custom.Obj2608.SensType, NUM_AINS );
}

// Scheduler.  Fetch input types for all analog input channels.
EXPORT(GWERR) S26_Sched2608_GetAinTypes( HXACT x, IOMPORT IomPort, u8 *types )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2608 );

	// Schedule: Fetch gain flags for all analog input chans.
	GWAddAction( x, OP_AIO_GETINPUTRANGES, NULL, ZERO_SIZE, x->PPRspBuf, STATMASK_NONE, LittleEndian_16x1 );
	
	// Schedule: Postprocess gain flags by validating against declared types, then copy types to application buffer
	return GWAddPost( x, x->PPRspBuf, types, IMPLICIT_SIZE, GetAinTypes_pp, 0 );
}

///////////////////////////////
// Set 50/60 Hz rejection.

// Scheduler: freq is the enumerated line frequency: 0 = 60 Hz, 1 = 50 Hz.
EXPORT(GWERR) S26_Sched2608_SetLineFreq( HXACT x, IOMPORT IomPort, u8 freq )
{
	// Select the iom.
	if ( SelectIom( &x, IomPort, 2608 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal command values.
	if ( freq > 1 )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Schedule the SetLineFreq action for execution.
	return GWAddAction( x, OP_AIO_SETFSTANDARD, &freq, sizeof(freq), NULL, STATMASK_NONE, NoEndian_0x0 );
}

//////////////////////////////////////////////
// VFUNC: Initialize board image upon reset.

static void s2608_IomReset( IOM *iom )
{
	int			i;
	IOM_2608	*iom2608 = &iom->Custom.Obj2608;

	// Default units are degrees C.
	iom2608->UseDegreesF = FALSE;

	// All analog input channels are set to default gain.
	iom2608->GainFlags = 0;

	// Set to default all analog input channel types.
	for ( i = 0; i < NUM_AINS; i++ )
		iom2608->SensType[i] = RAW_LG_TYPE;
}

//////////////////////////////////////////////////////////////
// VFUNC: Initialize board and its image upon registration.

static void s2608_IomRegister( IOM *iom )
{
	int			i;
	u8			image[EEADRS_CHECKSUM + 1];
	XACT		*x;
	IOM_2608	*iom2608 = &iom->Custom.Obj2608;

	// Standardized ordering for raw reference adc data values.  Each member specifies the index
	// of the commented reference value in the data array returned by OP_AIO_GETINTERNALS.
	// Ordering for 2608 firmware versions < 3.00:
	static const int RefIndexPreV3[] = {
		0,		// INDEX_REF_LG_0V
		1,		// INDEX_REF_LG_10V,
		2,		// INDEX_REF_HG_0V,
		3,		// INDEX_REF_HG_100MV,
		5,		// INDEX_REF_T0,
		7,		// INDEX_REF_T1,
	};

	// Ordering for 2608 firmware versions >= 3.00:
	static const int RefIndexPostV3[] = {
		0,		// INDEX_REF_LG_0V
		1,		// INDEX_REF_LG_10V,
		2,		// INDEX_REF_HG_0V,
		3,		// INDEX_REF_HG_100MV,
		4,		// INDEX_REF_T0,
		5,		// INDEX_REF_T1,
		6,		// INDEX_REF_T2,
		7,		// INDEX_REF_T3,
		8,		// INDEX_REF_T4,
		9,		// INDEX_REF_T5,
		10,		// INDEX_REF_T6,
		11,		// INDEX_REF_T7
	};

	// Initialize iom's configuration image.
	s2608_IomReset( iom );

	// Fetch and validate the 2608's eeprom image.  If the eeprom image was not successfully acquired ...
	if ( !s2608_GetEepromImage( iom, image, iom->MaxInitTime, iom->MaxRetries ) )
	{
		// Force default adc reference values.
		EE_10V		= (s32)( 10.0 * ANA_SCALAR_NORM );
		EE_100MV	= (s32)(  0.1 * ANA_SCALAR_NORM );

		// Force default dac offset/scalar standardization values.
		for ( i = 0; i < MAX_NUM_AOUTS; i++ )
		{
			EE_DACOS(i)	= 0;
			EE_DACSC(i)	= ANA_SCALAR_NORM;
		}

		// Force default tref offset values.
		for ( i = 0; i < MAX_NUM_TREFS; i++ )
			EE_TREFOS(i) = 0;

		// Force default dac count.
		EE_NUMDACS = 0;

		// Set a middleware-simulated iom error flag to indicate we are using default calibration constants.
		iom->SimStatus |= STATUS_2608_CALERR;
	}

	// Declare number of onboard temperature sensors: MAX_NUM_TREFS sensors if v3.00 or higher, otherwise 2.
	iom2608->NumTemps	= ( iom->FwVersion < 300 ) ? 2 : MAX_NUM_TREFS;
	iom2608->NumRefs	= ( iom->FwVersion < 300 ) ? NUM_REFS_PRE_V300 : NUM_REFS_POST_V300;
	iom2608->pRefIndex	= ( iom->FwVersion < 300 ) ? RefIndexPreV3 : RefIndexPostV3;

	// Compute adc reference voltage values, in Volts, and save for later standardizations when acquiring external analog input channels.
	iom2608->RefVolts_10v	= (DOUBLE)EE_10V   / (DOUBLE)ANA_SCALAR_NORM;
	iom2608->RefVolts_100mv	= (DOUBLE)EE_100MV / (DOUBLE)ANA_SCALAR_NORM;

	// Save reference temperature offsets for later standardizations.
	for ( i = 0; i < MAX_NUM_TREFS; i++ )
		iom2608->TRefOffset[i] = EE_TREFOS(i);

	// Declare number of populated analog output channels.
	iom2608->NumDacs = (int)EE_NUMDACS;

	// Store all dac correction parameters and program all dacs to zero volts, taking into account their correction parameters.
	if ( iom2608->NumDacs > 0 )
	{
		x = XactAlloc( iom->mm, LOGDEV_GATEWAY, iom->MaxRetries );
		for ( i = 0; i < iom2608->NumDacs; i++ )
		{
			iom2608->dac_std[i].std_offset	= EE_DACOS(i);
			iom2608->dac_std[i].std_scalar	= (DOUBLE)EE_DACSC(i) / (DOUBLE)ANA_SCALAR_NORM;
			S26_Sched2608_SetAout( x, iom->IomPort, (u8)i, 0.0 );
		}
		S26_SchedExecute( x, iom->MaxInitTime, NULL );
	}

	// Simulate the first calibration cycle (assumes no digitized reference data is acquired yet) --------------------

	// Simulate raw adc acquisition for all reference standards and onboard temperature sensors.
	iom2608->RawRefData[ INDEX_REF_LG_0V    ] = 0;
	iom2608->RawRefData[ INDEX_REF_LG_10V   ] = MIN_CLB;
	iom2608->RawRefData[ INDEX_REF_HG_0V    ] = 0;
	iom2608->RawRefData[ INDEX_REF_HG_100MV ] = MIN_CLB;
	for ( i = 0; i < iom2608->NumRefs; i++ )
		iom2608->RawRefData[ INDEX_REF_T0 + i ] = 0;

	// Compute all calibration values as if we just acquired the digitized reference standards.
	ComputeCalValues( iom2608 );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////  MODEL 2610 DIGITAL I/O BOARD ACTIONS  ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Return/set the operating modes ( PWM vs. Normal) for channels 0-7.

// Scheduler (set modes).
EXPORT(GWERR) S26_Sched2610_SetModes( HXACT x, IOMPORT IomPort, u8 *modes )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2610 );

	// Schedule the SetModes action for execution.
	return GWAddAction( x, OP_DIO_SETMODES, modes, sizeof(u8), NULL, STATMASK_NONE, NoEndian_0x0 );
}

// Scheduler (get modes).
EXPORT(GWERR) S26_Sched2610_GetModes( HXACT x, IOMPORT IomPort, u8 *modes )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2610 );

	// Schedule the GetModes action for execution.
	return GWAddAction( x, OP_DIO_GETMODES, NULL, ZERO_SIZE, modes, STATMASK_NONE, NoEndian_8x1 );
}

///////////////////////////////////////////////////////////////////////////
// Return/set the operating modes ( PWM vs. Normal) for channels 0-31.
// Valid only for 2610 firmware version 1.02 and higher.

// Scheduler (set modes).
EXPORT(GWERR) S26_Sched2610_SetModes32( HXACT x, IOMPORT IomPort, u8 *modes )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2610 );

	// Schedule the SetModes action for execution.
	return GWAddAction( x, OP_DIO_SETMODES32, modes, 4, PVOID, STATMASK_NONE, NoEndian_0x0 );
}

// Scheduler (get modes).
EXPORT(GWERR) S26_Sched2610_GetModes32( HXACT x, IOMPORT IomPort, u8 *modes )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2610 );

	// Schedule the GetModes action for execution.
	return GWAddAction( x, OP_DIO_GETMODES32, PVOID, ZERO_SIZE, modes, STATMASK_NONE, NoEndian_8x4 );
}

/////////////////////////////////////////////////////////////////////////////////
// Return/set the PWM on/off times for a channel that operates in the PWM mode.

// Scheduler.
EXPORT(GWERR) S26_Sched2610_SetPwmRatio( HXACT x, IOMPORT IomPort, u8 chan, u8 OnTime, u8 OffTime )
{
	// Construct ordered argument list for the iom command.
	u8 pwminfo[3];
	pwminfo[0] = chan;
	pwminfo[1] = OnTime;
	pwminfo[2] = OffTime;

	// Select the iom.
	SelectIom( &x, IomPort, 2610 );

	// Schedule the SetRatio action for execution.
	return GWAddAction( x, OP_DIO_SETPWMRATIO, &pwminfo, sizeof(pwminfo) / sizeof(pwminfo[0]), NULL, STATMASK_NONE, NoEndian_0x0 );
}

// Scheduler.
EXPORT(GWERR) S26_Sched2610_GetPwmRatio( HXACT x, IOMPORT IomPort, u8 chan, u8 *OnTime, u8 *OffTime )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2610 );

	// Schedule the GetPwmRatio action for execution.
	GWAddAction( x, OP_DIO_GETPWMRATIO, &chan, sizeof(u8), x->PPRspBuf, STATMASK_NONE, NoEndian_8x2 );

	// Schedule: copy on/off times to application buffers.
	GWAddPost( x, x->PPRspBuf, OnTime, sizeof(u8), memcpy_pp, 0 );
	return GWAddPost( x, x->PPRspBuf + 1, OffTime, sizeof(u8), memcpy_pp, 0 );
}

////////////////////////////////////////////////////////////////////////////////////
// Return/set channel output states for all channels operating in the Normal mode.

// Scheduler.
EXPORT(GWERR) S26_Sched2610_SetOutputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2610 );

	// Schedule the SetOutputs action for execution.
	return GWAddAction( x, OP_DIO_SETOUTPUTS, states, DIO_BYTECOUNT, NULL, STATMASK_NONE, NoEndian_0x0 );
}

// Scheduler.
EXPORT(GWERR) S26_Sched2610_GetOutputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2610 );

	// Schedule the GetOutputs action for execution.
	return GWAddAction( x, OP_DIO_GETOUTPUTS, NULL, ZERO_SIZE, states, STATMASK_NONE, NoEndian_DioStates );
}

//////////////////////////////////////////////////////////////////////////////
// Return the measured states for all channels operating in the Normal mode.

// Scheduler.
EXPORT(GWERR) S26_Sched2610_GetInputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2610 );

	// Schedule the GetInputs action for execution.
	return GWAddAction( x, OP_DIO_GETINPUTS, NULL, 0, states, STATMASK_NONE, NoEndian_DioStates );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////  MODEL 2612 ANALOG INPUT BOARD ACTIONS  //////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


#define NCHANS_2612		4

// Data structure for one channel in user's format. We don't care about byte ordering.
typedef union S2612_CALIBRATION
{
    struct {
        DOUBLE  Offset,
			Scale;
	} d;

    char        Data[ sizeof(DOUBLE) * 2 + 1 ];   // 1 extra byte for checksum

} S2612_CALIBRATION;

//////////////////////////////////////////////
// Set the operating mode for a channel.

// Scheduler (set modes).
EXPORT(GWERR) S26_Sched2612_SetMode( HXACT x, IOMPORT IomPort, u8 chan, u32 mode )
{
	GWERR	err;
	u8		arg[5];
	u32 	nordValue = HtoLE_32( mode );

	// Abort if chan is not legal.
	if ( chan < 0 || chan > NCHANS_2612 )
		return GWERR_BADVALUE | 4;

	// Select the iom.
	err = SelectIom( &x, IomPort, 2612 );

	// Clear sample counter and tstamps.
	if ( err == ERR_NONE )
	{
		x->mm->IomObj[IomPort].Custom.Obj2612.Sample[chan] = 0;
		x->mm->IomObj[IomPort].Custom.Obj2612.Tstamp[chan] = 0;
		x->mm->IomObj[IomPort].Custom.Obj2612.TstampOld[chan] = 0;

		// Construct ordered argument list for the iom command.
		arg[0] = chan;
		arg[1] = ((u8 *)&nordValue)[0];
		arg[2] = ((u8 *)&nordValue)[1];
		arg[3] = ((u8 *)&nordValue)[2];
		arg[4] = ((u8 *)&nordValue)[3];

		// Schedule the SetRatio action for execution.
		err = GWAddAction( x, OP_AI_SETMODE, arg, sizeof(arg) / sizeof(arg[0]), NULL, STATMASK_NONE, NoEndian_0x0 );
	}
	
	return err;
}

////////////////////////////////////////
// Set the voltages for all channels.

// Scheduler.
EXPORT(GWERR) S26_Sched2612_SetVoltages( HXACT x, IOMPORT IomPort, u8 volts )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2612 );

	// Schedule the GetPwmRatio action for execution.
	return GWAddAction( x, OP_AI_SETVOLTAGES, &volts, sizeof(u8), NULL, STATMASK_NONE, NoEndian_0x0 );
}

///////////////////////////////////////////////////////
// Return values and timestamps from all channels.

// Scheduler.
EXPORT(GWERR) S26_Sched2612_GetValues( HXACT x, IOMPORT IomPort, s32 *values, u8 *tstamps )
{
    GWERR err;

	// Select the iom.
	SelectIom( &x, IomPort, 2612 );

    // Fetch values and timestamp into temporary buffer.
    GWAddAction( x, OP_AI_GETINPUTS, NULL, ZERO_SIZE, x->PPRspBuf, STATMASK_NONE, LittleEndian_32x4_8x4 );

    // Copy to app buffers.
    err = GWAddPost( x, x->PPRspBuf, values, sizeof(u32) * 4, memcpy_pp, 0 );

	if ( tstamps != NULL )
        err = GWAddPost( x, x->PPRspBuf + sizeof(u32) * 4, tstamps, sizeof(u8) * 4, memcpy_pp, 0 );

    return err;
}

//////////////////////////////////////////////
// Refresh data buffers for all channels.

// Scheduler.
EXPORT(GWERR) S26_Sched2612_RefreshData( HXACT x, IOMPORT IomPort )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2612 );

    // Fetch values and timestamp into temporary buffer.
    GWAddAction( x, OP_AI_GETINPUTS, NULL, ZERO_SIZE, x->PPRspBuf, STATMASK_NONE, LittleEndian_32x4_8x4 );

    // Copy to IOM buffer.
    GWAddPost( x, x->PPRspBuf, x->mm->IomObj[IomPort].Custom.Obj2612.Value, sizeof(u32) * 4, memcpy_pp, 0 );
	return GWAddPost( x, x->PPRspBuf + sizeof(u32) * 4, x->mm->IomObj[IomPort].Custom.Obj2612.Tstamp, sizeof(u8) * 4, memcpy_pp, 0 );
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Derived calibration functions. These are executed in a private transaction (vs. scheduled).


///////////////////////////////////////////////////////////////////
// Private helper: return values and timestamps from all channels.

static GWERR S26_2612_GetValues( HBD hbd, IOMPORT IomPort, u32 msec, s32 *values, u8 *tstamps, u32 retries )
{
	XACT	*x;

	// Abort if board handle is not legal.
	if ( hbd >= MMCount )
        return GWERR_BADVALUE | 1;

	// Abort if IomPort is not legal.
	if ( IomPort >= NUM_IOMPORTS )
        return GWERR_BADVALUE | 2;

	// Open a new transaction object.
	x = XactAlloc( &obj[hbd], LOGDEV_GATEWAY, retries );

	// Schedule the action for execution.
    S26_Sched2612_GetValues( x, IomPort, values, tstamps );

	// Execute and release the transaction object.  Abort if transaction failed.
	return S26_SchedExecute( x, msec, NULL );
}

////////////////////////////////////////////////////////////////////////////
// Private helper: validate common arguments passed to 2612 api functions.

static GWERR ValidateArgs2612( HBD hbd, IOMPORT IomPort, u8 chan )
{
	// Abort if board handle is not legal.
	if ( hbd >= MMCount )
        return GWERR_BADVALUE | 1;

	// Abort if IomPort is not legal.
	if ( IomPort >= NUM_IOMPORTS )
        return GWERR_BADVALUE | 2;

	if ( obj[hbd].IomObj[IomPort].pIomAttr->IomType != 2612 )
		return GWERR_IOMTYPE;

	// Abort if channel number is not legal.
	if ( chan >= NCHANS_2612 )
		return GWERR_BADVALUE | 4;	// assumed to always be the 4th argument in the api function call
		
	return GWERR_NONE;
}

//////////////////////////////////////////////////
// Return <nsmp> averaged value from a channel.

static GWERR S26_2612_GetAvrValue( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan, u32 nsmp, DOUBLE *value )
{
	u32 	i;
	s32 	values[NCHANS_2612];
	u8		tstamps[NCHANS_2612];
	u8		lasttstamp	= 0;
	DOUBLE	res			= 0;
	GWERR	err			= ERR_NONE;


	// Abort if illegal hbd, iomport, chan or registered module type.
	if ( ( err = ValidateArgs2612( hbd, IomPort, chan ) ) != GWERR_NONE )
		return err;

	// Abort if sample count is not legal.
	if ( nsmp == 0 )
		return GWERR_BADVALUE | 5;

	// For each sample that is to be averaged ...
	for ( i = 0; i < nsmp; i++ )
	{
		// Repeatedly fetch samples until the target channel returns fresh data.
		do
		{
			err = S26_2612_GetValues( hbd, IomPort, msec, values, tstamps, 10 );
			if ( err != GWERR_NONE )
				return err;
		}
		while ( tstamps[chan] == lasttstamp );

		// Abort if over/underflow, otherwise accumulate the fresh data sample.
		switch ( values[chan] & 0x30000000 )
		{
		case 0x30000000:	return GWERR_IOMSPECIFIC | STATUS_2612_OVERFLOW;		// Positive overrange.
		case 0x00000000:	return GWERR_IOMSPECIFIC | STATUS_2612_UNDERFLOW;		// Negative overrange.
		default:			res += ( (DOUBLE)( values[chan] << 3 ) / 0x7fffffff );	// In range: accumulate.
		}

		// Remember the sample's timestamp.
		lasttstamp = tstamps[chan];
	}

	// Compute averaged value.
	*value = res / i;

    return err;
}

///////////////////////////////////////////////////
// Register Zero Load condition for a channel.

EXPORT(GWERR) S26_2612_RegisterZero( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan, u32 nsmp )
{
    GWERR	err;
    DOUBLE	value;

	// Get value.
	if ( ( err = S26_2612_GetAvrValue( hbd, IomPort, msec, chan, nsmp, &value ) ) == GWERR_NONE )
		obj[hbd].IomObj[IomPort].Custom.Obj2612.Offset[chan] = value;

    return err;
}

/////////////////////////////////////////////////
// Register Full Load condition for a channel.

EXPORT(GWERR) S26_2612_RegisterSpan( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan, u32 nsmp, DOUBLE load )
{
	GWERR	err;
	DOUBLE	value;
	DOUBLE	span;

	// Get channel's analog input value, averaged over specified number of samples. If successful ...
	if ( ( err = S26_2612_GetAvrValue( hbd, IomPort, msec, chan, nsmp, &value ) ) == GWERR_NONE )
	{
		// Cache pointer to iom object.
		IOM_2612 *aio = &obj[hbd].IomObj[IomPort].Custom.Obj2612;

		// Compute span, which is the difference between full-scale and offset values.
		span = value - aio->Offset[chan];

		// Force span to a useable value if it's magnitude is too small.
		if ( ( span > -1e-7 ) && ( span < 1e-7 ) )
			span = ( span < 0.0 ) ? -1e-7 : 1e-7;

		// Compute scale factor for the span, based on the specified application load.
		aio->Scale[chan] = load / span;
	}

	// Return error, if any.
	return err;
}

//////////////////////////////////////////
// Register Tare for a channel.

EXPORT(GWERR) S26_2612_RegisterTare( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan, u32 nsmp )
{
    GWERR     err;
    DOUBLE    value;

	// Get tare value. If successful ...
	if ( ( err = S26_2612_GetAvrValue( hbd, IomPort, msec, chan, nsmp, &value ) ) == GWERR_NONE )
	{
		// Cache pointer to iom object.
		IOM_2612 *aio = &obj[hbd].IomObj[IomPort].Custom.Obj2612;

		// Compute and store normalized tare value.
		aio->Tare[chan] = ( value - aio->Offset[chan] ) * aio->Scale[chan];
	}

	// Return error, if any.
    return err;
}

/////////////////////////////////////////
// Write eeprom byte value.

static GWERR S26_2612_WriteEeprom( HBD hbd, IOMPORT IomPort, u32 msec, u8 address, u8 value, u32 retries )
{
	XACT	*x;
	u8		arg[2];
	GWERR	err;

	// Abort if illegal hbd, iomport or registered module type.
	if ( ( err = ValidateArgs2612( hbd, IomPort, 0 ) ) != GWERR_NONE )
		return err;

	// Populate the action argument list.
	arg[0] = address;
	arg[1] = value;

	// Open a new transaction object.
	x = XactAlloc( &obj[hbd], LOGDEV_GATEWAY, retries );

	// Select the iom.
	SelectIom( &x, IomPort, 2612 );

	// Schedule the WriteEeprom action for execution.
	GWAddAction( x, OP_AI_WRITEEPROM, &arg, sizeof(arg) / sizeof(arg[0]), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Execute and release the transaction object.  Abort if transaction failed.
	return S26_SchedExecute( x, msec, NULL );
}


//////////////////////////////////////////////
// Private helper: read eeprom byte value.

static GWERR S26_2612_ReadEeprom( HBD hbd, IOMPORT IomPort, u32 msec, u8 address, u8 *value, u32 retries )
{
	XACT	*x;
	GWERR	err;

	// Abort if illegal hbd, iomport or registered module type.
	if ( ( err = ValidateArgs2612( hbd, IomPort, 0 ) ) != GWERR_NONE )
		return err;

	// Open a new transaction object.
	x = XactAlloc( &obj[hbd], LOGDEV_GATEWAY, retries );

	// Select the iom.
	SelectIom( &x, IomPort, 2612 );

	// Schedule the WriteEeprom action for execution.
	GWAddAction( x, OP_AI_READEEPROM, &address, sizeof(u8), value, STATMASK_NONE, NoEndian_8x1 );

	// Execute and release the transaction object.  Abort if transaction failed.
	return S26_SchedExecute( x, msec, NULL );
}

////////////////////////////////////////////
// Save calibration values to EEPROM.

EXPORT(GWERR) S26_2612_SaveCalibrations( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan )
{
	GWERR				err;
	int					i;
	S2612_CALIBRATION	clb;
	u8					sum;
	IOM_2612			*aio;
	u8					address;
	u8					value;
	
	// Abort if illegal hbd, iomport, chan or registered module type.
	if ( ( err = ValidateArgs2612( hbd, IomPort, chan ) ) != GWERR_NONE )
		return err;

	// Cache pointer to iom object.
	aio = &obj[hbd].IomObj[IomPort].Custom.Obj2612;

	// Copy to buffer.
	clb.d.Offset = aio->Offset[chan];
	clb.d.Scale  = aio->Scale[chan];

	// Data to EEPROM.
	for( i = 0, sum = 0, address = sizeof(clb.Data) * chan; i < sizeof(clb.d); i++, address++ )
	{
		value = clb.Data[i];

		if ( ( err = S26_2612_WriteEeprom( hbd, IomPort, msec, address, value, 10 ) ) != GWERR_NONE )
			return err | STATUS_2612_EEPROM;

		if ( ( err = S26_2612_ReadEeprom( hbd, IomPort, msec, address, &value, 10 ) ) != GWERR_NONE )
			return err | STATUS_2612_EEPROM;

		// Calculate checksum.
		sum += value;
	}

	// Checksum to EEPROM.
	if ( ( err = S26_2612_WriteEeprom( hbd, IomPort, msec, address, sum, 10 ) ) != GWERR_NONE )
		return err | STATUS_2612_EEPROM;

	return GWERR_NONE;
}

/////////////////////////////////////////////////////////////////////
// Restore calibration values from EEPROM.

EXPORT(GWERR) S26_2612_RestoreCalibrations( HBD hbd, IOMPORT IomPort, u32 msec, u8 chan )
{
	GWERR				err;
	int 				i;
	S2612_CALIBRATION	clb;
	u8					sum;
	IOM_2612			*aio;
	u8					address;
	u8					value;
	
	// Abort if illegal hbd, iomport, chan or registered module type.
	if ( ( err = ValidateArgs2612( hbd, IomPort, chan ) ) != GWERR_NONE )
		return err;

	// Cache pointer to iom object.
	aio = &obj[hbd].IomObj[IomPort].Custom.Obj2612;

	// Data from EEPROM.
	for( i = 0, sum = 0, address = sizeof(clb.Data) * chan; i < sizeof(clb.d); i++, address++ )
	{
		if ( ( err = S26_2612_ReadEeprom( hbd, IomPort, msec, address, &value, 10 ) ) != GWERR_NONE )
			return err | STATUS_2612_EEPROM;

		clb.Data[i] = value;

		// Calculate checksum.
		sum += value;
	}

	// Checksum from EEPROM.
	if ( ( err = S26_2612_ReadEeprom( hbd, IomPort, msec, address, &value, 10 ) ) != GWERR_NONE )
		return err | STATUS_2612_EEPROM;

	if ( value != sum )
		return err | STATUS_2612_EEPROM;

	// Copy to iom object.
	aio->Offset[chan] = clb.d.Offset;
	aio->Scale[chan]  = clb.d.Scale;

	return GWERR_NONE;
}


//////////////////////////////////////////////////////////////////////////////////////////
// Derived calibration functions. These are executed immediately (vs. scheduled).


/////////////////////////////////////////////////////////////////////
// Return calibrated value from a channel.

EXPORT(DOUBLE) S26_2612_GetCalibratedValue( HBD hbd, IOMPORT IomPort, u8 chan, u32 *sample )
{
	IOM_2612	*aio;
	s32 		v;
	u8			dts;
	
	// If board handle, iom port and channel are legal ...
	if ( hbd < MMCount && IomPort < NUM_IOMPORTS && chan < 4 )
	{
		// If registered iom type is 2612 ...
		if ( obj[hbd].IomObj[IomPort].pIomAttr->IomType == 2612 )
		{
			// Cache pointer to iom object.
			aio = &obj[hbd].IomObj[IomPort].Custom.Obj2612;
			
			// Calculate sample number.
			dts = aio->Tstamp[chan] - aio-> TstampOld[chan];
			aio->TstampOld[chan] = aio-> Tstamp[chan];
			aio->Sample[chan] += dts;
			
			if ( sample != NULL )
				*sample = aio->Sample[chan];
			
			v = aio->Value[chan];
			
			if ( ( v & 0x30000000 ) == 0x30000000 )
				return 1e100;							// Out of range (positive).

			if ( ( v & 0x30000000 ) == 0x00000000 )
				return -1e100;							// Out of range (negative).
			
			// Convert raw adc data to a corrected voltage and return it.
			return ( ( DOUBLE )( v << 3 ) / 0x7fffffff - aio->Offset[chan] )	// Subtract the offset.
				* aio->Scale[chan]												// Normalize against user's standard.
				- aio->Tare[chan];												// Subtract the tare.
			
		}
	}
	
	// Return default value in case of error.
	return 0.0;
}

/////////////////////////////////////////////////////////////////////
// Return Offset value from a channel.

EXPORT(DOUBLE) S26_2612_GetOffset( HBD hbd, IOMPORT IomPort, u8 chan )
{
	IOM_2612 *aio;

	// If board handle, iom port and channel are legal ...
	if ( hbd < MMCount && IomPort < NUM_IOMPORTS && chan < 4 )
	{
		// If registered iom type is 2612 ...
		if ( obj[hbd].IomObj[IomPort].pIomAttr->IomType == 2612 )
		{
			// Cache pointer to iom object.
			aio = &obj[hbd].IomObj[IomPort].Custom.Obj2612;

			return aio->Offset[chan];
		}
	}

	// Return default value in case of error.
	return 0.0;
}

/////////////////////////////////////////////////////////////////////
// Return Scale value from a channel.

EXPORT(DOUBLE) S26_2612_GetScale( HBD hbd, IOMPORT IomPort, u8 chan )
{
	IOM_2612 *aio;

	// If board handle, iom port and channel are legal ...
	if ( hbd < MMCount && IomPort < NUM_IOMPORTS && chan < 4 )
	{
		// If registered iom type is 2612 ...
		if ( obj[hbd].IomObj[IomPort].pIomAttr->IomType == 2612 )
		{
			// Cache pointer to iom object.
			aio = &obj[hbd].IomObj[IomPort].Custom.Obj2612;

			return aio->Scale[chan];
		}
	}

	// Return default value in case of error.
	return 0.0;
}

/////////////////////////////////////////////////////////////////////
// Return Tare value from a channel.

EXPORT(DOUBLE) S26_2612_GetTare( HBD hbd, IOMPORT IomPort, u8 chan )
{
	IOM_2612 *aio;

	// If board handle, iom port and channel are legal ...
	if ( hbd < MMCount && IomPort < NUM_IOMPORTS && chan < 4 )
	{
		// If registered iom type is 2612 ...
		if ( obj[hbd].IomObj[IomPort].pIomAttr->IomType == 2612 )
		{
			// Cache pointer to iom object.
			aio = &obj[hbd].IomObj[IomPort].Custom.Obj2612;

			return aio->Tare[chan];
		}
	}

	// Return default value in case of error.
	return 0.0;
}

/////////////////////////////////////////////////////////////////////
// Set calibration values to a channel.

EXPORT(GWERR) S26_2612_SetCalibrations( HBD hbd, IOMPORT IomPort, u8 chan, DOUBLE Offset, DOUBLE Scale, DOUBLE Tare )
{
	IOM_2612 *aio;

	// If board handle, iom port and channel are legal ...
	if ( hbd < MMCount && IomPort < NUM_IOMPORTS && chan < 4 )
	{
		// If registered iom type is 2612 ...
		if ( obj[hbd].IomObj[IomPort].pIomAttr->IomType == 2612 )
		{
			// Cache pointer to iom object.
			aio = &obj[hbd].IomObj[IomPort].Custom.Obj2612;

            aio->Offset[chan] = Offset;
			aio->Scale[chan]  = Scale;
			aio->Tare[chan]   = Tare;

            return GWERR_NONE;
		}
	}

	// Return error code.
	return GWERR_BADVALUE;
}

//////////////////////////////////////////////
// VFUNC: Initialize board image upon reset.

static void s2612_IomReset( IOM *iom )
{
	int			i;
	IOM_2612	*iom2612 = &iom->Custom.Obj2612;

	// Initialize object.
	for ( i = 0; i < 4; i++ )
	{
  		iom2612->Sample[i]		= 0;				// Sample counter.
  		iom2612->Tstamp[i]		= 0;				// Timestamps.
  		iom2612->TstampOld[i]	= 0;
   		iom2612->Value[i]		= 0;				// Default value.
		iom2612->Offset[i]		= 0;				// Default offset.
		iom2612->Scale[i]		= SG_DEFAULT_SCALE;	// Default scale.
		iom2612->Tare[i]		= 0;				// Default Tare.
	}
}

//////////////////////////////////////////////////////////////
// VFUNC: Initialize board and its image upon registration.

static void s2612_IomRegister( IOM *iom )
{
	// Initialize iom's configuration image.
	s2612_IomReset( iom );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////  MODEL 2620 COUNTER BOARD ACTIONS  ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
// VFUNC: Initialize board image upon registration or reset.

static void s2620_IomReset( IOM *iom )
{
	int			i;
	IOM_2620	*iom2620 = &iom->Custom.Obj2620;

	// CommonControl register image defaults to zero: gate time divisor = 2 msec, time stamp resolution = 1 us.
	iom2620->CommonControl = 0x0000;

	// Set all channel Mode and Preload register images to zero.
	for ( i = 0; i < 4; i++ )
	{
		iom2620->ChanAttr->Mode			= 0;
		iom2620->ChanAttr->Preload[0]	= 0;
		iom2620->ChanAttr->Preload[1]	= 0;
	}
}

static void s2620_IomRegister( IOM *iom )
{
	s2620_IomReset( iom );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Program the control register for one counter channel.  This triggers a Latch or Preload operation.

// Scheduler.
EXPORT(GWERR) S26_Sched2620_SetControlReg( HXACT x, IOMPORT IomPort, u8 chan, u8 DataVal )
{
	// Select the iom.
	if ( SelectIom( &x, IomPort, 2620 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal command values.
	if ( chan >= 4 )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Program control register.
	return GWAddAction( x, (u8)( ( chan << 4 ) | OP_CNT_SOFTTRIGGER ), &DataVal, sizeof(DataVal), NULL, STATMASK_NONE, NoEndian_0x0 );
}

////////////////////////////////////////////////
// Return status info for one counter channel.

// Scheduler.
EXPORT(GWERR) S26_Sched2620_GetStatus( HXACT x, IOMPORT IomPort, u8 chan, u16 *status )
{
	// Select the iom.
	if ( SelectIom( &x, IomPort, 2620 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal command values.
	if ( chan >= 4 )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Return status.
	return GWAddAction( x, (u8)( ( chan << 4 ) | OP_CNT_GETSTATUS ), NULL, ZERO_SIZE, status, STATMASK_NONE, LittleEndian_16x1 );
}

/////////////////////////////////////////////////////////////////////
// Program one of the specified counter channel's preload registers.
// Imports:
//	chan   = counter channel number in range 0:3.
//	reg    = preload register: CNT_REG_PRELOAD0 or CNT_REG_PRELOAD1.
//	value  = 32-bit value to be preloaded.

// Scheduler.
EXPORT(GWERR) S26_Sched2620_SetPreload( HXACT x, IOMPORT IomPort, u8 chan, u8 reg, u32 value )
{
	// Correct byte ordering for the preload value.
	u32 nordValue = HtoLE_32( value );

	// Select the iom.
	if ( SelectIom( &x, IomPort, 2620 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal channel number.
	if ( chan >= 4 )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Check for illegal preload register identifier.
	if ( reg >= 2 )
		x->GWErrCode = GWERR_BADVALUE | 4;

	// Preload the counter.
	GWAddAction( x, (u8)( ( chan << 4 ) | ( ( reg == 0 ) ? OP_CNT_SETPRELOAD0 : OP_CNT_SETPRELOAD1 ) ), &nordValue, sizeof(nordValue), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Schedule post-processing step: copy preload value to middleware image.
	return GWAddPost( x, PushCmdArg( x, &value, sizeof(value) ), &x->mm->IomObj[IomPort].Custom.Obj2620.ChanAttr[chan].Preload[reg], sizeof(value), memcpy_pp, 0 );
}

//////////////////////////////////////////////////////////////////////
// Return middleware's image of preload counter.
// Imports:
//	chan   = counter channel number in range 0:3.
//	reg    = preload register: CNT_REG_PRELOAD0 or CNT_REG_PRELOAD1.
//	value  = pointer to application buffer that is to receive the preload value.

// Scheduler.
EXPORT(GWERR) S26_Sched2620_GetPreload( HXACT x, IOMPORT IomPort, u8 chan, u8 reg, u32 *value )
{
	// Select the iom.
	if ( SelectIom( &x, IomPort, 2620 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal channel number.
	if ( chan >= 4 )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Check for illegal preload register identifier.
	if ( reg >= 2 )
		x->GWErrCode = GWERR_BADVALUE | 4;

	// Schedule post-processing step: copy middleware's preload value image to application buffer.
	return GWAddPost( x, &x->mm->IomObj[IomPort].Custom.Obj2620.ChanAttr[chan].Preload[reg], value, sizeof(u32), memcpy_pp, 0 );
}

/////////////////////////////////////////////////////////////////////
// Return counts from the specified counter channel.

// Scheduler.
EXPORT(GWERR) S26_Sched2620_GetCounts( HXACT x, IOMPORT IomPort, u8 chan, u32 *value, u16 *tstamp )
{
	// Select the iom.
	if ( SelectIom( &x, IomPort, 2620 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal command values.
	if ( chan >= 4 )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// If both counts and timestamp are to be fetched ...
	if ( tstamp != 0 )
	{
		// Fetch counts and timestamp into temporary buffer.
		GWAddAction( x, (u8)( ( chan << 4 ) | OP_CNT_GETCOUNTS_TS ), NULL, ZERO_SIZE, x->PPRspBuf, STATMASK_NONE, LittleEndian_32x1_16x1 );

		// Copy to app buffers.
		GWAddPost( x, x->PPRspBuf, value, 4, memcpy_pp, 0 );
		return GWAddPost( x, x->PPRspBuf + 4, tstamp, 2, memcpy_pp, 0 );
	}
	else	// If only the count is to be fetched ...
	{
		// Fetch counts.
		return GWAddAction( x, (u8)( ( chan << 4 ) | OP_CNT_GETCOUNTS ), NULL, ZERO_SIZE, value, STATMASK_NONE, LittleEndian_32x1 );
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// Program the common control register (gate generator period and timestamp resolution).

// Scheduler.
EXPORT(GWERR) S26_Sched2620_SetCommonControl( HXACT x, IOMPORT IomPort, u16 period, u8 tstamp )
{
	u16 arg;
	u16 nordArg;

	// Select the iom.
	if ( SelectIom( &x, IomPort, 2620 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal command values.
	if ( ( period > 16383 ) || ( period < 2 ) )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Check for illegal command values.
	if ( tstamp > 3 )
		x->GWErrCode = GWERR_BADVALUE | 4;

	// Compute combined value for the specified gate time and timestamp resolution.
	arg = (u16)( ( 4 * ( period / 2 - 1 ) ) | ( tstamp & 3 ) );

	// Convert register image to iom byte order.
	nordArg = HtoLE_16( arg );

	// Schedule the action for execution.
	GWAddAction( x, OP_CNT_SETCOMMONCONTROL, &nordArg, sizeof(nordArg), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Schedule postprocessing, which will copy new CommonControl register value to middleware's image.
	return GWAddPost( x, PushCmdArg( x, &arg, sizeof(arg) ), &x->mm->IomObj[IomPort].Custom.Obj2620.CommonControl, sizeof(u16), memcpy_pp, 0 );
}

////////////////////////////////////////////////////////////////////////////////
// Return the common control register setting (from middleware's image).

// Postprocessor: break CommonControl image into constituent Period and Tstamp parts and store them into PPRspBuf.
static void GetCommonControl_pp( POSTPROC *p )
{
	// Cache the middleware's CommonControl image for the target 2620.
	u16 ccval = *( (u16 *)p->pSrcData );

	// Extract Period value into PPRspBuf[0-1].
	*( (u16 *)p->x->PPRspBuf ) = ccval >> 2;

	// Extract Tstamp value into PPRspBuf[2].
	p->x->PPRspBuf[2] = (u8)( ccval & 3 );
}

// Scheduler.
EXPORT(GWERR) S26_Sched2620_GetCommonControl( HXACT x, IOMPORT IomPort, u16 *period, u8 *tstamp )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2620 );

	// Execute postprocessor, which splits CommonControl image into period and tstamp parts and stores them in PPRspBuf.
	GWAddPost( x, &x->mm->IomObj[IomPort].Custom.Obj2620.CommonControl, x->PPRspBuf, IMPLICIT_SIZE, GetCommonControl_pp, 0 );

	// Copy period to application buffer.
	GWAddPost( x, x->PPRspBuf, period, sizeof(u16), memcpy_pp, 0 );

	// Copy tstamp to application buffer.
	return GWAddPost( x, &x->PPRspBuf[2], tstamp, sizeof(u8), memcpy_pp, 0 );
}

//////////////////////////////////////////////////////////
// Set the operating mode for one counter channel.

// Scheduler: this is the "generic" function that sets the counter mode to any arbitrary value.
EXPORT(GWERR) S26_Sched2620_SetMode( HXACT x, IOMPORT IomPort, u8 chan, u16 mode )
{
	// Correct byte ordering.
	u16 nordMode = HtoLE_16( mode );

	// Select the iom.
	if ( SelectIom( &x, IomPort, 2620 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal command values.
	if ( chan >= 4 )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Schedule the action.
	GWAddAction( x, (u8)( ( chan << 4 ) | OP_CNT_SETMODE ), &nordMode, sizeof(nordMode), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Schedule post-processing, which will copy new mode value to middleware's image.
	mode &= ~CT_RUN_ENABLE;		// Mask off Run bit.
	return GWAddPost( x, PushCmdArg( x, &mode, sizeof(mode) ), &x->mm->IomObj[IomPort].Custom.Obj2620.ChanAttr[chan].Mode, sizeof(mode), memcpy_pp, 0 );
}

////////////////////////////////////////////////////////////////////////////////
// Return the operating mode for one counter channel (from middleware's image).

// Scheduler.
EXPORT(GWERR) S26_Sched2620_GetMode( HXACT x, IOMPORT IomPort, u8 chan, u16 *mode )
{
	// Select the iom.
	if ( SelectIom( &x, IomPort, 2620 ) != GWERR_NONE )
		return x->GWErrCode;

	// Check for illegal command values.
	if ( chan >= 4 )
		x->GWErrCode = GWERR_BADVALUE | 3;

	// Schedule post-processing step: copy mode image to application buffer.
	return GWAddPost( x, &x->mm->IomObj[IomPort].Custom.Obj2620.ChanAttr[chan].Mode, mode, sizeof(u16), memcpy_pp, 0 );
}

/////////////////////////////////////////////////////////////////////
// Derived mode-setting functions.

// Scheduler helper for derived mode-setting functions.
static GWERR SetMode2620( HXACT x, IOMPORT IomPort, u8 chan, u16 mode )
{
	// Halt channel and configure its operating mode.
	S26_Sched2620_SetMode( x, IomPort, chan, (u16)( mode & ~CT_RUN_ENABLE ) );

	// Start channel running in the new mode.
	S26_Sched2620_SetMode( x, IomPort, chan, (u16)( mode | CT_RUN_ENABLE ) );

	// Transfer preload register to core.
	return S26_Sched2620_SetControlReg( x, IomPort, chan, CNT_TRIG_LOAD );
}

// Scheduler: set counter mode for encoder interface.
EXPORT(GWERR) S26_Sched2620_SetModeEncoder( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowX, u16 HardPreload, u16 ClkMode )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_DONTCARE						// Output pin is not used, so mode is not relevant.
		| ( ActLowX ? CT_XP_ACTLOW : CT_XP_ACTHIGH )		// Optional: select index polarity if used as preload trigger.
		| ( HardPreload ? CT_PL_INDEX : CT_PL_SOFTONLY )	// Optional: copy Preload0 to core upon index leading edge.
		| CT_LAT_SOFTONLY									// Core copied to latch only upon software command.
		| CT_CET_CONFIG										// Enable counting upon configuration.
		| CT_OP_DONTCARE									// Output pin is not used, so polarity is not relevant.
		| ( ClkMode << CT_M_SHFT )							// Clock mode may have any value from 0 to 6.
		| CT_CD_NEVER										// Never disable counting after started.
		| CT_PLM_SINGLE										// Use Preload0 only.
		| CT_XC_EXTERNAL );									// If used, index signal is applied to external index pin.

	// Program counter channel's operating mode.
	return SetMode2620( x, IomPort, chan, mode );
}

// Scheduler: set counter mode for pulse generator.
// Pulse is triggered by software, or optionally, by hardware trigger on index input.
// ClkB input must be configured to count down.
EXPORT(GWERR) S26_Sched2620_SetModePulseGen( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowX, u16 HardTrig, u16 ActLowOut )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_ZEROACTIVE						// Output pin goes active (pulse goes inactive) when core reaches zero counts.
		| ( ActLowX ? CT_XP_ACTLOW : CT_XP_ACTHIGH )		// Optional: select index polarity if used as harware pulse trigger.
		| ( HardTrig ? CT_PL_INDEX : CT_PL_SOFTONLY )		// Optional: copy Preload0 to core upon index leading edge (triggers pulse output).
		| CT_LAT_SOFTONLY									// Core copied to latch only upon software command.
		| CT_CET_CONFIG										// Enable counting upon configuration.
		| ( ActLowOut ? CT_OP_ACTLOW : CT_OP_ACTHIGH )		// Select polarity of output pulse.
		| CT_M_INTERNAL										// Use internal 10MHz clock as the clock source.
		| CT_CD_ZERO										// Disable counting when core reaches zero counts.
		| CT_PLM_SINGLE										// Use Preload0 only.
		| CT_XC_EXTERNAL );									// If used, index signal is applied to external index pin.

	// Program counter channel's operating mode.
	return SetMode2620( x, IomPort, chan, mode );
}

// Scheduler: set counter mode for pwm generator.
// Upon entry, preload 1/0 registers should have been previously set to desired on/off times.
EXPORT(GWERR) S26_Sched2620_SetModePwmGen( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowOut )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_ZEROTOGGLE						// Output pin toggles state when core reaches zero counts.
		| CT_XP_DONTCARE									// Index input is not used, so its polarity is don't care.
		| CT_PL_ZERO										// Preload the alternate preload register when core reaches zero counts.
		| CT_LAT_SOFTONLY									// Core copied to latch only by soft command.
		| CT_CET_CONFIG										// Enable counting upon configuration.
		| ( ActLowOut ? CT_OP_ACTLOW : CT_OP_ACTHIGH )		// Select polarity of output pulse.
		| CT_M_INTERNAL										// Use internal 10MHz clock as the clock source.
		| CT_CD_NEVER										// Never disable counting after started.
		| CT_PLM_BOTH										// Use both preload registers.
		| CT_XC_EXTERNAL );									// Index input is not used, but we mark it "external" anyway.

	// Program counter channel's operating mode.
	return SetMode2620( x, IomPort, chan, mode );
}

// Scheduler: set counter mode for pulse width measurement.
EXPORT(GWERR) S26_Sched2620_SetModePulseMeas( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowX )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_DONTCARE						// Output pin is not used, so mode is not relevant.
		| ( ActLowX ? CT_XP_ACTHIGH : CT_XP_ACTLOW )		// Select index polarity.  This selects the EDGE, which is opposite of LEVEL.
		| CT_PL_INDEX
		| CT_LAT_INDEX
		| CT_CET_INDEX										// Enable counting on first gate leading edge.
		| CT_OP_DONTCARE									// Output pin is not used, so polarity is not relevant.
		| CT_M_INTERNAL										// Use internal 10MHz clock as the clock source.
		| CT_CD_INDEX
		| CT_PLM_SINGLE										// Use Preload0 only.
		| CT_XC_EXTERNAL );

	// Program counter channel's operating mode.
	return SetMode2620( x, IomPort, chan, mode );
}

// Scheduler: set counter mode for period measurement.  Input signal is applied to index input.
EXPORT(GWERR) S26_Sched2620_SetModePeriodMeas( HXACT x, IOMPORT IomPort, u8 chan, u16 ActLowX )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_DONTCARE						// Output pin is not used, so mode is not relevant.
		| ( ActLowX ? CT_XP_ACTLOW : CT_XP_ACTHIGH )		// Index polarity is selectable in case one signal edge has more jitter.
		| CT_PL_INDEX										// Preload zero on index leading edge.
		| CT_LAT_INDEX										// Latch counts on index leading edge.
		| CT_CET_INDEX										// Enable counting on first input signal leading edge.
		| CT_OP_DONTCARE									// Output pin is not used, so polarity is not relevant.
		| CT_M_INTERNAL										// Use internal 10MHz clock as the clock source.
		| CT_CD_NEVER										// Never disable counting after started.
		| CT_PLM_SINGLE										// Use Preload0 only.
		| CT_XC_EXTERNAL );									// Input signal to be measured is applied to external index pin.

	// Set Preload0 value to zero; this will be automatically copied to the core each time a new measurement begins.
	S26_Sched2620_SetPreload( x, IomPort, chan, CNT_REG_PRELOAD0, 0 );

	// Program counter channel's operating mode.
	return SetMode2620( x, IomPort, chan, mode );
}

// Scheduler: set counter mode for frequency measurement.
// The index, which is used as a CountEnable gate, can be either external or internal depending on IntGate.
EXPORT(GWERR) S26_Sched2620_SetModeFreqMeas( HXACT x, IOMPORT IomPort, u8 chan, u16 igate )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_DONTCARE						// Output pin is not used, so mode is not relevant.
		| CT_XP_ACTHIGH										// Sample intervals begin on gate (index) rising edge.
		| CT_PL_INDEX										// Preload zero on gate (index input) leading edge.
		| CT_LAT_INDEX										// Latch on gate (index input) leading edge.
		| CT_CET_INDEX										// Enable counting on first gate leading edge.
		| CT_OP_DONTCARE									// Output pin is not used, so polarity is not relevant.
		| CT_M_MONOARISE									// Count rising edges on ClkA input during gate interval.
		| CT_CD_NEVER										// Never disable counting after started.
		| CT_PLM_SINGLE										// Use Preload0 only, which must be preset to zero.
		| ( igate ? CT_XC_INTERNAL : CT_XC_EXTERNAL ) );	// Optional: use internal gate instead of index.

	// Set Preload0 value to zero; this will be automatically copied to the core each time a new measurement begins.
	S26_Sched2620_SetPreload( x, IomPort, chan, CNT_REG_PRELOAD0, 0 );

	// Program counter channel's operating mode.
	return SetMode2620( x, IomPort, chan, mode );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////  MODEL 2631 MOTOR CONTROL BOARD ACTIONS  ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
// VFUNC: Initialize board image upon registration or reset.

static void s2631_IomReset( IOM *iom )
{
	IOM_2631	*iom2631 = &iom->Custom.Obj2631;

	// CommonControl register image defaults to zero: gate time divisor = 2 msec, time stamp resolution = 1 us.
	iom2631->CommonControl = 0x0000;

	// Set input channel Mode and Preload register images to zero.
    iom2631->ChanAttrIn.Mode		= 0;
    iom2631->ChanAttrIn.Preload[0]	= 0;
    iom2631->ChanAttrIn.Preload[1]	= 0;
}

static void s2631_IomRegister( IOM *iom )
{
	s2631_IomReset( iom );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// Input Counter //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////
// Program the control register for input counter channel.  This triggers a Latch or Preload operation.

// Scheduler.
EXPORT(GWERR) S26_Sched2631_SetControlReg( HXACT x, IOMPORT IomPort, u8 DataVal )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Program control register.
	return GWAddAction( x, (u8)( OP_MC_SOFTTRIGGER ), &DataVal, sizeof(DataVal), NULL, STATMASK_NONE, NoEndian_0x0 );
}

////////////////////////////////////////////////
// Return status info for input counter channel.

// Scheduler.
EXPORT(GWERR) S26_Sched2631_GetStatus( HXACT x, IOMPORT IomPort, u16 *status )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Return status.
	return GWAddAction( x, (u8)( OP_MC_GETSTATUS ), NULL, ZERO_SIZE, status, STATMASK_NONE, LittleEndian_16x1 );
}

/////////////////////////////////////////////////////////////////////
// Program one of the specified counter channel's preload registers.
// Imports:
//	reg    = preload register: CNT_REG_PRELOAD0 or CNT_REG_PRELOAD1.
//	value  = 32-bit value to be preloaded.

// Scheduler.
EXPORT(GWERR) S26_Sched2631_SetPreload( HXACT x, IOMPORT IomPort, u8 reg, u32 value )
{
	// Correct byte ordering for the preload value.
	u32 nordValue = HtoLE_32( value );

	// Check for illegal preload register identifier.
	if ( reg >= 2 )
		return GWERR_BADVALUE | 4;

	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Preload the counter.
	GWAddAction( x, (u8)( ( ( reg == 0 ) ? OP_MC_SETPRELOAD0 : OP_MC_SETPRELOAD1 ) ), &nordValue, sizeof(nordValue), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Schedule post-processing step: copy preload value to middleware image.
	return GWAddPost( x, PushCmdArg( x, &value, sizeof(value) ), &x->mm->IomObj[IomPort].Custom.Obj2631.ChanAttrIn.Preload[reg], sizeof(value), memcpy_pp, 0 );
}

//////////////////////////////////////////////////////////////////////
// Return middleware's image of preload counter.
// Imports:
//	reg    = preload register: CNT_REG_PRELOAD0 or CNT_REG_PRELOAD1.
//	value  = pointer to application buffer that is to receive the preload value.

// Scheduler.
EXPORT(GWERR) S26_Sched2631_GetPreload( HXACT x, IOMPORT IomPort, u8 reg, u32 *value )
{
	// Check for illegal preload register identifier.
	if ( reg >= 2 )
		return GWERR_BADVALUE | 4;

	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Schedule post-processing step: copy middleware's preload value image to application buffer.
	return GWAddPost( x, &x->mm->IomObj[IomPort].Custom.Obj2631.ChanAttrIn.Preload[reg], value, sizeof(u32), memcpy_pp, 0 );
}

/////////////////////////////////////////////////////////////////////
// Return counts from the input counter channel.

// Scheduler.
EXPORT(GWERR) S26_Sched2631_GetCounts( HXACT x, IOMPORT IomPort, u32 *value, u16 *tstamp )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// If both counts and timestamp are to be fetched ...
	if ( tstamp != 0 )
	{
		// Fetch counts and timestamp into temporary buffer.
		GWAddAction( x, (u8)( OP_MC_GETCOUNTS_TS ), NULL, ZERO_SIZE, x->PPRspBuf, STATMASK_NONE, LittleEndian_32x1_16x1 );

		// Copy to app buffers.
		GWAddPost( x, x->PPRspBuf, value, 4, memcpy_pp, 0 );
		return GWAddPost( x, x->PPRspBuf + 4, tstamp, 2, memcpy_pp, 0 );
	}
	else	// If only the count is to be fetched ...
	{
		// Fetch counts.
		return GWAddAction( x, (u8)( OP_MC_GETCOUNTS ), NULL, ZERO_SIZE, value, STATMASK_NONE, LittleEndian_32x1 );
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// Program the common control register (gate generator period and timestamp resolution).

// Scheduler.
EXPORT(GWERR) S26_Sched2631_SetCommonControl( HXACT x, IOMPORT IomPort, u16 period, u8 tstamp )
{
	u16 arg;
	u16 nordArg;

	// Check for illegal command values.
	if ( ( period > 16383 ) || ( period < 2 ) )
		return GWERR_BADVALUE | 3;

	// Check for illegal command values.
	if ( tstamp > 3 )
		return GWERR_BADVALUE | 4;

	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Compute combined value for the specified gate time and timestamp resolution.
	arg = (u16)( ( 4 * ( period / 2 - 1 ) ) | ( tstamp & 3 ) );

	// Convert register image to iom byte order.
	nordArg = HtoLE_16( arg );

	// Schedule the action for execution.
	GWAddAction( x, OP_MC_SETCOMMONCONTROL, &nordArg, sizeof(nordArg), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Schedule postprocessing, which will copy new CommonControl register value to middleware's image.
	return GWAddPost( x, PushCmdArg( x, &arg, sizeof(arg) ), &x->mm->IomObj[IomPort].Custom.Obj2631.CommonControl, sizeof(u16), memcpy_pp, 0 );
}

// Scheduler.
EXPORT(GWERR) S26_Sched2631_GetCommonControl( HXACT x, IOMPORT IomPort, u16 *period, u8 *tstamp )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Execute postprocessor, which splits CommonControl image into period and tstamp parts and stores them in PPRspBuf.
	GWAddPost( x, &x->mm->IomObj[IomPort].Custom.Obj2631.CommonControl, x->PPRspBuf, IMPLICIT_SIZE, GetCommonControl_pp, 0 );

	// Copy period to application buffer.
	GWAddPost( x, x->PPRspBuf, period, sizeof(u16), memcpy_pp, 0 );

	// Copy tstamp to application buffer.
	return GWAddPost( x, &x->PPRspBuf[2], tstamp, sizeof(u8), memcpy_pp, 0 );
}

//////////////////////////////////////////////////////////
// Set the operating mode for input counter channel.

// Scheduler: this is the "generic" function that sets the counter mode to any arbitrary value.
EXPORT(GWERR) S26_Sched2631_SetMode( HXACT x, IOMPORT IomPort, u16 mode )
{
	// Correct byte ordering.
	u16 nordMode = HtoLE_16( mode );

	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Schedule the action.
	GWAddAction( x, (u8)( OP_MC_SETMODE ), &nordMode, sizeof(nordMode), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Schedule post-processing, which will copy new mode value to middleware's image.
	mode &= ~CT_RUN_ENABLE;		// Mask off Run bit.
	return GWAddPost( x, PushCmdArg( x, &mode, sizeof(mode) ), &x->mm->IomObj[IomPort].Custom.Obj2631.ChanAttrIn.Mode, sizeof(mode), memcpy_pp, 0 );
}

////////////////////////////////////////////////////////////////////////////////
// Return the operating mode for input counter channel (from middleware's image).

// Scheduler.
EXPORT(GWERR) S26_Sched2631_GetMode( HXACT x, IOMPORT IomPort, u16 *mode )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Schedule post-processing step: copy mode image to application buffer.
	return GWAddPost( x, &x->mm->IomObj[IomPort].Custom.Obj2631.ChanAttrIn.Mode, mode, sizeof(u16), memcpy_pp, 0 );
}

/////////////////////////////////////////////////////////////////////
// Derived mode-setting functions.

// Scheduler helper for derived mode-setting functions.
static GWERR SetMode2631( HXACT x, IOMPORT IomPort, u16 mode )
{
	// Halt channel and configure its operating mode.
  	S26_Sched2631_SetMode( x, IomPort, (u16)( mode & ~CT_RUN_ENABLE ) );

	// Start channel running in the new mode.
  	S26_Sched2631_SetMode( x, IomPort, (u16)( mode | CT_RUN_ENABLE ) );

	// Transfer preload register to core.
	return S26_Sched2631_SetControlReg( x, IomPort, CNT_TRIG_LOAD );
}

// Scheduler: set counter mode for encoder interface.
EXPORT(GWERR) S26_Sched2631_SetModeEncoder( HXACT x, IOMPORT IomPort, u16 ActLowX, u16 HardPreload, u16 ClkMode )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_DONTCARE						// Output pin is not used, so mode is not relevant.
		| ( ActLowX ? CT_XP_ACTLOW : CT_XP_ACTHIGH )		// Optional: select index polarity if used as preload trigger.
		| ( HardPreload ? CT_PL_INDEX : CT_PL_SOFTONLY )	// Optional: copy Preload0 to core upon index leading edge.
		| CT_LAT_SOFTONLY									// Core copied to latch only upon software command.
		| CT_CET_CONFIG										// Enable counting upon configuration.
		| CT_OP_DONTCARE									// Output pin is not used, so polarity is not relevant.
		| ( ClkMode << CT_M_SHFT )							// Clock mode may have any value from 0 to 6.
		| CT_CD_NEVER										// Never disable counting after started.
		| CT_PLM_SINGLE										// Use Preload0 only.
		| CT_XC_EXTERNAL );									// If used, index signal is applied to external index pin.

	// Program counter channel's operating mode.
	return SetMode2631( x, IomPort, mode );
}

// Scheduler: set counter mode for pulse generator.
// Pulse is triggered by software, or optionally, by hardware trigger on index input.
// ClkB input must be configured to count down.
EXPORT(GWERR) S26_Sched2631_SetModePulseGen( HXACT x, IOMPORT IomPort, u16 ActLowX, u16 HardTrig, u16 ActLowOut )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_ZEROACTIVE						// Output pin goes active (pulse goes inactive) when core reaches zero counts.
		| ( ActLowX ? CT_XP_ACTLOW : CT_XP_ACTHIGH )		// Optional: select index polarity if used as harware pulse trigger.
		| ( HardTrig ? CT_PL_INDEX : CT_PL_SOFTONLY )		// Optional: copy Preload0 to core upon index leading edge (triggers pulse output).
		| CT_LAT_SOFTONLY									// Core copied to latch only upon software command.
		| CT_CET_CONFIG										// Enable counting upon configuration.
		| ( ActLowOut ? CT_OP_ACTLOW : CT_OP_ACTHIGH )		// Select polarity of output pulse.
		| CT_M_INTERNAL										// Use internal 10MHz clock as the clock source.
		| CT_CD_ZERO										// Disable counting when core reaches zero counts.
		| CT_PLM_SINGLE										// Use Preload0 only.
		| CT_XC_EXTERNAL );									// If used, index signal is applied to external index pin.

	// Program counter channel's operating mode.
	return SetMode2631( x, IomPort, mode );
}

// Scheduler: set counter mode for pwm generator.
// Upon entry, preload 1/0 registers should have been previously set to desired on/off times.
EXPORT(GWERR) S26_Sched2631_SetModePwmGen( HXACT x, IOMPORT IomPort, u16 ActLowOut )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_ZEROTOGGLE						// Output pin toggles state when core reaches zero counts.
		| CT_XP_DONTCARE									// Index input is not used, so its polarity is don't care.
		| CT_PL_ZERO										// Preload the alternate preload register when core reaches zero counts.
		| CT_LAT_SOFTONLY									// Core copied to latch only by soft command.
		| CT_CET_CONFIG										// Enable counting upon configuration.
		| ( ActLowOut ? CT_OP_ACTLOW : CT_OP_ACTHIGH )		// Select polarity of output pulse.
		| CT_M_INTERNAL										// Use internal 10MHz clock as the clock source.
		| CT_CD_NEVER										// Never disable counting after started.
		| CT_PLM_BOTH										// Use both preload registers.
		| CT_XC_EXTERNAL );									// Index input is not used, but we mark it "external" anyway.

	// Program counter channel's operating mode.
	return SetMode2631( x, IomPort, mode );
}

// Scheduler: set counter mode for pulse width measurement.
EXPORT(GWERR) S26_Sched2631_SetModePulseMeas( HXACT x, IOMPORT IomPort, u16 ActLowX )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_DONTCARE						// Output pin is not used, so mode is not relevant.
		| ( ActLowX ? CT_XP_ACTHIGH : CT_XP_ACTLOW )		// Select index polarity.  This selects the EDGE, which is opposite of LEVEL.
		| CT_PL_INDEX
		| CT_LAT_INDEX
		| CT_CET_INDEX										// Enable counting on first gate leading edge.
		| CT_OP_DONTCARE									// Output pin is not used, so polarity is not relevant.
		| CT_M_INTERNAL										// Use internal 10MHz clock as the clock source.
		| CT_CD_INDEX
		| CT_PLM_SINGLE										// Use Preload0 only.
		| CT_XC_EXTERNAL );

	// Program counter channel's operating mode.
	return SetMode2631( x, IomPort, mode );
}

// Scheduler: set counter mode for period measurement.  Input signal is applied to index input.
EXPORT(GWERR) S26_Sched2631_SetModePeriodMeas( HXACT x, IOMPORT IomPort, u16 ActLowX )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_DONTCARE						// Output pin is not used, so mode is not relevant.
		| ( ActLowX ? CT_XP_ACTLOW : CT_XP_ACTHIGH )		// Index polarity is selectable in case one signal edge has more jitter.
		| CT_PL_INDEX										// Preload zero on index leading edge.
		| CT_LAT_INDEX										// Latch counts on index leading edge.
		| CT_CET_INDEX										// Enable counting on first input signal leading edge.
		| CT_OP_DONTCARE									// Output pin is not used, so polarity is not relevant.
		| CT_M_INTERNAL										// Use internal 10MHz clock as the clock source.
		| CT_CD_NEVER										// Never disable counting after started.
		| CT_PLM_SINGLE										// Use Preload0 only.
		| CT_XC_EXTERNAL );									// Input signal to be measured is applied to external index pin.

	// Set Preload0 value to zero; this will be automatically copied to the core each time a new measurement begins.
	S26_Sched2631_SetPreload( x, IomPort, CNT_REG_PRELOAD0, 0 );

	// Program counter channel's operating mode.
	return SetMode2631( x, IomPort, mode );
}

// Scheduler: set counter mode for frequency measurement.
// The index, which is used as a CountEnable gate, can be either external or internal depending on IntGate.
EXPORT(GWERR) S26_Sched2631_SetModeFreqMeas( HXACT x, IOMPORT IomPort, u16 igate )
{
	// Specify counter channel's operating mode.
	u16 mode = (u16)( CT_OM_DONTCARE						// Output pin is not used, so mode is not relevant.
		| CT_XP_ACTHIGH										// Sample intervals begin on gate (index) rising edge.
		| CT_PL_INDEX										// Preload zero on gate (index input) leading edge.
		| CT_LAT_INDEX										// Latch on gate (index input) leading edge.
		| CT_CET_INDEX										// Enable counting on first gate leading edge.
		| CT_OP_DONTCARE									// Output pin is not used, so polarity is not relevant.
		| CT_M_MONOARISE									// Count rising edges on ClkA input during gate interval.
		| CT_CD_NEVER										// Never disable counting after started.
		| CT_PLM_SINGLE										// Use Preload0 only, which must be preset to zero.
		| ( igate ? CT_XC_INTERNAL : CT_XC_EXTERNAL ) );	// Optional: use internal gate instead of index.

	// Set Preload0 value to zero; this will be automatically copied to the core each time a new measurement begins.
	S26_Sched2631_SetPreload( x, IomPort, CNT_REG_PRELOAD0, 0 );

	// Program counter channel's operating mode.
	return SetMode2631( x, IomPort, mode );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Output Counter and Control /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
// Enable output.
// Imports:
//	enable = 8-bit value: 1-enable; 0-disable.

// Scheduler
EXPORT(GWERR) S26_Sched2631_OutEnable( HXACT x, IOMPORT IomPort, u8 enable )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Schedule the action.
	return GWAddAction( x, (u8)( enable ? OP_MC_OUTENABLE : OP_MC_OUTDISABLE ), NULL, 0, NULL, STATMASK_NONE, NoEndian_0x0 );
}

//////////////////////////////////////////////////////////
// Set output mode.
// Imports:
//	mode = 8-bit mode register value.

// Scheduler
EXPORT(GWERR) S26_Sched2631_SetOutMode( HXACT x, IOMPORT IomPort, u8 mode )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Schedule the action.
 	return GWAddAction( x, (u8)( OP_MC_SETOUTMODE ), &mode, sizeof(mode), NULL, STATMASK_NONE, NoEndian_0x0 );
}

//////////////////////////////////////////////////////////
// Program output amplitude value.
// Imports:
//	amplitude = 16-bit amplitude value.

// Scheduler
EXPORT(GWERR) S26_Sched2631_SetAmpl( HXACT x, IOMPORT IomPort, u32 amplitude )
{
	// Correct byte ordering.
	u32 nordValue = HtoLE_32( amplitude );

	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Schedule the action.
	return GWAddAction( x, (u8)( OP_MC_SETAMPLITUDE ), &nordValue, sizeof(nordValue), NULL, STATMASK_NONE, NoEndian_0x0 );
}

/////////////////////////////////////////////////////////////////////
// Program output frequency value.
// Imports:
//	frequency = 32-bit frequency value.

// Scheduler.
EXPORT(GWERR) S26_Sched2631_SetOutputFrq( HXACT x, IOMPORT IomPort, u32 frequency )
{
	// Correct byte ordering for the preload value.
	u32 nordValue = HtoLE_32( frequency );

	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Preload the counter.
	GWAddAction( x, (u8)( OP_MC_SETOUTFRQ ), &nordValue, sizeof(nordValue), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Schedule post-processing step: copy preload value to middleware image.
	return GWAddPost( x, PushCmdArg( x, &frequency, sizeof(frequency) ), &x->mm->IomObj[IomPort].Custom.Obj2631.ChanAttrOut.outfreq, sizeof(frequency), memcpy_pp, 0 );
}

/////////////////////////////////////////////////////////////////////
// Program PWM frequency value.
// Imports:
//	frequency = 32-bit frequency value.

// Scheduler.
EXPORT(GWERR) S26_Sched2631_SetPWMFrq( HXACT x, IOMPORT IomPort, u32 frequency )
{
	// Correct byte ordering for the preload value.
	u32 nordValue = HtoLE_32( frequency );

	// Select the iom.
	SelectIom( &x, IomPort, 2631 );

	// Preload the counter.
	GWAddAction( x, (u8)( OP_MC_SETPWMFRQ ), &nordValue, sizeof(nordValue), NULL, STATMASK_NONE, NoEndian_0x0 );

	// Schedule post-processing step: copy preload value to middleware image.
	return GWAddPost( x, PushCmdArg( x, &frequency, sizeof(frequency) ), &x->mm->IomObj[IomPort].Custom.Obj2631.ChanAttrOut.pwmfreq, sizeof(frequency), memcpy_pp, 0 );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////  MODEL 2650 RELAY BOARD ACTIONS  ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


// Scheduler.  Return relay driver physical states.
EXPORT(GWERR) S26_Sched2650_GetInputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2650 );

	// Schedule the iom action for execution.
	return GWAddAction( x, OP_RLY_GETINPUTS, NULL, ZERO_SIZE, states, STATMASK_NONE, NoEndian_8x1 );
}

// Scheduler.  Return programmed relay states.
EXPORT(GWERR) S26_Sched2650_GetOutputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2650 );

	// Schedule the iom action for execution.
	return GWAddAction( x, OP_RLY_GETOUTPUTS, NULL, ZERO_SIZE, states, STATMASK_NONE, NoEndian_8x1 );
}

// Scheduler.  Program relay states.
EXPORT(GWERR) S26_Sched2650_SetOutputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2650 );

	// Schedule the iom action for execution.
	return GWAddAction( x, OP_RLY_SETOUTPUTS, states, sizeof(u8), NULL, STATMASK_2650, NoEndian_0x0 );
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////  MODEL 2652 DIGITAL I/O BOARD ACTIONS  ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Return/set the operating modes ( PWM vs. Normal) for channels 0-7.

// Scheduler (set modes).

EXPORT(GWERR) S26_Sched2652_SetModes( HXACT x, IOMPORT IomPort, u8 *modes )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2652 );

	// Schedule the SetModes action for execution.
	return GWAddAction( x, OP_RLY_SETMODES, modes, sizeof(u8), NULL, STATMASK_NONE, NoEndian_0x0 );
}

// Scheduler (get modes).
EXPORT(GWERR) S26_Sched2652_GetModes( HXACT x, IOMPORT IomPort, u8 *modes )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2652 );

	// Schedule the GetModes action for execution.
	return GWAddAction( x, OP_RLY_GETMODES, NULL, ZERO_SIZE, modes, STATMASK_NONE, NoEndian_8x1 );
}

/////////////////////////////////////////////////////////////////////////////////
// Return/set the PWM on/off times for a channel that operates in the PWM mode.//

// Scheduler.
EXPORT(GWERR) S26_Sched2652_SetPwmRatio( HXACT x, IOMPORT IomPort, u8 chan, u8 OnTime, u8 OffTime )
{
	// Construct ordered argument list for the iom command.
	u8 pwminfo[3];
	pwminfo[0] = chan;
	pwminfo[1] = OnTime;
	pwminfo[2] = OffTime;

	// Select the iom.
	SelectIom( &x, IomPort, 2652 );

	// Schedule the SetRatio action for execution.
	return GWAddAction( x, OP_RLY_SETPWMRATIO, &pwminfo, sizeof(pwminfo) / sizeof(pwminfo[0]), NULL, STATMASK_NONE, NoEndian_0x0 );
}

// Scheduler.
EXPORT(GWERR) S26_Sched2652_GetPwmRatio( HXACT x, IOMPORT IomPort, u8 chan, u8 *OnTime, u8 *OffTime )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2652 );

	// Schedule the GetPwmRatio action for execution.
	GWAddAction( x, OP_RLY_GETPWMRATIO, &chan, sizeof(u8), x->PPRspBuf, STATMASK_NONE, NoEndian_8x2 );

	// Schedule: copy on/off times to application buffers.
	GWAddPost( x, x->PPRspBuf, OnTime, sizeof(u8), memcpy_pp, 0 );
	return GWAddPost( x, x->PPRspBuf + 1, OffTime, sizeof(u8), memcpy_pp, 0 );
}

// Scheduler.  Return relay driver physical states.
EXPORT(GWERR) S26_Sched2652_GetInputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2652 );

	// Schedule the iom action for execution.
	return GWAddAction( x, OP_RLY_GETINPUTS, NULL, ZERO_SIZE, states, STATMASK_NONE, NoEndian_8x1 );
}

// Scheduler.  Return programmed relay states.
EXPORT(GWERR) S26_Sched2652_GetOutputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2652 );

	// Schedule the iom action for execution.
	return GWAddAction( x, OP_RLY_GETOUTPUTS, NULL, ZERO_SIZE, states, STATMASK_NONE, NoEndian_8x1 );
}

// Scheduler.  Program relay states.
EXPORT(GWERR) S26_Sched2652_SetOutputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2652 );

	// Schedule the iom action for execution.
	return GWAddAction( x, OP_RLY_SETOUTPUTS, states, sizeof(u8), NULL, STATMASK_2652, NoEndian_0x0 );
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////  MODEL 2653 DIGITAL I/O BOARD ACTIONS  ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Return/set the operating modes ( PWM vs. Normal) for channels 0-15.

// Scheduler (set modes).

EXPORT(GWERR) S26_Sched2653_SetModes( HXACT x, IOMPORT IomPort, u8 *modes )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2653 );

	// Schedule the SetModes action for execution.
	return GWAddAction( x, OP_RLY_SETMODES, modes, 2 * sizeof(u8), NULL, STATMASK_NONE, NoEndian_0x0 );
}

// Scheduler (get modes).
EXPORT(GWERR) S26_Sched2653_GetModes( HXACT x, IOMPORT IomPort, u8 *modes )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2653 );

	// Schedule the GetModes action for execution.
	return GWAddAction( x, OP_RLY_GETMODES, NULL, ZERO_SIZE, modes, STATMASK_NONE, NoEndian_8x2 );
}

/////////////////////////////////////////////////////////////////////////////////
// Return/set the PWM on/off times for a channel that operates in the PWM mode.//

// Scheduler.
EXPORT(GWERR) S26_Sched2653_SetPwmRatio( HXACT x, IOMPORT IomPort, u8 chan, u8 OnTime, u8 OffTime )
{
	// Construct ordered argument list for the iom command.
	u8 pwminfo[3];
	pwminfo[0] = chan;
	pwminfo[1] = OnTime;
	pwminfo[2] = OffTime;

	// Select the iom.
	SelectIom( &x, IomPort, 2653 );

	// Schedule the SetRatio action for execution.
	return GWAddAction( x, OP_RLY_SETPWMRATIO, &pwminfo, sizeof(pwminfo) / sizeof(pwminfo[0]), NULL, STATMASK_NONE, NoEndian_0x0 );
}

// Scheduler.
EXPORT(GWERR) S26_Sched2653_GetPwmRatio( HXACT x, IOMPORT IomPort, u8 chan, u8 *OnTime, u8 *OffTime )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2653 );

	// Schedule the GetPwmRatio action for execution.
	GWAddAction( x, OP_RLY_GETPWMRATIO, &chan, sizeof(u8), x->PPRspBuf, STATMASK_NONE, NoEndian_8x2 );

	// Schedule: copy on/off times to application buffers.
	GWAddPost( x, x->PPRspBuf, OnTime, sizeof(u8), memcpy_pp, 0 );
	return GWAddPost( x, x->PPRspBuf + 1, OffTime, sizeof(u8), memcpy_pp, 0 );
}

// Scheduler.  Return relay driver physical states.
EXPORT(GWERR) S26_Sched2653_GetInputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2653 );

	// Schedule the iom action for execution.
	return GWAddAction( x, OP_RLY_GETINPUTS, NULL, ZERO_SIZE, states, STATMASK_NONE, NoEndian_8x2 );
}

// Scheduler.  Return programmed relay states.
EXPORT(GWERR) S26_Sched2653_GetOutputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2653 );

	// Schedule the iom action for execution.
	return GWAddAction( x, OP_RLY_GETOUTPUTS, NULL, ZERO_SIZE, states, STATMASK_NONE, NoEndian_8x2 );
}

// Scheduler.  Program relay states.
EXPORT(GWERR) S26_Sched2653_SetOutputs( HXACT x, IOMPORT IomPort, u8 *states )
{
	// Select the iom.
	SelectIom( &x, IomPort, 2653 );

	// Schedule the iom action for execution.
	return GWAddAction( x, OP_RLY_SETOUTPUTS, states, 2 * sizeof(u8), NULL, STATMASK_2653, NoEndian_0x0 );
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////  MODULE REGISTRATION  //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////
// Detect and fetch info from all connected ioms.

static GWERR DetectAllIoms( MM_OBJ *mm, u32 msec, u16 *IomType, u16 *FwVersion, u32 retries )
{
	XACT	*x;
	GWERR	rtnval;
	IOMPORT	i;
	u16		mask;
	u16		LinkFlags;

	// Fetch link flags for all detected ioms.  Abort if failed.
	x = XactAlloc( mm, LOGDEV_GATEWAY, retries );							// Create a new transaction object.
 	S26_Sched2601_GetLinkStatus( x, &LinkFlags );							// Schedule GetLinkStatus action.
	if ( ( rtnval = S26_SchedExecute( x, msec, NULL ) ) == GWERR_NONE )	// Execute transaction, and if successful ...
	{
		// Create a new transaction object.
		x = XactAlloc( mm, LOGDEV_GATEWAY, retries );

		// For each iom port ...
		for ( i = 0, mask = 1; i < NUM_IOMPORTS; i++, mask <<= 1 )
		{
			// If an iom was detected at this port then schedule RST negation and IomInfo fetching.
			if ( ( LinkFlags & mask ) != 0 )
				SchedDetectIom( x, i, &IomType[i], &FwVersion[i] );
		}

		// Fetch product id and firmware version for all detected ioms.
		rtnval = S26_SchedExecute( x, msec, NULL );
	}

	// Return any detected non-iom-specific errors.
	return rtnval;
}

////////////////////////////////////////////////////////////////////////////////////
//  Detect and register all iom's.  Clear the RST status flag of all detected iom's.
//  Imports:
//  IomCount   = pointer to application word buffer that is to receive the iom count.
//  IomTypeList = pointer to application word[16] buffer that is to receive the iom types.
//  Returns gateway error flags.
//
//  Private helper.
static GWERR RegisterAllIoms( MM_OBJ *mm, u32 msec, u16 *IomCount, u16 *IomTypeList, u8 *IomStatus, u32 retries )
{
	u16		InfoIomType[NUM_IOMPORTS]   = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
	u16		InfoFwVersion[NUM_IOMPORTS] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
	int		i;

	// Detect all ioms and get their type and version info.  If we couldn't detect iom's and/or get the iom info
	// due to communication problems then we will leave the middleware's iom count and registered types unmodified.
	GWERR rtnval = DetectAllIoms( mm, msec, InfoIomType, InfoFwVersion, retries );

	// Disregard any module-specific errors that may have been detected, as these don't affect module detection.
	if ( ( rtnval & GWERRMASK ) == GWERR_IOMSPECIFIC )
		rtnval = GWERR_NONE;

	// If we successfully acquired the iom info ...
	if ( rtnval == GWERR_NONE )
	{
		// Register all iom's.
		for ( i = 0; i < NUM_IOMPORTS; i++ )
			RegisterOneIom( &mm->IomObj[i], msec, InfoIomType[i], InfoFwVersion[i], retries );

		// Copy registered iom count to application buffer if one is specified.
		if ( IomCount != NULL )	
			*IomCount = (u16)mm->IomCount;

		// Copy registered iom types to application buffer if one is specified.
		if ( IomTypeList != NULL )
		{
			for ( i = 0; i < NUM_IOMPORTS; i++ )
				*IomTypeList++ = mm->IomObj[i].pIomAttr->IomType;
		}

		// If application buffer is to receive simulated iom status flags ...
		if ( IomStatus != NULL )
		{
			// Copy status flags to application buffer.
			for ( i = 0; i < NUM_IOMPORTS; i++ )
				IomStatus[i] = mm->IomObj[i].SimStatus;

			// Signal error if any iom flags were asserted.
			for ( i = 0; i < NUM_IOMPORTS; i++ )
			{
				if ( IomStatus[i] != STATUS_NONE )
				{
					rtnval = GWERR_IOMSPECIFIC | i;
					break;
				}
			}
		}
	}

	// Indicate error, if any.
	return rtnval;
}

// Public function.
EXPORT(GWERR) S26_RegisterAllIoms( HBD hbd, u32 msec, u16 *IomCount, u16 *IomTypeList, u8 *IomStatus, u32 retries )
{
	// Abort if board handle is not legal.
	if ( hbd >= MMCount )
		return GWERR_BADVALUE | 1;

	// Detect and register all ioms.
	return RegisterAllIoms( &obj[hbd], msec, IomCount, IomTypeList, IomStatus, retries );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////  UTILITY FUNCTIONS  ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
// Utilities for type-challenged environments (e.g., VisualBasic).

EXPORT(u32 *) S26_CastAsLong( void *pval )
{
	return (u32 *)pval;
}

EXPORT(u16 *) S26_CastAsShort( void *pval )
{
	return (u16 *)pval;
}

EXPORT(void) S26_CopyAny( void *dst, const void *src, int nbytes )
{
	memcpy( dst, src, nbytes );
}

EXPORT(u32) S26_Bitmask( u32 bitnum )
{
	return ( 1 << bitnum );
}
