///////////////////////////////////////////////////////////////
// Module    : bfr_demo.c
// Function  : Autodetect and exercise 2608 and 2620 i/o modules.
// Target OS : Linux
///////////////////////////////////////////////////////////////

//For compiling the demo in cpp instead of c.
#ifdef __cplusplus
extern "C" {
#endif

  #include "app2600.h"    // Linux api to 2600 middleware

#ifdef __cplusplus
}
#endif

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>


// CONSTANTS //////////////////////////////////////////////////////////////////
// NOTE TODO : This is weird code in the sensoray driver. Should really be a const.
const char mm_ip_addr_[] 	=	{"10.10.10.1"};		// Set this to the MM's IP address.

const int mm_handle_ =0;					// This is the first MM in the system, so it is number 0.

const int timeout_gateway_ms_	= 100;					// This many milliseconds before timing out or retry gateway transactions.
const int timeout_comport_ms_	= 100;					// This many milliseconds before timing out or retry comport transactions.

const int retries_com_ = 50;					// Do up to this many comport retries.
const int retries_gateway_	 = 50;					// Do up to this many gateway retries.

// 2620 channel usage for this app:
const int s2620_channel_width_ = 0;					// Pulse width measurement.
const int s2620_channel_freq_	 = 1;					// Frequency counter.
const int s2620_channel_pwm_ = 2;					// Pulse width modulated output.
const int s2620_channel_encoder_ = 3;					// Incremental encoder input.

// Comport usage for this app.  With two null-modem cables, we can loop back two ports into two other ports:
const u8 com_src_a_ = 	LOGDEV_COM2;			// Transmit A.
const u8 com_dest_a_ = LOGDEV_COM1;			// Receive A.
const u16 com_baud_a_ = SIO_BR_9600;			// Baudrate for A.

const u8 com_src_b_ = LOGDEV_COM4;			// Transmit B.
const u8 com_dest_b_ = LOGDEV_COM3;			// Receive B.
const u16 com_baud_b_ = SIO_BR_115200;		// Baudrate for B.

const int com_reject_ignore_ = 0;					// Ignore the comport REJ flag.
const int com_reject_evaluate_	 = 1;					// Treat comport REJ flag as an error.

// PUBLIC STORAGE ///////////////////////////////////////////////////////////////

int		iters = 0;						// Number of times through the control loop so far.

u16		nboards;						// Number of detected iom's.
u16		IomType[16];					// Detected iom types.
u8		IomStatus[16];					// Iom status info.

u8		nAouts2608[16];					// Number of dac channels (applies to 2608 only).

// Input data from the i/o system.
u16		LinkFlags;						// IOM port Link status.
u8		LockFlags;						// Interlock power status.
u8		DinStates[6];					// Digital input states (48 channels).
DOUBLE	Ain[16];						// Analog input voltages.

// Output data to the i/o system.
u8		RelayStates			= 0;		// Relay states.
u8		DoutStates[6]		= { 0, };	// Digital output states (48 channels).
DOUBLE	Aout[MAX_NUM_AOUTS]	= { 0, };	// Analog output voltages.
u32		Counts[4]			= { 0, };	// Counter data.
u16		Timestamp[4]		= { 0, };	// Counter timestamps.

// FORWARD REFERENCES ////////////////////////////////////////////////////////////

static int	io_exec( void* x );
static void io_control_main( void );
static int	DetectAllIoms( void );
static void*   CreateTransaction( HBD hbd );

static void	kbopen( void );
static void	kbclose( void );
static int	kbhit( void );
static int	kbread( void );

// EXECUTABLE ////////////////////////////////////////////////////////////////////

int main()
{
	u32 faults;

	kbopen();

	// Open the 2600 api.  Declare one MM in system.
	if ( ( faults = S26_DriverOpen( 1 ) ) != 0 )
	{
		printf( "DriverOpen() fault: %d\n", (int)faults );
	}
	else
	{
		// Open the MM.
		if ( ( faults = S26_BoardOpen( mm_handle_, 0, mm_ip_addr_ ) ) != 0 )
			printf( "BoardOpen() fault: %d\n", (int)faults );

		// If MM was successfully opened ...
		else
		{
			// Reset the I/O system.
			S26_ResetNetwork( mm_handle_ );

			// Register all iom's.  If no errors, execute the I/O control loop until it is terminated.
			if ( DetectAllIoms() )
			{
				// Execute the i/o control loop until it terminates.
				io_control_main();
			}
		}

		// Close the api library.
		S26_DriverClose();
	}

	kbclose();

	return 0;
}

////////////////////////////////
// Display gateway error info.

void ShowErrorInfo( u32 gwerr, u8 *IomStatus )
{
	char	errmsg[128];
	int		ExtraInfo = gwerr & ~GWERRMASK;
	u8		status;

	switch ( gwerr & GWERRMASK )
	{
	case GWERR_IOMSPECIFIC:

		status = IomStatus[ExtraInfo];

		sprintf( errmsg, "Iom-specific error on iomport %d", ExtraInfo );

		if ( IomStatus )
		{
			switch( IomType[ExtraInfo] )
			{
			case 2608:
				if ( status & STATUS_2608_CALERR )	strcat( errmsg, ": using default cal values" );
				break;

			case 2610:
				if ( status & STATUS_2610_STRM )	strcat( errmsg, ": serial stream error" );
				break;

			case 2650:
				if ( status & STATUS_2650_DRVR )	strcat( errmsg, ": relay coil driver fault" );
				if ( status & STATUS_2650_STRM )	strcat( errmsg, ": serial stream error" );
				break;
			case 2652:
				if ( status & STATUS_2652_STRM )	strcat( errmsg, ": serial stream error" );
				break;
			}
		}

		break;

	case GWERR_BADVALUE:		sprintf( errmsg, "Illegal value for argument %d", ExtraInfo );				break;
	case GWERR_IOMCLOSED:		sprintf( errmsg, "Iom is not open" );												break;
	case GWERR_IOMERROR:		sprintf( errmsg, "Iom CERR asserted" );												break;
	case GWERR_IOMNORESPOND:	sprintf( errmsg, "Bad module response from iomport %d", ExtraInfo );		break;
	case GWERR_IOMRESET:		sprintf( errmsg, "Module RST asserted" );											break;
	case GWERR_IOMTYPE:			sprintf( errmsg, "Action not supported by iom type" );								break;
	case GWERR_MMCLOSED:		sprintf( errmsg, "MM is closed" );													break;
	case GWERR_MMNORESPOND:		sprintf( errmsg, "MM response timed out" );											break;
	case GWERR_PACKETSEND:		sprintf( errmsg, "Failed to send cmd packet" );										break;
	case GWERR_TOOLARGE:		sprintf( errmsg, "Command or response packet too large" );							break;
	case GWERR_XACTALLOC:		sprintf( errmsg, "Transaction Object allocation problem" );							break;
	default:					sprintf( errmsg, "Unknown error" );													break;
	}

	printf( "Error: 0x%X (%s), iters = %d.\n", (int)gwerr, errmsg, iters );
}

//////////////////////////////////////
// Process comport error, if any.

u32 ComError( u32 gwerr, const char *fname, int evalComReject )
{
	char errmsg[128];

	// If transaction error was detected ...
	if ( gwerr & GWERRMASK )
	{
		switch ( gwerr & GWERRMASK )
		{
		case GWERR_BADVALUE:		sprintf( errmsg, "Illegal value for argument %lu", gwerr & ~GWERRMASK );	break;
		case GWERR_IOMRESET:		sprintf( errmsg, "Module RST asserted" );									break;
		case GWERR_MMCLOSED:		sprintf( errmsg, "MM is closed" );											break;
		case GWERR_MMNORESPOND:		sprintf( errmsg, "MM response timed out" );									break;
		case GWERR_PACKETSEND:		sprintf( errmsg, "Failed to send cmd packet" );								break;
		case GWERR_TOOLARGE:		sprintf( errmsg, "Command or response packet too large" );					break;
		case GWERR_XACTALLOC:		sprintf( errmsg, "Transaction Object allocation problem" );					break;
		default:					sprintf( errmsg, "Unknown error" );											break;
		}
	}

	// If comport error was detected ...
	else if ( gwerr & COM_FRAMINGERROR )	sprintf( errmsg, "Framing error" );
	else if ( gwerr & COM_PARITYERROR )		sprintf( errmsg, "Parity error" );
	else if ( gwerr & COM_OVERFLOWERROR )	sprintf( errmsg, "Receiver overflow" );

	// If comport command was rejected and checking for this condition is enabled ...
	else if ( ( ( gwerr & COM_REJECTED ) != 0 ) && ( evalComReject == com_reject_evaluate_ ) )
		sprintf( errmsg, "Command rejected" );

	// If no errors were detected ...
	else
		return GWERR_NONE;

	printf( "%s() error: 0x%lX (%s), iters = %d.\n", fname, gwerr, errmsg, iters );
	return gwerr;
}

/////////////////////////////////////////////////////////////////////////////
// Detect and register all i/o modules connected to the 2601 main module.

int DetectAllIoms( void )
{
	int	i;
	u32	faults;

	// Detect and register all iom's.
	if ( ( faults = S26_RegisterAllIoms( mm_handle_, timeout_gateway_ms_, &nboards, IomType, IomStatus, retries_gateway_ ) ) != 0 )
	{
		ShowErrorInfo( faults, IomStatus );
		return 0;	// failed.
	}

	// List the discovered iom's.
	printf( "DETECTED IOM'S:\n" );
	for ( i = 0; i < 16; i++ )
	{
		if ( IomType[i] )
			printf( " port %2.2d: %4.4d\n", i, IomType[i] );
	}

	return 1;	// success.
}


///////////////////////////////////////////////////////////////
// Schedule some gateway I/O into transaction object x.

static void sched_io( void* x )
{
	int		chan;
	u8		i;

	// Schedule some 2601 actions.
	S26_Sched2601_GetLinkStatus( x, &LinkFlags );
	S26_Sched2601_GetInterlocks( x, &LockFlags );

	// Schedule some iom actions.  For each iom port ...
	for ( i = 0; i < 16; i++ )
	{
		// Schedule some i/o actions based on module type.
		switch( IomType[i] )
		{
		case 2608:
			// Update reference standards and read analog inputs
			S26_Sched2608_GetCalData( x, i, 0 );					// Auto-cal.  Only needed ~once/sec, but we always do it for simplicity.
			S26_Sched2608_GetAins( x, i, Ain, ADC_INTEGRATED );	// Fetch the analog inputs.

			// Program all analog outputs.
			for ( chan = 0; chan < (int)nAouts2608[chan]; chan++ )
				S26_Sched2608_SetAout( x, i, (u8)chan, Aout[chan] );
			break;

		case 2610:
			S26_Sched2610_SetOutputs( x, i, DoutStates );		// Program the dio outputs.
			S26_Sched2610_GetInputs( x, i, DinStates );			// Fetch physical inputs.
			break;

		case 2620:
			// Transfer counter cores to latches.
			S26_Sched2620_SetControlReg( x, i, s2620_channel_pwm_, 2 );
			S26_Sched2620_SetControlReg( x, i, s2620_channel_encoder_, 2 );

			// Read latches.
			for ( chan = 0; chan < 4; chan++ )
				S26_Sched2620_GetCounts( x, i, (u8)chan, &Counts[chan], &Timestamp[chan] );	// Fetch values from all channel latches.
			break;

		case 2650:
			S26_Sched2650_SetOutputs( x, i, &RelayStates );		// Program the relays.
			break;
		case 2652:
			S26_Sched2652_SetOutputs( x, i, &RelayStates );		// Program the relays.
			break;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize two serial comports, one of which may be looped back into the other via a null-modem cable.
// Returns zero if successful.

static int SerialInit( u8 ComSrc, u8 ComDst, u16 BaudRate )
{
	u32	comstatus;

	// Source (transmitter) port.
	comstatus = S26_ComSetMode( mm_handle_,
		ComSrc,
		BaudRate,
		SIO_PHY_RS232 | SIO_PARITY_NONE | SIO_DATA_8 | SIO_STOP_1 | SIO_FLOW_OFF,
		SIO_LED_TRANSMIT,
		timeout_comport_ms_,
		retries_com_ );
	if ( ComError( comstatus, "S26_ComSetMode", com_reject_evaluate_ ) )
		return 1;

	comstatus = S26_ComOpen( mm_handle_, ComSrc, timeout_comport_ms_, retries_com_ );
	if ( ComError( comstatus, "S26_ComOpen", com_reject_evaluate_ ) )
		return 1;

	// Destination (receiver) port.
	comstatus = S26_ComSetMode( mm_handle_,
		ComDst,
		BaudRate,
		SIO_PHY_RS232 | SIO_PARITY_NONE | SIO_DATA_8 | SIO_STOP_1 | SIO_FLOW_OFF,
		SIO_LED_RECEIVE,
		timeout_comport_ms_,
		retries_com_ );
	if ( ComError( comstatus, "S26_ComSetMode", com_reject_evaluate_ ) )
		return 1;

	comstatus = S26_ComOpen( mm_handle_, ComDst, timeout_comport_ms_, retries_com_ );
	if ( ComError( comstatus, "S26_ComOpen", com_reject_evaluate_ ) )
		return 1;

	return 0;
}

//////////////////////////////////////////////
// Do some serial I/O.
// Returns zero if successful.

static int SerialIo( u8 ComSrc, u8 ComDst )
{
	static int	totalSent = 0;

	int		msglen;
	char	SndBuf[1024];		// String to be sent.
	char	RcvBuf[1024];		// Buffer that will receive the string.
	u16		BufLen;				// Max number of characters to receive.
	u32		comstatus;

	// Construct a message.
	sprintf( SndBuf, "12345678901234567890123456:%d\n", iters );
	msglen = strlen( SndBuf );

	// Attempt to send message.  Ignore the COM_REJECT error, which is caused by insufficient transmit buffer free space.
	comstatus = S26_ComSend( mm_handle_, ComSrc, (u8 *)SndBuf, (u16)msglen, timeout_comport_ms_, retries_com_ );
	if ( ComError( comstatus, "S26_ComSend", com_reject_ignore_ ) )
	{
		printf( "TotalSent = %d\n", totalSent );
		return 1;
	}
	else
		totalSent += msglen;

	// Receive all available data from the receiving comport's receive buffer.
	BufLen = sizeof(RcvBuf);     // Max number of characters to receive.
	comstatus = S26_ComReceive( mm_handle_, ComDst, (u8 *)RcvBuf, &BufLen, timeout_comport_ms_, retries_com_ );
	if ( ComError( comstatus, "S26_ComReceive", com_reject_evaluate_ ) )
	{
		printf( "TotalSent = %d\n", totalSent );
		return 1;
	}
	else
		RcvBuf[BufLen] = 0;           // Append null to end of string.

	// Success.
	return 0;
}

///////////////////////////////////////////////////////
// Main control loop.  Returns loop iteration count.

static int io_control_loop( void )
{
	void*		x;				// Transaction object.
	u8		chan;
    struct	timeval	tStart;
    struct	timeval	tEnd;
	int				tDiff;
	int				tMax = 0;

	// Repeat until keypress ...
	for ( ; ; )
	{
		// Wait for a real-time tick (crude simulation via Sleep used here).
		// Disable this to benchmark the i/o system performance.
//		Sleep( 100 );

		// Compute the next output states -----------------------------------------------------------

		// Bump the relay state images.
		RelayStates++;

		// Bump the dio state images.
		if ( ++DoutStates[0] == 0 )
		if ( ++DoutStates[1] == 0 )
		if ( ++DoutStates[2] == 0 )
		if ( ++DoutStates[3] == 0 )
		if ( ++DoutStates[4] == 0 )
		++DoutStates[5];

		// Bump analog output voltage images.
		for ( chan = 0; chan < MAX_NUM_AOUTS; chan++ )
			Aout[chan] = (double)( chan + 1 );

		// Schedule and execute the gateway I/O --------------------------------------------------------------

		// First do some serial io.
		if ( SerialIo( com_src_a_, com_dest_a_ ) )
			return iters;

		if ( SerialIo( com_src_b_, com_dest_b_ ) )
			return iters;

		// Start a new transaction.
		x = CreateTransaction( mm_handle_ );

		// Schedule all I/O into the transaction.
		sched_io( x );

		// Cache the i/o start time.
		gettimeofday( &tStart, 0 );

		// Execute the scheduled i/o and then release the transaction object.  Exit loop if there was no error.
		if ( io_exec( x ) )
		{
			printf( "Terminating i/o control loop.\n" );
			break;
		}

		// Compute elapsed time for the i/o.
	    gettimeofday( &tEnd, 0 );
		tDiff = tEnd.tv_usec - tStart.tv_usec;
		if ( tDiff <= 0 )
			tDiff += 1000000;

		// Update worst-case i/o time.
		if ( tDiff > tMax )
			tMax = tDiff;

		// Bump the loop count.
		iters++;

		// Exit on keypress.
		if ( kbhit() )
		{
			kbread();	// Flush the keypress.
			break;
		}
	}

	// Report worst-case i/o time.
	printf( "Worst-case I/O time (msec):  %d\n", tMax / 1000 );

	// Return cycle count.
	return iters;
}

/////////////////////////////////////////////////////////
// Initialize all I/O and run control loop "forever."

static void io_control_main( void )
{
	u8		i;
	int		j;
	void*		x;				// Transaction object.
	time_t	StartTime;		// Benchmark start time.
	double	tElapsed;		// Benchmark elapsed time.

	// Analog input types for the 2608 iom.  These can be any supported voltage or thermocouple types.
	// Voltage types return volts, temperature types return degrees C.
	const u8 ain_types[] = {
		V_10_TYPE,	// chan 0: 10V range on channels 0-15.
		V_10_TYPE,	// chan 1
		V_10_TYPE,	// chan 2
		V_10_TYPE, 	// chan 3
		V_10_TYPE,	// chan 4
		V_10_TYPE,	// chan 1
		V_10_TYPE,	// chan 6
		V_10_TYPE, 	// chan 7
		V_10_TYPE,	// chan 8
		V_10_TYPE,	// chan 9
		V_10_TYPE,	// chan 10
		V_10_TYPE,	// chan 11
		V_10_TYPE,	// chan 12
		V_10_TYPE,	// chan 13
		V_10_TYPE,	// chan 14
		V_10_TYPE	// chan 15
	};

	// Initialize serial comports.
	SerialInit( com_src_a_, com_dest_a_, com_baud_a_ );
	SerialInit( com_src_b_, com_dest_b_, com_baud_b_ );

	// INITIALIZE IOM's ================================================================================

	// Start a new transaction.
	x = CreateTransaction( mm_handle_ );

	// Schedule the I/O actions into the transaction.
	// For each iom port on the MM ...
	for ( i = 0; i < 16; i++ )
	{
		// Schedule some initialization actions based on iom type.
		switch ( IomType[i] )
		{
		case 2608:

			S26_Sched2608_ReadEeprom( x, i, 0, &nAouts2608[i] );	// Get the dac channel count.
			S26_Sched2608_SetLineFreq( x, i, LINEFREQ_60HZ );		// 60 Hz line frequency (default).
			S26_Sched2608_SetAinTypes( x, i, ain_types );			// Declare the analog input types.

			break;

		case 2620:

			// Set gate period to 1 second, timestamp resolution to 1 millisecond.
			S26_Sched2620_SetCommonControl( x, i, 1000, 3 );

			// Set up the pwm generator on s2620_channel_pwm_.
			S26_Sched2620_SetPreload( x, i, s2620_channel_pwm_, 1, 99 );		// Preload 1: on time.
			S26_Sched2620_SetPreload( x, i, s2620_channel_pwm_, 0, 4899 );		// Preload 0: off time.
			S26_Sched2620_SetModePwmGen( x, i, s2620_channel_pwm_, 0 );		// Configure as pwm generator.

			// Set up the frequency counter on s2620_channel_freq_.
			S26_Sched2620_SetModeFreqMeas( x, i, s2620_channel_freq_, 1 );		// Configure channel 1: freq counter, internal gate.

			// Set up for pulse width measurement on CHAN_PULSE.
			S26_Sched2620_SetModePulseMeas( x, i, s2620_channel_width_, 0 );		// active high input.

			// Set up encoder interface on s2620_channel_encoder_.
			S26_Sched2620_SetModeEncoder( x, i, s2620_channel_encoder_, 0, 0, 3 );

			break;
		}
	}

	// Execute the scheduled i/o and release the transaction object.
	if ( io_exec( x ) )
		return;

	// MAIN CONTROL LOOP ====================================================================================

	printf( "Running main control loop\nHit any key to terminate\n" );

	// Start benchmark timer.
	StartTime = time( NULL );

	// Run control loop until terminated or error.
	iters = io_control_loop();

	// Stop benchmark timer.
	tElapsed = difftime( time( NULL ), StartTime );

	// Show the final system state ============================================================================

	// For each iom port ...
	for ( i = 0; i < 16; i++ )
	{
		switch ( IomType[i] )
		{
		case 2608:
			// Print the analog inputs.
			printf( "AIN states:\n" );
			for ( j = 0; j < 16; j++ )
				printf( " %8f\n", Ain[j] );
			break;

		case 2610:
			// Print the dio input states.
			printf( "DIN states:" );
			for ( j = 0; j < 6; j++ )
				printf( " %2.2hhX", DinStates[j] );
			printf( "\n" );
			break;

		case 2620:
			// Print the dio input states.
			printf( "Counter states:" );
			for ( j = 0; j < 4; j++ )
				printf( "%d:%8.8lX ", j, Counts[j] );
			printf( "\n" );
			break;

		case 2650:
			break;
			
		case 2652:
			break;
		}
	}

	// Report benchmark results.
	printf( "Control loop cycles:    %d\n", iters );
	printf( "Elapsed time (seconds): %lu\n", (u32)tElapsed );
	printf( "Average I/O cycle time (msec):  %.2f\n", tElapsed / (double)iters * 1000.0 );
}

////////////////////////////////////////////////////////
// Start a new transaction.
// Returns non-zero transaction handle if successful.

void* CreateTransaction( HBD hbd )
{
	// Create a new transaction.
	void* x = S26_SchedOpen( hbd, retries_gateway_ );

	// Report error if transaction couldn't be created.
	if ( x == 0 )
		printf( "Error: S26_SchedOpen() failed to allocate a transaction object.\n" );

	return x;
}

////////////////////////////////
// Execute all scheduled i/o.
// Returns zero if successful.

int io_exec( void* x )
{
	GWERR	err;

	// Execute the scheduled i/o.  Report error if one was detected.
	if ( ( err = S26_SchedExecute( x, timeout_gateway_ms_, IomStatus ) ) != 0 )
		ShowErrorInfo( err, IomStatus );

	return (int)err;
}

////////////////////////////////////////////////
// Keyboard functions.

static struct termios	initial_settings;
static struct termios	new_settings;
static int				peek_character = -1;

static void kbopen( void )
{
	tcgetattr( 0, &initial_settings );
	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_lflag &= ~ISIG;
	new_settings.c_cc[VMIN] = 1;
	new_settings.c_cc[VTIME] = 0;
	tcsetattr( 0, TCSANOW, &new_settings );
}
 
static void kbclose( void )
{
	tcsetattr(0, TCSANOW, &initial_settings);
}
 
static int kbhit( void )
{
	unsigned char ch;
	int nread;

	if ( peek_character != -1 )
		return 1;

	new_settings.c_cc[VMIN] = 0;
	tcsetattr( 0, TCSANOW, &new_settings );

	nread = read( 0, &ch, 1 );

	new_settings.c_cc[VMIN] = 1;
	tcsetattr( 0, TCSANOW, &new_settings );

	if ( nread == 1 )
	{
		peek_character = ch;
		return 1;
	}
	
	return 0;
}
 
static int kbread( void )
{
	char ch;

	if ( peek_character != -1 )
	{
		ch = peek_character;
		peek_character = -1;
		return ch;
	}

	read( 0, &ch, 1 );

	return ch;
}
