//////////////////////////////////////////////////////////////////////////
// Module    : s26mod.c
// Function  : OS-dependent API library (top level) for 2600 core
// Target OS : Linux
// Author    : Jim Lamberson
// Copyright : (C) 2004 Sensoray
//////////////////////////////////////////////////////////////////////////


#include "s26app.h"
#include "s26mod.h"
#include "s26modf.h"

#include <sys/time.h>


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////  EXPORTED LIBRARY FUNCTIONS ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
// Convert IP dotted address to network byte ordered IP address.
// Imports:
//  dot_adrs : dotted IP address.
// Returns binary IP address in network byte order.

unsigned long S26MOD_inet_addr( const char *dot_adrs )
{
	return inet_addr( dot_adrs );
}

/////////////////////////////////////////////////////////////////////////
// Open the socket interface.
// Imports:
//	MaxNumSockets : number of sockets required for all MMs in the system.
// Returns zero if successful.

int S26MOD_OpenNetwork( u32 MaxNumSockets )
{
	return 0;	// success
}

//////////////////////////////////
// Close the socket interface.

int S26MOD_CloseNetwork()
{
	return 0;	// success
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Bind a socket to a network interface card.  In a multihomed host, specify the IP address of the
// desired network interface card to bind to a specific NIC.  Zero may be specified as the target
// IP address, in which case the socket will be bound to the host's default NIC.
// Imports:
//  hSock          : handle to socket.
//  nordClientIPAdrs : IP address, in network byte order, of client's interface (0 = default).
// Returns True if successful.

int S26MOD_BindSocket( S26_HSOCKET hSock, u32 nordClientIPAdrs )
{
	// Bind socket to the client's IP address.
	struct sockaddr_in	ClientAddr;
	ClientAddr.sin_family		= AF_INET;				// Use IP protocol.
	ClientAddr.sin_addr.s_addr	= nordClientIPAdrs;		// Bind to the NIC that has this IP address.
	ClientAddr.sin_port			= 0;					// Bind to a Windows-assigned, unique IP port.

	// bind() returns 0 if no error occurs.
	return ( bind( hSock, (struct sockaddr *)&ClientAddr, sizeof(ClientAddr) ) == 0 );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Transmit a UDP packet to the specified socket and IP port.  The calling thread is blocked until
// the packet is successfully sent or a timeout occurs.
// Imports:
//   hSock	: handle to open socket.
//   MMAdrs	: target IP address in network byte order.
//   MMPort	: target IP port in network byte order.
//   pCmd	: buffer that contains UDP payload to be transmitted.
//   CmdLen	: size of Cmd buffer.
//   msec	: maximum time to wait for socket to become ready to transmit.
// Returns non-zero value if successful.

int S26MOD_SendPacket( S26_HSOCKET hSock, u32 MMAdrs, u16 MMPort, const u8 *pCmd, u16 CmdLen, u32 msec )
{
	int rtnval = FALSE;

	// First we must wait for the socket to be transmit ready.
	struct sockaddr_in	ServerAddr;			// Server attributes.
	fd_set				EnabledFDSet;		// List of sockets to check.
	struct timeval		TimeValue;			// Maximum time to wait for Transmit Ready.

    TimeValue.tv_sec = 0;
    TimeValue.tv_usec = msec * 1000;

	// Init list of sockets to be tested for transmit readiness.
	FD_ZERO( &EnabledFDSet );
	FD_SET( hSock, &EnabledFDSet );

	// If socket becomes transmit-ready before we time out ...
	if ( select( hSock + 1, NULL, &EnabledFDSet, NULL, &TimeValue ) == 1 )
	{
		// Specify target MM attributes.
		ServerAddr.sin_port			= htons( MMPort );
		ServerAddr.sin_family		= AF_INET;
		ServerAddr.sin_addr.s_addr	= MMAdrs;

		// Attempt to transmit the packet, and if successful, set return value to TRUE.
		rtnval = ( sendto( hSock, (const char *)pCmd, CmdLen, 0, (struct sockaddr *)&ServerAddr, sizeof(ServerAddr) ) == CmdLen );
	}

	// Return TRUE if packet transmit was successful.
	return rtnval;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Receive a UDP datagram from the specified socket and IP port.  It is assumed that a packet has already
// been received; this should be verified by calling 0S_IsPacketRcvd() before calling this function.
// Imports:
//   hSock   : handle to socket.
//   pBuf    : buffer that is to receive the datagram.
//   BufLen  : size of receive buffer.
// Returns byte count of the received datagram.

#define SOCKET_ERROR	-1

u16 S26MOD_RecvPacket( S26_HSOCKET hSock, u8 *pBuf, u16 BufLen )
{
	// Receive datagram into pBuf[] and return datagram byte count.
	int rtnval = recvfrom( hSock, (char *)pBuf, (int)BufLen, 0, 0, 0 );

	// Force byte count to zero if an error was detected, otherwise return datagram's byte count.
	return ( rtnval == SOCKET_ERROR ) ? S26_INVALID_SOCKET : (u16)rtnval;
}

//////////////////////////////////////////////////////////////////
// Determine whether a packet has been received.
// Imports:
//   hSock : handle to socket.
//   msec  : maximum time, in milliseconds, to wait for packet.
// Returns TRUE if received, FALSE if not received.

int S26MOD_IsPacketRcvd( S26_HSOCKET hSock, u32 msec )
{
	// Set up structure that specifies maximum time to wait.
	fd_set	EnabledFDSet;				// List of sockets to check.
	struct timeval	TimeValue;			// Maximum time to wait for Transmit Ready.

    TimeValue.tv_sec = 0;
    TimeValue.tv_usec = msec * 1000;

	// Init list of sockets to be tested for receive readiness.
	FD_ZERO( &EnabledFDSet );
	FD_SET( hSock, &EnabledFDSet );

	// Test socket to discover whether a packet has been received and return appropriate value.
	return ( select( hSock + 1, &EnabledFDSet, NULL, NULL, &TimeValue ) == 1 );
}

////////////////////////////////////////////////////////////////
// Create a socket.
// Returns socket handle if successful, otherwise return 0.

#define INVALID_SOCKET -1

S26_HSOCKET S26MOD_CreateSocket( void )
{
	S26_HSOCKET hSocket = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP );
	return ( hSocket == INVALID_SOCKET ) ? S26_INVALID_SOCKET : hSocket;
}

////////////////////
// Close a socket.
// Imports:
//   hSock : handle to socket.

void S26MOD_CloseSocket( S26_HSOCKET hSock )
{
	close( hSock );
}

///////////////////////////////////////////////////////////////////////////////////
// Get the current time expressed in milliseconds (wraps around after 2^32 msec).

u32 S26MOD_CurrentTime( void )
{
	struct timeval t;
	
	gettimeofday( &t, NULL);
	
	return ( 1000 * t.tv_sec + t.tv_usec / 1000 );
}


////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////  CRITICAL SECTION FUNCTIONS  ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////
// Create critical section object and return its handle.
// Returns zero upon fail.

S26_CRITICALSECTION S26MOD_CriticalSectionCreate( void )
{
	// Allocate storage for a mutex that will serve as a critical section access controller.
	S26_CRITICALSECTION cs = malloc( sizeof( pthread_mutex_t ) );

	// Init the mutex to the default state.
	if ( cs )
		pthread_mutex_init( cs, NULL );

	// Return handle to the mutex.
	return cs;
}

///////////////////////////////////////////////////
// Destroy critical section access control object.

void S26MOD_CriticalSectionDestroy( S26_CRITICALSECTION cs )
{
	pthread_mutex_destroy( cs );
}

///////////////////////////////////////////////
// Begin critical section.

void S26MOD_CriticalSectionBegin( S26_CRITICALSECTION cs )
{
	pthread_mutex_lock( cs );
}

/////////////////////////////////////////
// Leave critical section.

void S26MOD_CriticalSectionEnd( S26_CRITICALSECTION cs )
{
	pthread_mutex_unlock( cs );
}
