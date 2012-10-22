///////////////////////////////////////////////////////////////////////////////////////////////
// Module    : s26modf.h	
// Function  : Functions implememented in s26mod.c and required by s26core.c
// Target OS : Any (operating system independent)
// Author    : Jim Lamberson
// Copyright : (C) 2008 Sensoray
///////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _INC_S26MODF_H_
#define _INC_S26MODF_H_

// Constants
#define S26_INVALID_SOCKET  			((S26_HSOCKET)0)			// Invalid socket handle: socket has not been created.
#define S26_INVALID_CRITICALSECTION		((S26_CRITICALSECTION)0)	// Invalid critical section handle: cs has not been created.

// Prototypes of functions that must be implemented in s26mod.c
int					S26MOD_BindSocket				( S26_HSOCKET hSock, u32 nordClientIPAdrs );
int					S26MOD_CloseNetwork				( void );
void				S26MOD_CloseSocket				( S26_HSOCKET hSock );
S26_HSOCKET			S26MOD_CreateSocket				( void );
void				S26MOD_CriticalSectionBegin		( S26_CRITICALSECTION cs );
S26_CRITICALSECTION	S26MOD_CriticalSectionCreate	( void );
void				S26MOD_CriticalSectionDestroy	( S26_CRITICALSECTION cs );
void				S26MOD_CriticalSectionEnd		( S26_CRITICALSECTION cs );
u32					S26MOD_CurrentTime				( void );
u32					S26MOD_inet_addr				( const char *dot_adrs );
int					S26MOD_IsPacketRcvd				( S26_HSOCKET hSock, u32 msec );
int					S26MOD_OpenNetwork				( u32 MaxNumSockets );
u16					S26MOD_RecvPacket				( S26_HSOCKET hSock, u8 *pBuf, u16 BufLen );
int					S26MOD_SendPacket				( S26_HSOCKET hSock, u32 MMAdrs, u16 MMPort, const u8 *pCmd, u16 CmdLen, u32 msec );

#endif	// ifdef _INC_S26MODF_H_
