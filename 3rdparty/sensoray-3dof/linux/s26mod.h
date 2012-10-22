/////////////////////////////////////////////////////////////////////////////
// Module    : s26mod.h	
// Function  : 2600 declarations shared by mod/core
// Target OS : Linux
// Author    : Jim Lamberson
// Copyright : (C) 2008 Sensoray
/////////////////////////////////////////////////////////////////////////////

#ifndef _INC_S26MOD_H_
#define _INC_S26MOD_H_

#include <stdlib.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>		// Mutex support.

// Constants.
#define FALSE		0
#define TRUE		1

// Types.
typedef u32					S26_HSOCKET;
typedef pthread_mutex_t		*S26_CRITICALSECTION;

#endif	// ifdef _INC_S26MOD_H_
