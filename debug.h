#ifndef	_DEBUG_H
#define	_DEBUG_H

#include <stdint.h>

#ifdef	DEBUG
	#define		DEBUG_GCODE_PROCESS 	0x0001
	#define		DEBUG_LIMSW		0x0002
	#define		DEBUG_HOME		0x0004
	#define		DEBUG_TRAJECT		0x0008
	#define		DEBUG_PRUSS		0x0010
	#define		DEBUG_POSITION		0x0020
	#define		DEBUG_HEATER		0x0040
	#define		DEBUG_PID		0x0080
	#define		DEBUG_PWM		0x0100
	#define		DEBUG_TEMP		0x0200
	#define		DEBUG_ANALOG		0x0400
	#define		DEBUG_VERBOSE		0x0800
	#define		DEBUG_COMM		0x2000
#else
	// by setting these to zero, the compiler should optimise the relevant code out
	#define		DEBUG_GCODE_PROCESS	0
	#define		DEBUG_LIMSW		0
	#define		DEBUG_HOME		0
	#define		DEBUG_TRAJECT		0
	#define		DEBUG_PRUSS		0
	#define		DEBUG_POSITION		0
	#define		DEBUG_HEATER		0
	#define		DEBUG_PID		0
	#define		DEBUG_PWM		0
	#define		DEBUG_TEMP		0
	#define		DEBUG_ANALOG		0
	#define		DEBUG_VERBOSE		0
	#define		DEBUG_COMM		0
#endif

#define			DEBUG_ECHO	(1<<31)

#define DBG( x)		( (x) && ((debug_flags & (x)) == (x)) )

extern volatile uint32_t debug_flags;

#endif	/* _DEBUG_H */
