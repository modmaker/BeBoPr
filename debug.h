#ifndef	_DEBUG_H
#define	_DEBUG_H

#include <stdint.h>
#include "config.h"

#ifdef	DEBUG
	#define		DEBUG_PID	1
	#define		DEBUG_DDA	2
	#define		DEBUG_POSITION	4
	#define		DEBUG_HOMING	8
	#define		DEBUG_TRAJECT	16
	#define		DEBUG_GCODE_PROCESS	32
	#define		DEBUG_TEMP	64
	#define		DEBUG_ANALOG	128
	#define		DEBUG_HEATER	256
	#define		DEBUG_PWM	512
	#define		DEBUG_LIMSW	1024
#else
	// by setting these to zero, the compiler should optimise the relevant code out
	#define		DEBUG_PID	0
	#define		DEBUG_DDA	0
	#define		DEBUG_POSITION	0
	#define		DEBUG_HOMING	0
	#define		DEBUG_TRAJECT	0
	#define		DEBUG_GCODE_PROCESS	0
	#define		DEBUG_TEMP	0
	#define		DEBUG_ANALOG	0
	#define		DEBUG_HEATER	0
	#define		DEBUG_PWM	0
	#define		DEBUG_LIMSW	0
#endif

#define			DEBUG_ECHO	(1<<31)

extern volatile uint32_t debug_flags;

#endif	/* _DEBUG_H */
