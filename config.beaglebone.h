/* Notice to developers: this file is intentionally included multiple times! */

#include "config_macros.h"

/** \file
 \brief RAMPS v1.3 Sample Configuration
 http://reprap.org/wiki/Arduino_Mega_Pololu_Shield
*/

#define NEW_DDA_CALCULATIONS
#define USE_STEP_CALC_FRACTION

/*
	CONTENTS

	1. Mechanical/Hardware
	2. Acceleration settings
	3. Pinouts
	4. Temperature sensors
	5. Heaters
	6. Communication options
	7. Miscellaneous
	8. Appendix A - PWMable pins and mappings
*/

/***************************************************************************\
*                                                                           *
* 1. MECHANICAL/HARDWARE                                                    *
*                                                                           *
\***************************************************************************/

/** \def F_CPU
	PRUSS clock rate
*/
#define	F_CPU	200000000L

/** \def HOST
	This is the motherboard, as opposed to the extruder. See extruder/ directory for GEN3 extruder firmware
*/
#define	HOST

/**********  B E G I N   O F   D R I V E  T R A I N   C O N F I G U R A T I O N  *********/
/*
	Values reflecting the gearing of your machine.
		All numbers are fixed point integers, so no more than 3 digits to the right of the decimal point, please :-)
*/


/**********************************************************
 *  Edit the defines in the following section to reflect  *
 *  your hardware.                                        *
 *********************************************************/

/// Filament diameters for a rough estimate of extruder output feed [mm].
#define FILAMENT_DIAM_IN			2.9
#define FILAMENT_DIAM_OUT			0.6

/// Physical motor characteristic: (full) steps per revolution for each axis [steps / rev].
#define MOTOR_S_P_R_X				400
#define MOTOR_S_P_R_Y				400
#define MOTOR_S_P_R_Z				200
#define MOTOR_S_P_R_E				200

/// Maximum obtainable motor speed [rev / sec].
#define MAX_REV_SPEED_X				4.2
#define MAX_REV_SPEED_Y				4.2
#define MAX_REV_SPEED_Z				4.0
#define MAX_REV_SPEED_E				3.0

/// Stepper controller characteristic: Microstep multiplier for each axis [pulses / step].
#define MICROSTEPPING_X				16
#define MICROSTEPPING_Y				16
#define MICROSTEPPING_Z				16
#define MICROSTEPPING_E				16

/// Fraction of the maximum feed that is to be used for low-speed moves [-].
#define SEARCH_FEED_FRACTION_X		0.10
#define SEARCH_FEED_FRACTION_Y		0.10
#define SEARCH_FEED_FRACTION_Z		0.25

/**********************************************************
 *  Select your printer model and extruder used from the  *
 *  ones below. If your model is missing, add it yourself *
 *  or ask some help to do so.                            *
 *********************************************************/

#include "prusa_mech.h"

// Override some dimensions from prusa_mech.h
#undef  AXIS_TRAVEL_X
#define	AXIS_TRAVEL_X				215.0
#undef  AXIS_TRAVEL_Z
#define	AXIS_TRAVEL_Z				80.0

// Override hobbed bolt diameter from wades_extruder.h
#define EXTRUDER_FEED_AXIS_DIAM		8.0

#include "wades_extruder.h"


/***********  E N D   O F   D R I V E  T R A I N   C O N F I G U R A T I O N  ************/


/// this is how many steps to suck back the filament by when we stop. set to zero to disable
#define	E_STARTSTOP_STEPS			0


/**
	Soft axis limits, in mm
	undefine if you don't want to use them
*/

#define	X_MIN			0.0
#define	X_MAX			(X_MIN + AXIS_TRAVEL_X)

#define	Y_MIN			0.0
#define	Y_MAX			(Y_MIN + AXIS_TRAVEL_Y)

#define	Z_MIN			0.0
#define	Z_MAX			(Z_MIN + AXIS_TRAVEL_Z)

/**	\def E_ABSOLUTE
	Some G-Code creators produce relative length commands for the extruder, others absolute ones. G-Code using absolute lengths can be recognized when there are G92 E0 commands from time to time. If you have G92 E0 in your G-Code, define this flag.
*/
// #define E_ABSOLUTE



/***************************************************************************\
*                                                                           *
* 2. ACCELERATION                                                           *
*                                                                           *
* IMPORTANT: choose only one! These algorithms choose when to step, trying  *
*            to use more than one will have undefined and probably          *
*            disastrous results!                                            *
*                                                                           *
\***************************************************************************/

/** \def ACCELERATION_RAMPING
	acceleration and deceleration ramping, this is built into the PRUSS code.
		Each movement starts at (almost) no speed, linearly accelerates to target speed and decelerates just in time to smoothly stop at the target. alternative to ACCELERATION_REPRAP
*/
#define ACCELERATION_RAMPING

/// how fast to accelerate when using ACCELERATION_RAMPING, given in mm/s^2
/// decimal allowed, useful range 1. to 10'000, typical range 10. to 100.
#define ACCELERATION 2400.


/***************************************************************************\
*                                                                           *
* 3. PINOUTS                                                                *
*                                                                           *
\***************************************************************************/

/* TODO: implement optional inversion in PRUSS code */
//#define	X_INVERT_DIR
#define	Y_INVERT_DIR
//#define	Z_INVERT_DIR
//#define	E_INVERT_DIR

#define X_MIN_DETECTOR (DETECTOR_SWITCH + DETECTOR_OPEN)
#define Y_MIN_DETECTOR (DETECTOR_SWITCH + DETECTOR_OPEN)
#define Z_MIN_DETECTOR (DETECTOR_OPTO   + DETECTOR_CLOSE)
#define Z_MAX_DETECTOR (DETECTOR_SWITCH + DETECTOR_OPEN)

#if ARCH == arm
#define	X_MIN_PIN
#define	Y_MIN_PIN
#define	Z_MIN_PIN
#define	Z_MAX_PIN
#endif

#define	X_DIR_PIN
#define	Y_DIR_PIN
#define	Z_DIR_PIN




/***************************************************************************\
*                                                                           *
* 4. TEMPERATURE SENSORS                                                    *
*                                                                           *
\***************************************************************************/

/**
	TEMP_HYSTERESIS: actual temperature must be target +/- hysteresis before target temperature can be achieved.
	Unit is degree Celsius.
*/
#define	TEMP_HYSTERESIS				8	/* 5 */
/**
TEMP_RESIDENCY_TIME: actual temperature must be close to target for this long before target is achieved

temperature is "achieved" for purposes of M109 and friends when actual temperature is within [hysteresis] of target for [residency] seconds
*/
#define	TEMP_RESIDENCY_TIME		20	/* 60 */

/// which temperature sensors are you using? (intercom is the gen3-style separate extruder board)
// #define	TEMP_MAX6675
#define	TEMP_THERMISTOR
// #define	TEMP_AD595
// #define	TEMP_PT100
// #define	TEMP_INTERCOM

/***************************************************************************\
*                                                                           *
* Define your temperature sensors here                                      *
*                                                                           *
* for GEN3 set temp_type to TT_INTERCOM and temp_pin to 0                   *
*                                                                           *
* Types are same as TEMP_ list above- TT_MAX6675, TT_THERMISTOR, TT_AD595,  *
*   TT_PT100, TT_INTERCOM. See list in temp.c.                              *
*                                                                           *
\***************************************************************************/

#ifndef DEFINE_TEMP_SENSOR
	#define DEFINE_TEMP_SENSOR(...)
#endif

/*
 * TODO: 20110810 SJL - Fix this workaround for using the high inputs of a 16 input mux!
 *
 * NOTE: For the highest 8 inputs of a 16 channel mux, the correct 'pin' definition
 *       gives a wrong result because only the bit position in the (byte) port is
 *       defined and the port information isn't used. A workaround for this is to
 *       manually add 8 to the AIO pin setting starting with AIO8_PIN and up.
 *       E.g. AIO13_PIN should be written as (AIO13_PIN + 8).
 */

//                  name    	type          	pin             	additional
//DEFINE_TEMP_SENSOR( extruder,	TT_THERMISTOR,	(AIO13_PIN + 8),	THERMISTOR_EXTRUDER)
#if ARCH == arm
#else
DEFINE_TEMP_SENSOR( extruder,	TT_THERMISTOR,	(AIO14_PIN + 8),	THERMISTOR_EXTRUDER)
#endif
//DEFINE_TEMP_SENSOR( bed,     	TT_THERMISTOR,	(AIO14_PIN + 8),	THERMISTOR_EXTRUDER)



/***************************************************************************\
*                                                                           *
* 5. HEATERS                                                                *
*                                                                           *
\***************************************************************************/

/** \def HEATER_SANITY_CHECK
	check if heater responds to changes in target temperature, disable and spit errors if not
	largely untested, please comment in forum if this works, or doesn't work for you!
*/
// #define	HEATER_SANITY_CHECK

/***************************************************************************\
*                                                                           *
* Define your heaters here                                                  *
*                                                                           *
* If your heater isn't on a PWM-able pin, set heater_pwm to zero and we'll  *
*   use bang-bang output. Note that PID will still be used                  *
*                                                                           *
* See Appendix 8 at the end of this file for PWMable pin mappings           *
*                                                                           *
* If a heater isn't attached to a temperature sensor above, it can still be *
*   controlled by host but otherwise is ignored by firmware                 *
*                                                                           *
* To attach a heater to a temp sensor above, simply use exactly the same    *
*   name - copy+paste is your friend                                        *
*                                                                           *
* Some common names are 'extruder', 'bed', 'fan', 'motor'                   *
*                                                                           *
\***************************************************************************/

//#ifndef DEFINE_HEATER
//	#define DEFINE_HEATER(...)
//#endif

// NOTE: these pins are for RAMPS V1.1 and newer. V1.0 is different
//               name      port   pin    pwm
//DEFINE_HEATER( extruder,	PB4)
//DEFINE_HEATER( bed,     	PH5)
//DEFINE_HEATER( fan,     	PH6)
// DEFINE_HEATER(chamber,	PORTD, PIND7, OCR2A)
// DEFINE_HEATER(motor,		PORTD, PIND6, OCR2B)

/// and now because the c preprocessor isn't as smart as it could be,
/// uncomment the ones you've listed above and comment the rest.
/// NOTE: these are used to enable various capability-specific chunks of code, you do NOT need to create new entries unless you are adding new capabilities elsewhere in the code!
/// so if you list a bed above, uncomment HEATER_BED, but if you list a chamber you do NOT need to create HEATED_CHAMBER
/// I have searched high and low for a way to make the preprocessor do this for us, but so far I have not found a way.

//#define	HEATER_EXTRUDER HEATER_extruder
//#define HEATER_BED HEATER_bed
// #define HEATER_FAN HEATER_fan



/***************************************************************************\
*                                                                           *
* 6. COMMUNICATION OPTIONS                                                  *
*                                                                           *
\***************************************************************************/

/** \def REPRAP_HOST_COMPATIBILITY
	RepRap Host changes it's communications protocol from time to time and intentionally avoids backwards compatibility. Set this to the date the source code of your Host was fetched from RepRap's repository, which is likely also the build date.
	See the discussion on the reprap-dev mailing list from 11 Oct. 2010.

	Undefine it for best human readability, set it to an old date for compatibility with hosts before August 2010
*/
// #define REPRAP_HOST_COMPATIBILITY 19750101
#define REPRAP_HOST_COMPATIBILITY 20100806
// #define REPRAP_HOST_COMPATIBILITY <date of next RepRap Host compatibility break>

/**
	Baud rate for the connection to the host. Usually 115200, other common values are 19200, 38400 or 57600.
*/
#define	BAUD	115200

/** \def XONXOFF
	Xon/Xoff flow control.
		Redundant when using RepRap Host for sending GCode, but mandatory when sending GCode files with a plain terminal emulator, like GtkTerm (Linux), CoolTerm (Mac) or HyperTerminal (Windows).
		Can also be set in Makefile
*/
//#define	XONXOFF



/***************************************************************************\
*                                                                           *
* 7. MISCELLANEOUS OPTIONS                                                  *
*                                                                           *
\***************************************************************************/

/** \def DEBUG
	DEBUG
		enables /heaps/ of extra output, and some extra M-codes.
		WARNING: this WILL break most host-side talkers that expect particular responses from firmware such as reprap host and replicatorG
		use with serial terminal or other suitable talker only.
*/
#define	DEBUG

#ifdef DEBUG
// initial setting for debug_flags after start
# define DEBUG_INIT (DEBUG_TRAJECT|DEBUG_GCODE_PROCESS|DEBUG_PRUSS)
#endif

/** \def BANG_BANG
BANG_BANG
drops PID loop from heater control, reduces code size significantly (1300 bytes!)
may allow DEBUG on '168
*/
// #define	BANG_BANG
/** \def BANG_BANG_ON
BANG_BANG_ON
PWM value for 'on'
*/
// #define	BANG_BANG_ON	200
/** \def BANG_BANG_OFF
BANG_BANG_OFF
PWM value for 'off'
*/
// #define	BANG_BANG_OFF	45

/**
	move buffer size, in number of moves
		note that each move takes a fair chunk of ram (69 bytes as of this writing) so don't make the buffer too big - a bigger serial readbuffer may help more than increasing this unless your gcodes are more than 70 characters long on average.
		however, a larger movebuffer will probably help with lots of short consecutive moves, as each move takes a bunch of math (hence time) to set up so a longer buffer allows more of the math to be done during preceding longer moves
*/
#define	MOVEBUFFER_SIZE	8

/** \def USE_WATCHDOG
	Teacup implements a watchdog, which has to be reset every 250ms or it will reboot the controller. As rebooting (and letting the GCode sending application trying to continue the build with a then different Home point) is probably even worse than just hanging, and there is no better restore code in place, this is disabled for now.
*/
// #define USE_WATCHDOG

/**
	analog subsystem stuff
	REFERENCE - which analog reference to use. see analog.h for choices
*/
#define	REFERENCE			REFERENCE_AVCC

/** \def STEP_INTERRUPT_INTERRUPTIBLE
	this option makes the step interrupt interruptible (nested).
	this should help immensely with dropped serial characters, but may also make debugging infuriating due to the complexities arising from nested interrupts
	\note disable this option if you're using a '168 or for some reason your ram usage is above 90%. This option hugely increases likelihood of stack smashing.
*/
#define		STEP_INTERRUPT_INTERRUPTIBLE	1

/**
	temperature history count. This is how many temperature readings to keep in order to calculate derivative in PID loop
	higher values make PID derivative term more stable at the expense of reaction time
*/
#define	TH_COUNT					8

/// this is the scaling of internally stored PID values. 1024L is a good value
#define	PID_SCALE						1024L

