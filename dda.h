#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"config.h"

// Used in distance calculation during DDA setup
/// nanometers per step X
#define	NM_PER_STEP_X		(uint16_t)(1000000.0 / STEPS_PER_MM_X)
/// nanometers per step Y
#define	NM_PER_STEP_Y		(uint16_t)(1000000.0 / STEPS_PER_MM_Y)
/// nanometers per step Z
#define	NM_PER_STEP_Z		(uint16_t)(1000000.0 / STEPS_PER_MM_Z)
/// nanometers per step E
#define	NM_PER_STEP_E		(uint16_t)(1000000.0 / STEPS_PER_MM_E)

#ifdef ACCELERATION_REPRAP
	#ifdef ACCELERATION_RAMPING
		#error Cant use ACCELERATION_REPRAP and ACCELERATION_RAMPING together.
	#endif
#endif

/*
	types
*/

// target is simply a point in space/time
typedef struct {
	int32_t						X;
	int32_t						Y;
	int32_t						Z;
	int32_t						E;
	uint32_t					F;
} TARGET;

/**
	\struct MOVE_STATE
	\brief this struct is made for tracking the current state of the movement

	Parts of this struct are initialised only once per reboot, so make sure dda_step() leaves them with a value compatible to begin a new movement at the end of the movement. Other parts are filled in by dda_start().
*/
typedef struct {
	// bresenham counters
	int32_t						x_counter; ///< counter for total_steps vs this axis
	int32_t						y_counter; ///< counter for total_steps vs this axis
	int32_t						z_counter; ///< counter for total_steps vs this axis
	int32_t						e_counter; ///< counter for total_steps vs this axis

	// step counters
	uint32_t					x_steps; ///< number of steps on X axis
	uint32_t					y_steps; ///< number of steps on Y axis
	uint32_t					z_steps; ///< number of steps on Z axis
	uint32_t					e_steps; ///< number of steps on E axis

	#ifdef ACCELERATION_RAMPING
	/// counts actual steps done
	uint32_t					step_no;
	/// time until next step
	uint32_t					c;
	/// tracking variable
	int16_t						n;
	# ifdef USE_STEP_CALC_FRACTION
	/// rest from next step delay calculation
	uint32_t					frac;
	# endif
	#endif
} MOVE_STATE;

/**
	\struct DDA
	\brief this is a digital differential analyser data struct

	This struct holds all the details of an individual multi-axis move, including pre-calculated acceleration data.
	This struct is filled in by dda_create(), called from enqueue(), called mostly from gcode_process() and from a few other places too (eg \file homing.c)
*/
typedef struct {
	/// this is where we should finish
	TARGET						endpoint;

	union {
		struct {
			// status fields
			uint8_t						nullmove			:1; ///< bool: no axes move, maybe we wait for temperatures or change speed
			uint8_t						live					:1; ///< bool: this DDA is running and still has steps to do
			#ifdef ACCELERATION_REPRAP
			uint8_t						accel					:1; ///< bool: speed changes during this move, run accel code
			#endif

			// wait for temperature to stabilise flag
			uint8_t						waitfor_temp	:1; ///< bool: wait for temperatures to reach their set values

			// directions
			uint8_t						x_direction		:1; ///< direction flag for X axis
			uint8_t						y_direction		:1; ///< direction flag for Y axis
			uint8_t						z_direction		:1; ///< direction flag for Z axis
			uint8_t						e_direction		:1; ///< direction flag for E axis
		};
		uint8_t							allflags;	///< used for clearing all flags
	};

	// distances
	uint32_t					x_delta; ///< number of steps on X axis
	uint32_t					y_delta; ///< number of steps on Y axis
	uint32_t					z_delta; ///< number of steps on Z axis
	uint32_t					e_delta; ///< number of steps on E axis

	/// total number of steps: set to \f$\max(\Delta x, \Delta y, \Delta z, \Delta e)\f$
	uint32_t					total_steps;

	uint32_t					c; ///< time until next step, integer, IOclock ticks

	#ifdef ACCELERATION_REPRAP
	uint32_t					end_c; ///< time between 2nd last step and last step
	int32_t						n; ///< precalculated step time offset variable. At every step we calculate \f$c = c - (2 c / n)\f$; \f$n+=4\f$. See http://www.embedded.com/columns/technicalinsights/56800129?printable=true for full description
	#endif
	#ifdef ACCELERATION_RAMPING
	/// number of steps accelerating
	uint32_t					rampup_steps;
	/// number of last step before decelerating
	uint32_t					rampdown_steps;
	/// timer period at maximum speed, integer, IOclock ticks
	uint32_t					c_min;
	# ifdef NEW_DDA_CALCULATIONS
	/// initial timer period, integer, IOclock ticks
	uint32_t					c0;
	# endif
	#endif
} DDA;

/*
	variables
*/

/// steptimeout is set to zero when we step, and increases over time so we can turn the motors off when they've been idle for a while
/// It is also used inside and outside of interrupts, which is why it has been made volatile
extern volatile uint8_t steptimeout;

extern TARGET start_pos;

/*
	methods
*/
// initialize dda structures
void dda_init(void);

// create a DDA
void dda_create(DDA *dda, TARGET *target);

// start a created DDA (called from timer interrupt)
void dda_start(DDA *dda)																						__attribute__ ((hot));

// DDA takes one step (called from timer interrupt)
void dda_step(DDA *dda)																							__attribute__ ((hot));

// update current_position
void update_position(void);

#endif	/* _DDA_H */
