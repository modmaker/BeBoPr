/** \file
	\brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include	<string.h>
#include	<stdlib.h>
#include	<math.h>
#include	<stdio.h>

#include	"pruss.h"
#include	"dda.h"
#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"gcode_parse.h"
#include	"dda_queue.h"
#include	"debug.h"
#include	"sersendf.h"
#include	"pinio.h"
#include	"config.h"
#include	"memory_barrier.h"

#ifdef	DC_EXTRUDER
	#include	"heater.h"
#endif

#include	"dda_util.h"

/// step timeout
volatile uint8_t	steptimeout = 0;

/*
	position tracking
*/

/// \var current_steps
/// \brief target position of last move in queue
static TARGET current_steps __attribute__ ((__section__ (".bss")));

/// \var move_state
/// \brief numbers for tracking the current state of movement
static MOVE_STATE move_state __attribute__ ((__section__ (".bss")));

static uint32_t accel_const;

/*! Inititalise DDA movement structures
*/
void dda_init(void) {
	if (DEBUG_DDA) {
		debug_flags |= DEBUG_DDA;
	}
	// set up default feedrate
	current_steps.F = next_target.target.F = SEARCH_FEEDRATE_Z;

	#ifdef ACCELERATION_RAMPING
		move_state.n = 1;
#ifdef NEW_DDA_CALCULATIONS
		move_state.c = 0;
#if ARCH == arm
		//TODO: fix calculation
		accel_const = (16UL * (uint32_t) 200000000UL) / (10UL * int_sqrt( (uint32_t)(10UL * 4096UL * ACCELERATION)));

#else
		accel_const = (16UL * (uint32_t) F_CPU) / (10UL * int_sqrt( (uint32_t)(10UL * 4096UL * ACCELERATION)));
#endif
		if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
#if ARCH == arm
			sersendf_P(PSTR("\n{DDA_INIT: [ac:%u]\n"), accel_const);
#else
			sersendf_P(PSTR("\n{DDA_INIT: [ac:%ld]\n"), accel_const);
#endif
		}
#else
		// Calculate the initial step period, corrected by a factor 1/sqrt(2)
		// to overcome the error in the first step. (See Austin)
		move_state.c = ((uint32_t)((double)F_CPU / sqrt((double)(STEPS_PER_MM_X * ACCELERATION)))) << 8;
		if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
			sersendf_P(PSTR("\n{DDA_INIT: [c:%ld]\n"), move_state.c >> 8);
		}
#endif
	# ifdef USE_STEP_CALC_FRACTION
		move_state.frac = 0;
	# endif
	#endif
}

/*! CREATE a dda given current_position and a target, save to passed location so we can write directly into the queue
	\param *dda pointer to a dda_queue entry to overwrite
	\param *target the target position of this move

	\ref current_steps the beginning position of this move

	This function does a /lot/ of math. It works out directions for each axis, distance travelled, the time between the first and second step

	It also pre-fills any data that the selected accleration algorithm needs, and can be pre-computed for the whole move.

	This algorithm is probably the main limiting factor to print speed in terms of firmware limitations
*/
void dda_create(DDA *dda, TARGET *target_pos) {

#if ARCH == arm
	if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
		fprintf( stderr,  "\n{DDA_CREATE: [");
		fprintf( stderr, " pos(%d,%d,%d,%d,%u) ]\n",
			 target_pos->X, target_pos->Y, target_pos->Z, target_pos->E, target_pos->F);
	}


#else

	TARGET step_target;
	TARGET* target = &step_target;
#ifndef NEW_DDA_CALCULATIONS
	uint32_t	distance, c_limit, c_limit_calc;
#else
	uint32_t	distance = 0, c_limit;
#endif

	// initialise DDA to a known state
	dda->allflags = 0;

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
		fprintf( stderr, "\n{DDA_CREATE: [");
	}

	// TODO: Change division into multiply ???
	// Convert target_pos (nanometer resolution, range +/- 2147 mm)
	// into a target stepper position for each axis.
	target->X = target_pos->X / NM_PER_STEP_X;
	target->Y = target_pos->Y / NM_PER_STEP_Y;
	target->Z = target_pos->Z / NM_PER_STEP_Z;
	target->E = target_pos->E / NM_PER_STEP_E;
	target->F = target_pos->F;

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
#if ARCH == arm
		sersendf_P( PSTR( "pos(%d,%d,%d,%d,%u)->target(%d,%d,%d,%d,%u)"),
#else
		sersendf_P( PSTR( "pos(%ld,%ld,%ld,%ld,%ld)->target(%ld,%ld,%ld,%ld,%ld)"),
#endif
			target_pos->X, target_pos->Y, target_pos->Z, target_pos->E, target_pos->F,
			target->X, target->Y, target->Z, target->E, target->F );
	}

	// we end at the passed target
	memcpy( &(dda->endpoint), target, sizeof( TARGET));

	dda->x_delta = labs(target->X - current_steps.X);
	dda->y_delta = labs(target->Y - current_steps.Y);
	dda->z_delta = labs(target->Z - current_steps.Z);
	dda->e_delta = labs(target->E - current_steps.E);

	dda->x_direction = (target->X >= current_steps.X)?1:0;
	dda->y_direction = (target->Y >= current_steps.Y)?1:0;
	dda->z_direction = (target->Z >= current_steps.Z)?1:0;
	dda->e_direction = (target->E >= current_steps.E)?1:0;

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
#if ARCH == arm
		sersendf_P(PSTR("%c%u,%c%u,%c%u,%c%u] ["),
#else
		sersendf_P(PSTR("%c%ld,%c%ld,%c%ld,%c%ld] ["),
#endif
			(dda->x_direction)? '+' : '-', dda->x_delta,
			(dda->y_direction)? '+' : '-', dda->y_delta,
			(dda->z_direction)? '+' : '-', dda->z_delta,
			(dda->e_direction)? '+' : '-', dda->e_delta );

	// Determine the largest stepcount from all the axes.
	dda->total_steps = dda->x_delta;
	if (dda->y_delta > dda->total_steps) {
		dda->total_steps = dda->y_delta;
	}
	if (dda->z_delta > dda->total_steps) {
		dda->total_steps = dda->z_delta;
	}
	if (dda->e_delta > dda->total_steps) {
		dda->total_steps = dda->e_delta;
	}
	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
#if ARCH == arm
		sersendf_P(PSTR("ts:%u"), dda->total_steps);
#else
		sersendf_P(PSTR("ts:%lu"), dda->total_steps);
#endif

	if (dda->total_steps == 0) {
		dda->nullmove = 1;
	}
	else {
		// get steppers ready to go
		steptimeout = 0;
		power_on();
		x_enable();
		y_enable();
		// Z is enabled in dda_start()
		e_enable();

		// since it's unusual to combine X, Y and Z changes in a single move on reprap,
		// check if we can use simpler approximations before trying the full 3d approximation.
		if (dda->z_delta == 0) {
			if (dda->x_delta == 0) {
				distance = STEPS_TO_UM( Y, dda->y_delta);
			} else if (dda->y_delta == 0) {
				distance = STEPS_TO_UM( X, dda->x_delta);
			} else {
				distance = approx_distance_2d( STEPS_TO_UM( X, dda->x_delta), STEPS_TO_UM( Y, dda->y_delta));
			}
		} else if (dda->x_delta == 0 && dda->y_delta == 0) {
			distance = STEPS_TO_UM( Z, dda->z_delta);
		} else {
			distance = approx_distance_3d( STEPS_TO_UM( X, dda->x_delta), STEPS_TO_UM( Y, dda->y_delta), STEPS_TO_UM( Z, dda->z_delta));
		}
		// Handle E feed if specified. Note that E is not part of our coordinate space,
		// so it's not of influence on speed and distance travelled!
		uint32_t e_feed = STEPS_TO_UM( E, dda->e_delta);
		if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
#if ARCH == arm
			sersendf_P(PSTR(",ef:%u,ds:%u"), e_feed, distance);
#else
			sersendf_P(PSTR(",ef:%lu,ds:%lu"), e_feed, distance);
#endif
		}

		#ifdef	ACCELERATION_TEMPORAL
			// bracket part of this equation in an attempt to avoid overflow: 60 * 16MHz * 5mm is >32 bits
			uint32_t move_duration = distance * (60 * F_CPU / current_steps.F);
		#else
			// pre-calculate move speed in millimeter microseconds per step minute for less math in interrupt context
			// mm (distance) * 60000000 us/min / step (total_steps) = mm.us per step.min
			//   note: um (distance) * 60000 == mm * 60000000
			// so in the interrupt we must simply calculate
			// mm.us per step.min / mm per min (F) = us per step

			// break this calculation up a bit and lose some precision because 300,000um * 60000 is too big for a uint32
			// calculate this with a uint64 if you need the precision, but it'll take longer so routines with lots of short moves may suffer
			// 2^32/6000 is about 715mm which should be plenty

			// changed * 10 to * (F_CPU / 100000) so we can work in cpu_ticks rather than microseconds.
			// timer.c setTimer() routine altered for same reason

			// changed distance * 6000 .. * F_CPU / 100000 to
			//         distance * 2400 .. * F_CPU / 40000 so we can move a distance of up to 1800mm without overflowing

#ifndef NEW_DDA_CALCULATIONS
			uint32_t move_duration = ((distance * 2400) / dda->total_steps) * (F_CPU / 40000);
#else
			// 20110819 modmaker - Reverse engineering calculation:
			// 
			// move_duration [IOclocks/step * mm/min]  (<timer ticks> * <feed>)
			//		:= distance [um] * 60000 [ms/min] / dda->total_steps [steps] * F_CPU [IOclocks/us]
			//		== distance [mm] * 60000 [s/min] / dda->total_steps [steps] * F_CPU [IOclocks/s]
			//		== distance [mm/min] * 60000 [-] / dda->total_steps [-] * F_CPU [IOclocks/step]
			//
			// 20110820 modmaker - Eliminated division by total_steps, using a fixed divisor to
			//                     prevent overflow. When using move_duration later on, the division
			//                     by TIME_SCALING is undone.
			//
			// Overflow calculation:
			//   move_duration = (distance * 60 * 1000) * (F_CPU / 1000000)
			//   For a 16 MHz CPU this gives: distance * 960000, which overflows if
			//   distance > 12 bits, i.e. moves of more than 4 mm.
			//   Scaling this down by a factor 1000 (10 bits) gives fair precision
			//   (10 bits / 3 digits) and no overflow with current hardware configurations.
			//
			// For maximum precision, set TIME_SCALING to a divisor of F_CPU.
			#define TIME_SCALING	(F_CPU / 1000)
			// The compiler won't optimize this correctly, so do it manually:
			//    move_duration = (uint32_t) (distance [um] * 60 [s/min] * 1000 [um/mm] * (F_CPU [ioclocks/s] / 1000000.0 [us/s] ) / TIME_SCALING )
			//    move_duration = (uint32_t) (distance [mm/min] * 60 * F_CPU [ioclocks] / (1000 * TIME_SCALING) )
			uint32_t move_duration = distance * 60;		// [mm/min * IOclocks]
#endif
		#endif

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
#if ARCH == arm
			sersendf_P(PSTR(",md:%u"), move_duration);
#else
			sersendf_P(PSTR(",md:%lu"), move_duration);
#endif

		// similarly, find out how fast we can run our axes.
		// do this for each axis individually, as the combined speed of two or more axes can be higher than the capabilities of a single one.

#ifndef NEW_DDA_CALCULATIONS

		// 20110819 modmaker - Reverse engineering calculation:
		// 
		// dda->x_delta [steps] * [um/step] * [ms/min] / dda->total_steps [steps] * [IOclocks/us] / [mm/min]
		// ==> [um] * [ms/min] / [us] * [IOclocks/step] * [min/mm]
		// ==> [m] * [s/min] / [s] * [IOclocks/step] * [min/m]
		// ==> [m/min] * [IOclocks/step] * [min/m]
		// => c_limit_calc [IOclocks/step]
		// Note: UM_PER_STEP is not very accurate, so is the result c_limit_calc!
		//
		// 20111016 modmaker - quick hack for NM instead of UM per step. TODO: clean up.
		c_limit = 0;
		// check X axis
		c_limit_calc = ( (dda->x_delta * (NM_PER_STEP_X * 24UL)) / (10L * dda->total_steps) * (F_CPU / 40000) / MAXIMUM_FEEDRATE_X) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check Y axis
		c_limit_calc = ( (dda->y_delta * (NM_PER_STEP_Y * 24UL)) / (10L * dda->total_steps) * (F_CPU / 40000) / MAXIMUM_FEEDRATE_Y) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check Z axis
		c_limit_calc = ( (dda->z_delta * (NM_PER_STEP_Z * 24UL)) / (10L * dda->total_steps) * (F_CPU / 40000) / MAXIMUM_FEEDRATE_Z) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check E axis
		c_limit_calc = ( (dda->e_delta * (NM_PER_STEP_E * 24UL)) / (10L * dda->total_steps) * (F_CPU / 40000) / MAXIMUM_FEEDRATE_E) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		c_limit <<= 8;
#else

		// 20110820 modmaker - Revised calculation
		//                     Use MIN_CLOCKS_PER_STEP_ for more accurate calculations
		//                     MIN_CLOCKS_PER_STEP_ uses 12 bits (more on slow systems?) that leaves 20 bits for delta before overflowing.
		//

		// Calculate the duration of the complete move as run at the specified speed.
		// Adjust that duration for any one of the axes running above it's rated speed.
		// This will scale all axes simultanously, resulting in the same move at reduced feed.

#ifndef ACCELERATION_REPRAP
		// Start with (an estimate of) the duration of the vectored move at the targeted feed.
		// Scale back to IOclock ticks after the division by the feed.
		uint64_t limiting_total_clock_ticks = TIME_SCALING * (move_duration / target->F);		// [IOclocks]
		uint64_t min_total_clock_ticks;
#else
		// Less optimized, allows use with ACCELERATION_REPRAP
		// In this case move_duration is not used in the calculation of c_limit.
		uint64_t limiting_total_clock_ticks = 0;
		uint64_t min_total_clock_ticks;
#endif
		// For each axis, determine the number of IOclocks it would take to run that axis at it's
		// maximum speed. The maximum of these values (determined by the slowest axis) is used to
		// limit the (specified) feed.
		// Take into account that each axis runs at a part of the speed of the vectored move.
		//
		// 20110820 modmaker - overflow calculation:
		//    MIN_CLOCKS_PER_STEP_ is in range [1000..30000] -> 10-15 bits
		//    _delta is in range [1..1000000] -> 20 bits
		//   This would not fit into 32 bits, _but_ there is a reversed dependency between
		//   the two numbers. Look at it this way: 32 bits @ 16 MHz overflows after more than 268 s.
		//   Is there a move that would run for more than 268 s at maximum speed on any axis?
		//   For the slowest axis running at max. 75 mm/min that still gives more than 320 mm travel.
		//   TODO: sanity check in case F_CPU increases, max speed slowest axis decreases. or axis
		//         length increases!
		// conclusion: For now, no overflow will occur

		// 20111016 modmaker - MIN_CLOCKS_PER_STEP_x requires up to 14 bits (z-axis).
		//
		min_total_clock_ticks			= (uint64_t) dda->x_delta * (uint64_t)MIN_CLOCKS_PER_STEP_X;
		if (min_total_clock_ticks > limiting_total_clock_ticks) {
			limiting_total_clock_ticks	= min_total_clock_ticks;
		}
		min_total_clock_ticks			= (uint64_t) dda->y_delta * (uint64_t)MIN_CLOCKS_PER_STEP_Y;
		if (min_total_clock_ticks > limiting_total_clock_ticks) {
			limiting_total_clock_ticks	= min_total_clock_ticks;
		}
		min_total_clock_ticks			= (uint64_t) dda->z_delta * (uint64_t)MIN_CLOCKS_PER_STEP_Z;
		if (min_total_clock_ticks > limiting_total_clock_ticks) {
			limiting_total_clock_ticks	= min_total_clock_ticks;
		}
		min_total_clock_ticks			= (uint64_t) dda->e_delta * (uint64_t)MIN_CLOCKS_PER_STEP_E;
		if (min_total_clock_ticks > limiting_total_clock_ticks) {
			limiting_total_clock_ticks	= min_total_clock_ticks;
		}
		c_limit = (limiting_total_clock_ticks / dda->total_steps);

		// THIS SOLVES THE BIGGEST PROBLEM: LOW SPEED
		// 20110821 modmaker - Finally found the cause of the too low speeds:
		//                     The single timer start value (c0) calculated in dda_init
		// is not sufficient as it's only valid for a single axis move along the x (and y by accident).
		// We need to calculate c0 for each move, and that implies a division and a square root operation.
#if 0
		dda->c0 = F_CPU / int_sqrt( (1000 * ACCELERATION * dda->total_steps) / distance);
#else
		dda->c0 = (accel_const * int_sqrt( (4096UL * distance) / dda->total_steps) + 8) / 16;
#endif
		if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
#if ARCH == arm
			sersendf_P(PSTR(",c0:%u"), dda->c0);
#else
			sersendf_P(PSTR(",c0:%lu"), dda->c0);
#endif
		}
#endif

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
#if ARCH == arm
			sersendf_P(PSTR(",cl:%u"), c_limit);
#else
			sersendf_P(PSTR(",cl:%lu"), c_limit);
#endif

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
#if ARCH == arm
			sersendf_P(PSTR(",f:%u,lt:%llu"), target->F, limiting_total_clock_ticks);
#else
			sersendf_P(PSTR(",f:%lu,lt:%llu"), target->F, limiting_total_clock_ticks);
#endif


		#ifdef ACCELERATION_REPRAP
		// c is initial step time in IOclk ticks
		dda->c = TIME_SCALING * (move_duration / current_steps.F);
		if (dda->c < c_limit)
			dda->c = c_limit;
		dda->end_c = TIME_SCALING * (move_duration / target->F);
		if (dda->end_c < c_limit)
			dda->end_c = c_limit;

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
#if ARCH == arm
			sersendf_P(PSTR(",md:%u,c:%u"), move_duration, dda->c);
#else
			sersendf_P(PSTR(",md:%lu,c:%lu"), move_duration, dda->c);
#endif

		if (dda->c != dda->end_c) {
			uint32_t stF = current_steps.F / 4;
			uint32_t enF = target->F / 4;
			// now some constant acceleration stuff, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
			uint32_t ssq = (stF * stF);
			uint32_t esq = (enF * enF);
			int32_t dsq = (int32_t) (esq - ssq) / 4;

			uint8_t msb_ssq = msbloc(ssq);
			uint8_t msb_tot = msbloc(dda->total_steps);

			// the raw equation WILL overflow at high step rates, but 64 bit math routines take waay too much space
			// at 65536 mm/min (1092mm/s), ssq/esq overflows, and dsq is also close to overflowing if esq/ssq is small
			// but if ssq-esq is small, ssq/dsq is only a few bits
			// we'll have to do it a few different ways depending on the msb locations of each
			if ((msb_tot + msb_ssq) <= 30) {
				// we have room to do all the multiplies first
				if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
					serial_writechar('A');
				dda->n = ((int32_t) (dda->total_steps * ssq) / dsq) + 1;
			}
			else if (msb_tot >= msb_ssq) {
				// total steps has more precision
				if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
					serial_writechar('B');
				dda->n = (((int32_t) dda->total_steps / dsq) * (int32_t) ssq) + 1;
			}
			else {
				// otherwise
				if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
					serial_writechar('C');
				dda->n = (((int32_t) ssq / dsq) * (int32_t) dda->total_steps) + 1;
			}

			if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
				sersendf_P(PSTR("\n{DDA:CA end_c:%lu, n:%ld, md:%lu, ssq:%lu, esq:%lu, dsq:%lu, msbssq:%u, msbtot:%u}\n"), dda->end_c >> 8, dda->n, move_duration, ssq, esq, dsq, msb_ssq, msb_tot);

			dda->accel = 1;
		}
		else
			dda->accel = 0;
		#elif defined ACCELERATION_RAMPING
// remove this when people have swallowed the new config item
#ifdef ACCELERATION_STEEPNESS
#error ACCELERATION_STEEPNESS is gone, review your config.h and use ACCELERATION
#endif
			// yes, this assumes always the x axis as the critical one regarding acceleration. If we want to implement per-axis acceleration, things get tricky ...

			// 20110819 modmaker - Reverse engineering calculation:
			// 
			// (move_duration [IOclocks/step] * [mm/min] / target->F [mm/min]) << 8
			// ==> dda->x_delta [um] * [ms/min] / [us] * [IOclocks/step] * [min/mm]
			// ==> [m] * [s/min] / [s] * [IOclocks/step] * [min/m]
			// ==> [m/min] * [IOclocks/step] * [min/m]
			// => dda->c_min [IOclocks/step]

			// Set minimum counter value (in 24.8 fixed point format)
#ifndef NEW_DDA_CALCULATIONS
			dda->c_min = move_duration / target->F;
			if (dda->c_min < c_limit)
				dda->c_min = c_limit;
#else
			dda->c_min = c_limit;
#endif
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
#if ARCH == arm
				sersendf_P(PSTR(",c-:%u\nramp:"), dda->c_min);
#else
				sersendf_P(PSTR(",c-:%lu\nramp:"), dda->c_min);
#endif
			}
			// 20110819 modmaker - Calculation of the length of the ramps.
			// 
			// The profile is always symetrical (i.e. ramp up and ramp down have
			// the same slope, defined by ACCELERATION).
			//
			// This is a very tricky calculation to do in 32 bits as all precision is
			// needed to get the correct number of steps. If bits are lost and the
			// number is small, the reached feed will be too low.
			// 
			// Do this calculation in several steps so it's easy to detect overflow
			// when debug is on.
#if 0
			uint32_t x = (F_CPU / c_limit);						// (24 - 11..12) -> 12..13 bits
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
				sersendf_P(PSTR(",(x:%lu"), x);
			}
			x *= x;												// + (24 - 11..12) -> 24..26 bits
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
				sersendf_P(PSTR("->%lu"), x);
			}
			x >>= 12;	/* scale down but keep precision! */	// - 12 -> 12..14 bits
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
				sersendf_P(PSTR("->%lu"), x);
			}
			x *= distance;										// + 5..18 -> 17..32 bits
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
				sersendf_P(PSTR("->%lu"), x);
			}
			// total_steps has a fixed relation to distance (um/step) !
			x /= (((uint32_t)(2000 * ACCELERATION) >> 6) * dda->total_steps) >> 6; //    -> 0..14 bits
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
				sersendf_P(PSTR("->%lu"), x);
			}
#else
			// Use 64-bit calculation to prevent overflow or loss of accuracy
			uint64_t t = distance * (((uint64_t)F_CPU * F_CPU) / 2000);	// 24+24-11+5..18	-> 42..56 bits
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
				sersendf_P(PSTR("(t=%llu"), t);
			}
			uint64_t n = dda->total_steps * (uint64_t)ACCELERATION * c_limit * c_limit;		// 12+12+2..12+1..20	-> 27..56 bits
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
				sersendf_P(PSTR(",n=%llu"), n);
			}
			uint32_t x = (uint32_t) (t / n);
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
#if ARCH == arm
				sersendf_P(PSTR( ",t/n=%u)"), x);
#else
				sersendf_P(PSTR(",t/n=%lu)"), x);
#endif
			}
#endif
			dda->rampup_steps = x;
			// If move is too short for full ramp-up and ramp-down, clip ramping.
			if (2 * dda->rampup_steps > dda->total_steps) {
				dda->rampup_steps = dda->total_steps / 2;
			}
			// rampdown_steps is not actually the number of rampdown steps, but the
			// step number at which the rampdown starts!
			dda->rampdown_steps = dda->total_steps - dda->rampup_steps;
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
#if ARCH == arm
				sersendf_P(PSTR("ru:%u,rd:%u"), dda->rampup_steps, dda->rampdown_steps);
#else
				sersendf_P(PSTR("ru:%lu,rd:%lu"), dda->rampup_steps, dda->rampdown_steps);
#endif
			}
		#else
#ifndef NEW_DDA_CALCULATIONS
			dda->c = move_duration / target->F;
			if (dda->c < c_limit)
				dda->c = c_limit;
#else
			dda->c = c_limit;
#endif
		#endif
	}

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("] }\n"));

	// next dda starts where we finish
	memcpy( &current_steps, target, sizeof(TARGET));
	// if E is relative, reset it here
	#ifndef E_ABSOLUTE
		current_steps.E = 0;
	#endif

#endif

	MEMORY_BARRIER();
}

/*! Start a prepared DDA
	\param *dda pointer to entry in dda_queue to start

	This function actually begins the move described by the passed DDA entry.

	We set direction and enable outputs, and set the timer for the first step from the precalculated value.

	We also mark this DDA as running, so other parts of the firmware know that something is happening

	Called both inside and outside of interrupts.
*/
void dda_start(DDA *dda) {
	// called from interrupt context: keep it simple!
	if (dda->nullmove) {
		// just change speed?
//		current_feed = dda->endpoint.F;
		// keep dda->live = 0
	}
	else {
		// get ready to go
		steptimeout = 0;
		if (dda->z_delta)
			z_enable();

		// set direction outputs
#if ARCH == arm
		// TODO: implementation
#else
		x_direction(dda->x_direction);
		y_direction(dda->y_direction);
		z_direction(dda->z_direction);
		e_direction(dda->e_direction);
#endif
		#ifdef	DC_EXTRUDER
		if (dda->e_delta)
			heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
		#endif

		// initialise state variable
		// Initialize the Bresenham 'modulo total_steps' counters
		move_state.x_counter =
			move_state.y_counter =
				move_state.z_counter =
					move_state.e_counter = -(dda->total_steps >> 1);
		// Initialize the 'steps-to-go' down counters
		memcpy(&move_state.x_steps, &dda->x_delta, sizeof(uint32_t) * 4);
		#ifdef ACCELERATION_RAMPING
			move_state.step_no = 0;
		#endif

		# ifdef NEW_DDA_CALCULATIONS
			move_state.c = dda->c0;
#ifdef USE_STEP_CALC_FRACTION
			move_state.frac = 0;
#endif
		# endif

		// ensure this dda starts
		dda->live = 1;

		// set timeout for first step
		#ifdef ACCELERATION_RAMPING
		if (dda->c_min > move_state.c) // can be true when look-ahead removed all deceleration steps
			setTimer( dda->c_min);
		else
			setTimer( move_state.c);
		#else
		setTimer( dda->c);
		#endif
	}
	MEMORY_BARRIER();
}

/*! STEP
	\param *dda the current move

	This is called from our timer interrupt every time a step needs to occur. Keep it as simple as possible!
	We first work out which axes need to step, and generate step pulses for them
	Then we re-enable global interrupts so serial data reception and other important things can occur while we do some math.
	Next, we work out how long until our next step using the selected acceleration algorithm and set the timer.
	Then we decide if this was the last step for this move, and if so mark this dda as dead so next timer interrupt we can start a new one.
	Finally we de-assert any asserted step pins.

	\todo take into account the time that interrupt takes to run


	TODO: from dda_queue.c: the dda dies not directly after its last step, but when the timer fires and there's no steps to do

*/
void dda_step(DDA *dda) {
	uint8_t	did_step = 0;

	// Use Bresenham's algorithm to spread the step pulses for all axes
	// evenly over the total move.
	//
	// For one of these axes (the fastest moving one), dda->total_steps
	// equals to dda->*_delta (see calculation of total_steps in dda_create).
	// This implies that one of these axes will always step and thus did_step
	// always gets set.

	if (move_state.x_steps) {
		move_state.x_counter -= dda->x_delta;
		if (move_state.x_counter < 0) {
#if ARCH == arm
#else
			x_step();
#endif
			did_step = 1;
			move_state.x_steps--;
			move_state.x_counter += dda->total_steps;
		}
	}

	if (move_state.y_steps) {
		move_state.y_counter -= dda->y_delta;
		if (move_state.y_counter < 0) {
#if ARCH == arm
#else
			y_step();
#endif
			did_step = 1;
			move_state.y_steps--;
			move_state.y_counter += dda->total_steps;
		}
	}

	if (move_state.z_steps) {
		move_state.z_counter -= dda->z_delta;
		if (move_state.z_counter < 0) {
#if ARCH == arm
#else
			z_step();
#endif
			did_step = 1;
			move_state.z_steps--;
			move_state.z_counter += dda->total_steps;
		}
	}

	if (move_state.e_steps) {
		move_state.e_counter -= dda->e_delta;
		if (move_state.e_counter < 0) {
#if ARCH == arm
#else
			e_step();
#endif
			did_step = 1;
			move_state.e_steps--;
			move_state.e_counter += dda->total_steps;
		}
	}

	#if STEP_INTERRUPT_INTERRUPTIBLE
		// since we have sent steps to all the motors that will be stepping and the rest of this function isn't so time critical,
		// this interrupt can now be interruptible
		// however we must ensure that we don't step again while computing the below, so disable *this* interrupt but allow others to fire
// 		disableTimerInterrupt();
		sei();
	#endif

	#ifdef ACCELERATION_REPRAP
		// linear acceleration magic, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		if (dda->accel) {
			if ((dda->c > dda->end_c) && (dda->n > 0)) {
				uint32_t new_c = dda->c - (dda->c * 2) / dda->n;
				if (new_c <= dda->c && new_c > dda->end_c) {
					dda->c = new_c;
					dda->n += 4;
				}
				else
					dda->c = dda->end_c;
			}
			else if ((dda->c < dda->end_c) && (dda->n < 0)) {
				uint32_t new_c = dda->c + ((dda->c * 2) / -dda->n);
				if (new_c >= dda->c && new_c < dda->end_c) {
					dda->c = new_c;
					dda->n += 4;
				}
				else
					dda->c = dda->end_c;
			}
			else if (dda->c != dda->end_c) {
				dda->c = dda->end_c;
			}
			// else we are already at target speed
		}
	#endif

	#ifdef ACCELERATION_RAMPING
		// - algorithm courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		// - precalculate ramp lengths instead of counting them, see AVR446 tech note
		// debug ramping algorithm
		//if (move_state.step_no == 0) {
		//	sersendf_P(PSTR("\r\nc %lu  c_min %lu  n %d"), dda->c, dda->c_min, move_state.n);
		//}

#if 1
		uint8_t step_no_lt_rampup_steps = (move_state.step_no < dda->rampup_steps) ? (uint8_t) 1 : (uint8_t) 0;
		uint8_t step_no_gt_rampdown_steps = (move_state.step_no > dda->rampdown_steps) ? (uint8_t) 1 : (uint8_t) 0;
		if (step_no_lt_rampup_steps || step_no_gt_rampdown_steps) {
			// ramping up or down
			if ((step_no_lt_rampup_steps && move_state.n < 0) || (step_no_gt_rampdown_steps && move_state.n > 0)) {
				// wrong ramp direction, fix it
				move_state.n = -((int32_t)2) - move_state.n;
			}
			move_state.n += 4;
			// be careful of signedness!
#ifdef USE_STEP_CALC_FRACTION
			// Implemented calculation with remainder to improve accuracy
			// move_state.n is (4 * accel_count + 1) ?
			// use proper division that also returns the remainder here.
//			int32_t delta	= (2 * (int32_t)move_state.c + move_state.frac) / (int32_t)move_state.n;
//			move_state.frac	= (2 * (int32_t)move_state.c + move_state.frac) - delta * (int32_t)move_state.n;
//			move_state.c 	= (int32_t)move_state.c - delta;
			// delta and frac are always ge 0 !
			uint32_t ccf	= move_state.c + move_state.c + move_state.frac;
			uint32_t delta	= ccf / move_state.n;
			move_state.c	-= delta;
			move_state.frac	= ccf - move_state.n * delta;
#else
//			move_state.c 	= (int32_t)move_state.c - ((int32_t)(move_state.c * 2) / (int32_t)move_state.n);
			move_state.c 	= move_state.c - (move_state.c + move_state.c) / move_state.n;
#endif
		}
#else
		uint8_t recalc_speed;

		recalc_speed = 0;
		if (move_state.step_no < dda->rampup_steps) {
			if (move_state.n < 0) { // wrong ramp direction
				move_state.n = -((int32_t)2) - move_state.n;
			}
			recalc_speed = 1;
		}
		else if (move_state.step_no > dda->rampdown_steps) {
			if (move_state.n > 0) { // wrong ramp direction
				move_state.n = -((int32_t)2) - move_state.n;
			}
			recalc_speed = 1;
		}
		if (recalc_speed) {
			move_state.n += 4;
			// be careful of signedness!
#ifdef USE_STEP_CALC_FRACTION
			// Implemented calculation with remainder to improve accuracy
			// move_state.n is (4 * accel_count + 1) ?
			// use proper division that also returns the remainder here.
			int32_t delta	= (2 * (int32_t)move_state.c + move_state.rest) / (int32_t)move_state.n;
			move_state.rest	= (2 * (int32_t)move_state.c + move_state.rest) - delta * (int32_t)move_state_n;
			move_state.c = (int32_t)move_state.c - delta;
#else
			move_state.c = (int32_t)move_state.c - ((int32_t)(move_state.c * 2) / (int32_t)move_state.n);
#endif
		}

#endif
		move_state.step_no++;

		// debug ramping algorithm
		// for very low speeds like 10 mm/min, only
		//if (move_state.step_no % 10 /* 10, 100, ...*/ == 0)
		//	sersendf_P(PSTR("\r\nc %lu  c_min %lu  n %d"), dda->c, dda->c_min, move_state.n);
	#endif

	// TODO: did_step is obsolete ... Why? Maybe, see explanation above
	if (did_step) {
		// we stepped, reset stepper powerdown timeout
		steptimeout = 0;

		// if we could do anything at all, we're still running
		// otherwise, must have finished
	} else if (move_state.x_steps == 0 && move_state.y_steps == 0 && move_state.z_steps == 0 && move_state.e_steps == 0) {
		// TODO: if did_step is obsolete, this code and the test above is obsolete!
		// 20111017 modmaker - use of serdendf from ISR will hang the system!
		// this code is being executed during normal use
//		if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
//			sersendf_P(PSTR("** HIT-1 **"));
//		}
		dda->live = 0;
		#ifdef	DC_EXTRUDER
			heater_set(DC_EXTRUDER, 0);
		#endif
		// z stepper is only enabled while moving
		z_disable();
	} else {
		// If did_step really is obsolete, we should never arrive here!
		//
		// The unexpected happened, did_step isn't obsolete!???
		if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
			sersendf_P(PSTR("** HIT-2 **"));
		}
	}

	cli();

	#ifdef ACCELERATION_RAMPING
		// we don't hit maximum speed exactly with acceleration calculation, so limit it here
		// the nice thing about _not_ setting dda->c to dda->c_min is, the move stops at the exact same c
		// as it started, so we have to calculate c only once for the time being
		// TODO: set timer only if dda->c has changed
		if (dda->c_min > move_state.c)
			setTimer( dda->c_min);
		else
			setTimer( move_state.c);
	#else
		setTimer( dda->c);
	#endif

	// turn off step outputs, hopefully they've been on long enough by now to register with the drivers
	// if not, too bad. or insert a (very!) small delay here, or fire up a spare timer or something.
	// we also hope that we don't step before the drivers register the low- limit maximum speed if you think this is a problem.
#if ARCH == arm
#else
	unstep();
#endif
}

