#include	"clock.h"

/** \file
	\brief Do stuff periodically
*/

#include	"pinio.h"
#include	"sersendf.h"
#include	"dda_queue.h"
#include	"temp.h"
#include	"timer.h"
#include	"debug.h"
#include	"heater.h"
#include	"serial.h"
#ifdef	TEMP_INTERCOM
	#include	"intercom.h"
#endif
#include	"memory_barrier.h"
#include	"gcode_process.h"


/*!	do stuff every 1/4 second

	called from clock_10ms(), do not call directly
*/
void clock_250ms() {
	#ifndef	NO_AUTO_IDLE
	if (temp_all_zero())	{
		if (steptimeout > (30 * 4)) {
			power_off();
		}
		else {
#if ARCH != arm
			uint8_t save_reg = SREG;
#endif
			cli();
#if ARCH != arm
			CLI_SEI_BUG_MEMORY_BARRIER();
#endif
			steptimeout++;
			MEMORY_BARRIER();
#if ARCH != arm
			SREG = save_reg;
#endif
		}
	}
	#endif

	ifclock(clock_flag_1s) {
	  gcode_trace_move();
#if 0
		if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
			// current position
#if ARCH == arm
			sersendf_P(PSTR("Pos: %d,%d,%d,%d,%u\n"),
				   gcode_home_pos.X, gcode_home_pos.Y, gcode_home_pos.Z,
				   gcode_home_pos.E, gcode_home_pos.F);
#else
			sersendf_P(PSTR("Pos: %ld,%ld,%ld,%ld,%lu\n"),
				   gcode_home_pos.X, gcode_home_pos.Y, gcode_home_pos.Z,
				   gcode_home_pos.E, gcode_home_pos.F);
#endif
			// target position
			{
#if ARCH == arm
			unsigned mb_tail = dda_queue_get_mb_tail();
			sersendf_P(PSTR("Dst: %d,%d,%d,%d,%u\n"),
				   movebuffer[mb_tail].endpoint.X, movebuffer[mb_tail].endpoint.Y,
				   movebuffer[mb_tail].endpoint.Z, movebuffer[mb_tail].endpoint.E,
				   movebuffer[mb_tail].endpoint.F);
#else
			sersendf_P(PSTR("Dst: %ld,%ld,%ld,%ld,%lu\n"),
				   movebuffer[mb_tail].endpoint.X, movebuffer[mb_tail].endpoint.Y,
				   movebuffer[mb_tail].endpoint.Z, movebuffer[mb_tail].endpoint.E,
				   movebuffer[mb_tail].endpoint.F);
#endif
			}

			// Queue
			dda_queue_print();

			// newline
			serial_writechar('\n');
		}
#endif
		// temperature
		/*		if (temp_get_target())
		temp_print();*/
	}
	#ifdef	TEMP_INTERCOM
	start_send();
	#endif
}

/*! do stuff every 10 milliseconds

	call from ifclock(CLOCK_FLAG_10MS) in busy loops
*/
void clock_10ms() {
	// reset watchdog
	wd_reset();

	temp_tick();

	ifclock(clock_flag_250ms) {
		clock_250ms();
	}
}

