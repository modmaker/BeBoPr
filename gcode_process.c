/** \file
	\brief Work out what to do with received G-Code commands
*/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdint.h>

#include "bebopr.h"
#include "gcode_process.h"
#include "gcode_parse.h"
#include "debug.h"
#include "temp.h"
#include "heater.h"
#include "home.h"
#include "traject.h"
#include "pruss_stepper.h"
#include "heater.h"
#include "mendel.h"
#include "limit_switches.h"
#include "pepper.h"


/// the current tool
static uint8_t tool;
/// the tool to be changed when we get an M6
static uint8_t next_tool;

// 20111017 modmaker - use nanometers instead of steps for position
/// variable that holds the idea of 'current position' for the gcode interpreter.
/// the actual machine position will probably lag!
static TARGET gcode_current_pos;
//  Home Position holds the offset set by G92, it is used to convert the
//  gcode coordinates to machine / PRUSS coordinates.
static TARGET gcode_home_pos;
static double gcode_current_feed;
/*
 * Local copy of channel tags to prevent a lookup with each access.
 */
static channel_tag heater_extruder = NULL;
static channel_tag heater_bed = NULL;
static channel_tag temp_extruder = NULL;
static channel_tag temp_bed = NULL;
static channel_tag pwm_extruder = NULL;
static channel_tag pwm_fan = NULL;
static channel_tag pwm_bed = NULL;

static int extruder_temp_wait = 0;
static int bed_temp_wait = 0;

/*
	private functions
*/

static void wait_for_slow_signals( move5D* move)
{
  if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
    if (move) {
      printf( "\nMOVE[ %lu] - defer move(s) until temperature(s) are stable\n",
	      move->serno);
    } else {
      printf( "\nnon-MOVE - defer processing until temperature(s) are stable\n");
    }
  }
  while ( (extruder_temp_wait && !heater_temp_reached( heater_extruder)) ||
          (bed_temp_wait && !heater_temp_reached( heater_bed)) )
  {
    usleep( 100000);
  }
  extruder_temp_wait = 0;
  bed_temp_wait = 0;
  if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
    printf( "Resume after waiting for temperatures to stabilize\n");
  }
}

/*
 *  Generate unique serial numbers for moves
 */
static inline long unsigned int get_serno( void)
{
  static long unsigned int serno = 0;
  return ++serno;
}

/*
 *  Calculate traject for move to 'target' position and store result in 'move'
 * 
 *  Move from 'gcode_current_pos' to 'target', using 'gcode_home_pos' as offset
 *  with feed from 'target'.
 *  
 *  Calling this routine should have no side effects!!!
 *  (exception: global serial number increment in traject.c)
 */
static void move_calc( unsigned long seqno, const TARGET* target, TARGET* current_pos, move5D* move)
{
  move->serno = seqno;
  if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
    printf( "\nMOVE[ %lu] move_calc() for TARGET=( %d, %d, %d, %d, %u)\n",
            move->serno, target->X, target->Y, target->Z, target->E, target->F);
  }
 /*
  * The traject code requires SI coordinates.
  * The integer TARGET position coordinates are in nm !
  */
  traject5D traject = {
          .s0x = POS2SI( gcode_home_pos.X + current_pos->X),
          .s0y = POS2SI( gcode_home_pos.Y + current_pos->Y),
          .s0z = POS2SI( gcode_home_pos.Z + current_pos->Z),
          .s0e = POS2SI( gcode_home_pos.E + current_pos->E),
          .s1x = POS2SI( gcode_home_pos.X + target->X),
          .s1y = POS2SI( gcode_home_pos.Y + target->Y),
          .s1z = POS2SI( gcode_home_pos.Z + target->Z),
          .s1e = POS2SI( gcode_home_pos.E + target->E),
          .feed = (double)target->F,
  };
 /*
  * Calculate the trajectory details (e.g. velocities)) and store these in 'move'
  */
  traject_calc_all_axes( &traject, move);
}

/*
 *  Execute actual move to new position as specified in 'move'
 * 
 *  Synchronize with slow signals.
 *  
 */
static void move_execute( move5D* move)
{
  if (extruder_temp_wait || bed_temp_wait) {
    if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
      printf( "\nMOVE[ %lu] move_execute() - waiting for slow signals)\n",
              move->serno);
    }
    wait_for_slow_signals( move);
  }
  if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
    printf( "MOVE[ %lu] move_execute() from ( %1.6lf, %1.6lf, %1.6lf, %1.6lf) [m] with feed %1.3lf [mm/min]\n"
            "MOVE ............ over ( %1.6lf, %1.6lf, %1.6lf, %1.6lf) [m]\n"
            "MOVE ........ velocity ( %1.6lf, %1.6lf, %1.6lf, %1.6lf) [m/s]\n",
            move->serno, move->s0x, move->s0y, move->s0z, move->s0e,
            move->feed, move->dx, move->dy, move->dz, move->de,
            move->vx, move->vy, move->vz, move->ve);
  }
  /* actually make the move */
  traject_move_all_axes( move);
}

/*
 *  make a move to new 'target' position, at the end of this move 'target'
 *  should reflect the actual position.
 */
static void enqueue_pos( const TARGET* target)
{
  if (target) {
    move5D move;
    move_calc( get_serno(), target, &gcode_current_pos, &move);
    move_execute( &move);
  }
}


/************************************************************************//**

  \brief Processes command stored in global \ref next_target.
  This is where we work out what to actually do with each command we
    receive. All data has already been scaled to integers in gcode_parse.
    If you want to add support for a new G or M code, this is the place.


*//*************************************************************************/

void dump_position_info( void)
{
  printf(  "current: X=%1.6lf, Y=%1.6lf, Z=%1.6lf, E=%1.6lf, F=%1.6lf\n",
	  POS2MM( gcode_current_pos.X), POS2MM( gcode_current_pos.Y),
	  POS2MM( gcode_current_pos.Z), POS2MM( gcode_current_pos.E),
	  gcode_current_feed);
  printf(  "origin: X=%1.6lf, Y=%1.6lf, Z=%1.6lf, E=%1.6lf\n",
	  POS2MM( gcode_home_pos.X), POS2MM( gcode_home_pos.Y),
	  POS2MM( gcode_home_pos.Z), POS2MM( gcode_home_pos.E));
  pruss_dump_position();
}


static void clip_move( axis_e axis, int32_t* pnext_target, int32_t current_pos, int32_t home_pos)
{
  double limit;
  if (*pnext_target >= current_pos) {
    // forward move or no move
    if (config_max_soft_limit( axis, &limit)) {
      int32_t pos_limit = MM2POS( limit);
      if (home_pos + current_pos > pos_limit) {
        pos_limit = home_pos + current_pos;
      }
      if (home_pos + *pnext_target > pos_limit) {
        printf( "WARNING: Clipping target.%c (%d) to %d due to upper soft limit= %d (home= %d)\n",
                axisName( axis), *pnext_target, pos_limit, MM2POS( limit), home_pos);
        *pnext_target = pos_limit - home_pos;
      }
    }       
  } else {
    // backward move
    if (config_min_soft_limit( axis, &limit)) {
      int32_t pos_limit = MM2POS( limit);
      if (home_pos + current_pos < pos_limit) {
        pos_limit = home_pos + current_pos;
      }
      if (home_pos + *pnext_target < pos_limit) {
        printf( "WARNING: Clipping target.%c (%d) to %d due to lower soft limit= %d (home= %d)\n",
                axisName( axis), *pnext_target, pos_limit, MM2POS( limit), home_pos);
        *pnext_target = pos_limit - home_pos;
      }
    }       
  }
}

static void process_non_move_command( GCODE_COMMAND* target)
{
	uint32_t	backup_f;

	if (target->seen_F) {
		gcode_current_feed = target->target.F;
	} else {
		target->target.F = gcode_current_feed;
	}
	// convert relative to absolute
	if (target->option_relative) {
		target->target.X += gcode_current_pos.X;
		target->target.Y += gcode_current_pos.Y;
		target->target.Z += gcode_current_pos.Z;
		target->target.E += gcode_current_pos.E;
	}
	// The GCode documentation was taken from http://reprap.org/wiki/Gcode .

	if (target->seen_T) {
	    //? ==== T: Select Tool ====
	    //?
	    //? Example: T1
	    //?
	    //? Select extruder number 1 to build with.  Extruder numbering starts at 0.

	    next_tool = target->T;
	}

	// if we didn't see an axis word, set it to gcode_current_pos. this fixes incorrect moves after homing TODO: fix homing ???
//TODO: fix this ???
	if (target->seen_X == 0)
		target->target.X = gcode_current_pos.X;
	if (target->seen_Y == 0)
		target->target.Y = gcode_current_pos.Y;
	if (target->seen_Z == 0)
		target->target.Z = gcode_current_pos.Z;
	if (target->seen_E == 0)
		target->target.E = gcode_current_pos.E;

	if (target->seen_G) {
		uint8_t axisSelected = 0;
		switch (target->G) {
			// 	G0 - rapid, unsynchronised motion
			// since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
			case 0:
				//? ==== G0: Rapid move ====
				//?
				//? Example: G0 X12
				//?
				//? In this case move rapidly to X = 12 mm.  In fact, the RepRap firmware uses exactly the same code for rapid as it uses for controlled moves (see G1 below), as - for the RepRap machine - this is just as efficient as not doing so.  (The distinction comes from some old machine tools that used to move faster if the axes were not driven in a straight line.  For them G0 allowed any movement in space to get to the destination as fast as possible.)
			case 1:
			{
#if 1
				printf( "\n\n\nOOPS\n\nThis shouldn't happen!!!!\n\n\n");
#else
				//? ==== G1: Controlled move ====
				//?
				//? Example: G1 X90.6 Y13.8 E22.4
				//?
				//? Go in a straight line from the current (X, Y) point to the point (90.6, 13.8), extruding material as the move happens from the current extruded length to a length of 22.4 mm.
				/*
				 *  Implement soft axis limits:
				 *
				 *  The soft axis limits define a safe operating zone.
				 *  Coordinates are clipped in such a way that no moves are generate that would move
				 *  from the inside to the outside of the safe operating zone. All moves from outside
				 *  the safe operating zone directed towards the inside of the zone are allowed!
				 */
				if (target->seen_X) {
					clip_move( x_axis, &target->target.X, gcode_current_pos.X, gcode_home_pos.X);
				}
				if (target->seen_Y) {
					clip_move( y_axis, &target->target.Y, gcode_current_pos.Y, gcode_home_pos.Y);
				}
				if (target->seen_Z) {
					clip_move( z_axis, &target->target.Z, gcode_current_pos.Z, gcode_home_pos.Z);
				}

				if (target->G == 0) {
					backup_f = target->target.F;
					target->target.F = 100000;	// will be limited by the limitations of the individual axes
					enqueue_pos( &target->target);
					target->target.F = backup_f;
				} else {
					// synchronised motion
					enqueue_pos( &target->target);
				}
				/* update our sense of position */
				gcode_current_pos.X = target->target.X;
				gcode_current_pos.Y = target->target.Y;
				gcode_current_pos.Z = target->target.Z;
				gcode_current_pos.E = target->target.E;
				gcode_current_feed = target->target.F;
#endif
				break;
			}
				//	G2 - Arc Clockwise
				// unimplemented

				//	G3 - Arc Counter-clockwise
				// unimplemented

				//	G4 - Dwell
			case 4:
				//? ==== G4: Dwell ====
				//?
				//? Example: G4 P200
				//?
				//? In this case sit still doing nothing for 200 milliseconds.  During delays the state of the machine (for example the temperatures of its extruders) will still be preserved and controlled.
				//?

				traject_wait_for_completion();
				usleep( 1000* target->P);
				break;

				//	G20 - inches as units
			case 20:
				//? ==== G20: Set Units to Inches ====
				//?
				//? Example: G20
				//?
				//? Units from now on are in inches.
				//?
				target->option_inches = 1;
				break;

				//	G21 - mm as units
			case 21:
				//? ==== G21: Set Units to Millimeters ====
				//?
				//? Example: G21
				//?
				//? Units from now on are in millimeters.  (This is the RepRap default.)
				//?
				target->option_inches = 0;
				break;

				//	G30 - go home via point
			case 30:
				//? ==== G30: Go home via point ====
				//?
				//? Undocumented.
				enqueue_pos( &target->target);
				// no break here, G30 is move and then go home

				//	G28 - Move to Origin
			case 28:
				//? ==== G28: Move to Origin ====
				//?
				// 20110817 modmaker - Changed implementation according to info found in
				//                     these docs: http://linuxcnc.org/docs/html/gcode.html,
				// http://reprap.org/wiki/MCodeReference and http://reprap.org/wiki/GCodes .
				// G28 generates a rapid traversal to the origin (or a preset position).
				// Implementation: G0 like move with as destination the origin (x,y,z=0,0,0).
				// The (absolute) origin is set at startup (current position) or by executing
				// a calibration command (G161/G162) for one or more axes.
				if (target->seen_X) {
					target->target.X = 0;
					axisSelected = 1;
				}
				if (target->seen_Y) {
					target->target.Y = 0;
					axisSelected = 1;
				}
				if (target->seen_Z) {
					target->target.Z = 0;
					axisSelected = 1;
				}
				if (axisSelected != 1) {
					target->target.X = 0;
					target->target.Y = 0;
					target->target.Z = 0;
				}
				backup_f = target->target.F;
				target->target.F = 99999;		// let the software clip this to the maximum allowed rate
				enqueue_pos( &target->target);
				target->target.F = backup_f;
				break;

			//	G90 - absolute positioning
			case 90:
				//? ==== G90: Set to Absolute Positioning ====
				//?
				//? Example: G90
				//?
				//? All coordinates from now on are absolute relative to the origin of the machine.  (This is the RepRap default.)
				target->option_relative = 0;
				break;

				//	G91 - relative positioning
			case 91:
				//? ==== G91: Set to Relative Positioning ====
				//?
				//? Example: G91
				//?
				//? All coordinates from now on are relative to the last position.
				target->option_relative = 1;
				break;

				//	G92 - set home
			case 92:
				//? ==== G92: Set Position ====
				//?
				//? Example: G92 X10 E90
				//?
				//? Allows programming of absolute zero point, by reseting the current position to the values specified.  This would set the machine's X coordinate to 10, and the extrude coordinate to 90. No physical motion will occur.

				traject_wait_for_completion();

				if (target->seen_X) {
					gcode_home_pos.X += gcode_current_pos.X - target->target.X;
					gcode_current_pos.X = target->target.X;
					axisSelected = 1;
				}
				if (target->seen_Y) {
					gcode_home_pos.Y += gcode_current_pos.Y - target->target.Y;
					gcode_current_pos.Y = target->target.Y;
					axisSelected = 1;
				}
				if (target->seen_Z) {
					gcode_home_pos.Z += gcode_current_pos.Z - target->target.Z;
					gcode_current_pos.Z = target->target.Z;
					axisSelected = 1;
				}
				// Have to handle E-axis specially because it may be a relative-only axis
				if (target->seen_E) {
					if (!config_e_axis_is_always_relative() && target->target.E == 0) {
						// slicers use this te adjust the origin to prevent running
						// out of E range, adjust the PRUSS internal origin too.
						pruss_queue_adjust_origin( 4, gcode_home_pos.E + gcode_current_pos.E);
						// gcode_home_pos can overflow too, so clear it! NOTE: the E-axis
						// now doesn't behave like a normal (absolute) axis anymore!
						gcode_home_pos.E = 0;
					} else {
						gcode_home_pos.E += gcode_current_pos.E - target->target.E;
					}
					gcode_current_pos.E = target->target.E;
					axisSelected = 1;
				}
				if (axisSelected == 0) {
					gcode_home_pos.X += gcode_current_pos.X;
					gcode_current_pos.X = target->target.X = 0;
					gcode_home_pos.Y += gcode_current_pos.Y;
					gcode_current_pos.Y = target->target.Y = 0;
					gcode_home_pos.Z += gcode_current_pos.Z;
					gcode_current_pos.Z = target->target.Z = 0;
					gcode_home_pos.E += gcode_current_pos.E;
					gcode_current_pos.E = target->target.E = 0;
				}
				break;

#define FOR_ONE_AXIS( axis_lc, axis_uc, axis_i, code) \
	do {								\
		axis_xyz = axis_lc##_axis;				\
		pruss_axis_xyz = axis_i;				\
		next_target_seen_xyz = target->seen_##axis_uc;	\
		current_pos_xyz = gcode_current_pos.axis_uc;		\
		home_pos_xyz = gcode_home_pos.axis_uc;			\
		code							\
		gcode_current_pos.axis_uc = current_pos_xyz;		\
		gcode_home_pos.axis_uc = home_pos_xyz;			\
	} while (0)

#define FOR_EACH_AXIS_IN_XYZ( code) \
	do {								\
		uint32_t feed = target->target.F;			\
		axis_e axis_xyz;					\
		int pruss_axis_xyz;		   			\
		int next_target_seen_xyz;				\
		int32_t current_pos_xyz;				\
		int32_t home_pos_xyz;					\
		/* X */							\
		FOR_ONE_AXIS( x, X, 1, code);				\
		FOR_ONE_AXIS( y, Y, 2, code);				\
		FOR_ONE_AXIS( z, Z, 3, code);				\
	} while (0)
#define MIN_LIMIT_SWITCH	1
#define MAX_LIMIT_SWITCH	0

			// G161 - Home negative
			case 161:
			// G162 - Home positive
			case 162:
			{
				//? ==== G161: Home negative ====
				//? ==== G162: Home positive ====
				//?
				//? Find the corresponding limit of the specified axes by searching for the limit switch.
				// reference 'home' position to (then) current position

				// Also used feed override for homing
				double factor = traject_set_speed_override( 0.0);	// get old value
				traject_set_speed_override( factor);			// restore
				target->target.F *= factor;
				fprintf( stdout, "Homing feed = %u (factor=%1.3lf)\n", target->target.F, factor);

				// NOTE: G161/G162 clears any G92 offset !
				double pos;
				if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
					fprintf( stderr, "G16%c: X(%d)=%d, Y(%d)=%d, Z(%d)=%d, E(%d)=%d, F(%d)=%d\n",
						(target->G == 162) ? '2' : '1',
						target->seen_X, target->target.X,
						target->seen_Y, target->target.Y,
						target->seen_Z, target->target.Z,
						target->seen_E, target->target.E,
						target->seen_F, target->target.F );
				}
				FOR_EACH_AXIS_IN_XYZ(
					if (next_target_seen_xyz) {
						int limit_switch = (target->G == 161) ? MIN_LIMIT_SWITCH : MAX_LIMIT_SWITCH;
						/*
						 * Temporarily use machine coordinates during homing
						 */
						current_pos_xyz += home_pos_xyz;
						home_axis_to_limit_switch( axis_xyz, &current_pos_xyz, feed, limit_switch);
						current_pos_xyz -= home_pos_xyz;
						/*
						 *  If the limit switch is also a calibration switch, set the origin
						 */
						int is_reference_switch;
						if (limit_switch == MIN_LIMIT_SWITCH) {
							is_reference_switch = config_min_switch_pos( axis_xyz, &pos);
						} else {
							is_reference_switch = config_max_switch_pos( axis_xyz, &pos);
						}
						if (is_reference_switch) {
							home_pos_xyz = 0;
							current_pos_xyz = SI2POS( pos);
							pruss_queue_set_position( pruss_axis_xyz, home_pos_xyz + current_pos_xyz);
						}
					} );

				break;
			}

			// G244 - Resume halted PRUSS
			case 244: {
				int i = 1;
				if (!target->seen_S || target->S > 1) {
					i = target->S;	// number of iterations
				}
				do {
					pruss_stepper_resume();
					if (i > 1) {
						while (!pruss_stepper_halted()) {
							usleep( 100);
						}
					}
				} while (--i > 0);
				break;
			}
			// G255 - Dump PRUSS state
			case 255:
				// === G255: Dump PRUSS state ====
				// The (optional) parameter S0, will disable waiting
				// for the current command to complete, before dumping.
				if (!target->seen_S || target->S != 0) {
				  traject_wait_for_completion();
				}
				pruss_stepper_dump_state();
				break;

				// unknown gcode: spit an error
			default:
				printf("E: Bad G-code %d", target->G);
				// newline is sent from gcode_parse after we return
				return;
		}
#ifdef	DEBUG
		if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
			dump_position_info();
#  if 0
			traject_status_print();
#  endif
		}
#endif
	}
	else if (target->seen_M) {
		switch (target->M) {
			// M0- machine stop
			case 0:
			// M2- program end
			case 2:
				//? ==== M2: program end ====
				//?
				//? Undocumented.
				traject_wait_for_completion();
				// no break- we fall through to M112 below
			// M112- immediate stop
			case 112:
				//? ==== M112: Emergency Stop ====
				//?
				//? Example: M112
				//?
				//? Any moves in progress are immediately terminated, then RepRap shuts down.  All motors and heaters are turned off.
				//? It can be started again by pressing the reset button on the master microcontroller.  See also M0.

				traject_abort();

				x_disable();
				y_disable();
				z_disable();
				e_disable();
				power_off();
				for (;;) {
					usleep( 1000);
				}
				break;

			// M6- tool change
			case 6:
				//? ==== M6: tool change ====
				//?
				//? Undocumented.
				tool = next_tool;
				break;
			// M82- set extruder to absolute mode
			case 82: {
				int old_mode = config_set_e_axis_mode( 0);
				if (old_mode != 0 && DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
					fprintf( stderr, "M82: switching to absolute extruder coordinates\n");
				}
				break;
			}
			// M83- set extruder to relative mode
			case 83: {
				int old_mode = config_set_e_axis_mode( 1);
				if (old_mode == 0 && DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
					fprintf( stderr, "M83: switching to relative extruder coordinates\n");
				}
				break;
			}
			// M84- stop idle hold
			case 84:
				x_disable();
				y_disable();
				z_disable();
				e_disable();
				break;
			// M3/M101- extruder on
			case 3:
			case 101:
				//? ==== M101: extruder on ====
				//?
				//? Undocumented.
				#ifdef DC_EXTRUDER
					heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
				#elif E_STARTSTOP_STEPS > 0
					do {
						// backup feedrate, move E very quickly then restore feedrate
						backup_f = gcode_current_feed;
						gcode_current_feed = MAXIMUM_FEEDRATE_E;
						SpecialMoveE( E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
						gcode_current_feed = backup_f;
					} while (0);
				#endif
				break;

			// M102- extruder reverse

			// M5/M103- extruder off
			case 5:
			case 103:
				//? ==== M103: extruder off ====
				//?
				//? Undocumented.
				#ifdef DC_EXTRUDER
					heater_set(DC_EXTRUDER, 0);
				#elif E_STARTSTOP_STEPS > 0
					do {
						// backup feedrate, move E very quickly then restore feedrate
						backup_f = gcode_current_feed;
						gcode_current_feed = MAXIMUM_FEEDRATE_E;
						SpecialMoveE( E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
						gcode_current_feed = backup_f;
					} while (0);
				#endif
				break;

			// M104- set temperature
			case 104:
				//? ==== M104: Set Extruder Temperature (Fast) ====
			case 140:
				//? ==== M140: Set heated bed temperature (Fast) ====
			case 109:
				//? ==== M109: Set Extruder Temperature (Wait) ====
			case 190:
				//? ==== M190: Set Bed Temperature (Wait)  ====
			{
				channel_tag heater;
				if (target->M == 140 || target->M == 190) {
					heater = heater_bed;
				} else {
					if (target->seen_P && target->P == 1) {
						heater = heater_bed;
					} else {
						heater = heater_extruder;
					}
				}
				if (target->seen_S) {
					heater_set_setpoint( heater, target->S);
					// if setpoint is not null, turn power on
					if (target->S > 0) {
						power_on();
						heater_enable( heater_extruder, 1);
					} else {
						heater_enable( heater_extruder, 0);
					}
				}
				if (target->M == 109 || target->M == 190) {
					if (heater == heater_bed) {
						bed_temp_wait = 1;
					} else {
						extruder_temp_wait = 1;
					}
				}
				break;
			}
			// M105- get temperature
			case 105: {
				//? ==== M105: Get Extruder Temperature ====
				//?
				//? Example: M105
				//?
				//? Request the temperature of the current extruder and the build base in degrees Celsius.  The temperatures are returned to the host computer.  For example, the line sent to the host in response to this command looks like
				//?
				//? <tt>ok T:201 B:117</tt>
				//?
				//? Teacup supports an optional P parameter as a sensor index to address.
				double celsius;
#				ifdef ENFORCE_ORDER
					// wait for all moves to complete
					traject_wait_for_completion();
#				endif
				if (target->seen_P) {
					channel_tag temp_source;
					switch (target->P) {
					case 0:  temp_source = heater_extruder; break;
					case 1:  temp_source = heater_bed; break;
					default: temp_source = NULL;
					}
					if (heater_get_celsius( temp_source, &celsius) == 0) {
						printf( "\nT:%1.1lf", celsius);
					}
				} else {
					heater_get_celsius( heater_extruder, &celsius);
					printf( "\nT:%1.1lf", celsius);
					if (heater_bed != NULL) {
						heater_get_celsius( heater_bed, &celsius);
						printf( " B:%1.1lf", celsius);
					}
				}
				break;
			}
			// M7 - cooling mist on
			case 7:
				traject_wait_for_completion();
				pwm_set_output( pwm_fan, 100);
				break;

			// M106 - pwm device control
			case 106: {
				channel_tag pwm_device = pwm_fan;	// unless overridden by P setting
				int pwm_value  = 100;			// unless overridden by S setting
				
				//? ==== M106: output control ====
				//?
				//? Example: M106
				//?
				//? General purpose output control, backwards compatible
				//? with simple FAN control (without P and/or S).
				//? Sets PWM value for the device selected with P
				//? to output value as specified by S.
				//? S value 0..255 maps to 0..100% PWM output.

				// wait for all moves to complete
				traject_wait_for_completion();
				if (target->seen_P) {
					switch (target->P) {
					case 0:  pwm_device = pwm_extruder; break;
					case 1:  pwm_device = pwm_bed; break;
					case 2:  pwm_device = pwm_fan; break;
					}
				}
				if (target->seen_S) {
					if (target->S >= 255) {
						pwm_value = 100;
					} else {
						pwm_value = (100 * target->S) / 256;
					}
				}
				pwm_set_output( pwm_device, pwm_value);
				break;
			}
			// M107- fan off
			case 9:
			case 107:
				//? ==== M107: Fan Off ====
				//?
				//? Example: M107
				//?
				//? Turn off the cooling fan (if any).

				// wait for all moves to complete
				traject_wait_for_completion();
				pwm_set_output( pwm_fan, 0);
				break;

			// M110- set line number
			case 110:
				//? ==== M110: Set Current Line Number ====
				//?
				//? Example: N123 M110
				//?
				//? Set the current line number to 123.  Thus the expected next line after this command will be 124.
				//? This is a no-op in Teacup.
				break;
			// M111- set debug level
			#ifdef	DEBUG
			case 111:
				//? ==== M111: Get/Set Debug Level ====
				//?
				//? Example: M111 S6
				//?
				//? Set the level of debugging information transmitted back to the host to level 6.  The level is the OR of three bits:
				//?
				//? <Pre>
				//? #define         DEBUG_PID       1
				//? #define         DEBUG_DDA       2
				//? #define         DEBUG_POSITION  4
				//? </pre>
				//?
				//? This command is only available in DEBUG builds of Teacup.

				if (target->seen_S) {
					debug_flags = target->S;
					printf( "New debug_flags setting: 0x%04x (%u)\n", debug_flags, debug_flags);
				} else {
					printf( "Active debug_flags setting: 0x%04x (%u)\n", debug_flags, debug_flags);
				}
				break;
			#endif
			// M113- extruder PWM
			case 113: {
				//? ==== M113: Set (extruder) PWM ====
				//?
				//? Example: M113 S0.125
				//?
				//? Set the (raw) extruder heater output to the specified value: 0.0-1.0 gives 0-100% duty cycle.
				//? Should only be used when there is no heater control loop configured for this output!!!
				if (target->seen_S) {
					pwm_set_output( pwm_extruder, target->S);
				}
				break;
			}
			// M114- report XYZEF to host
			case 114:
				//? ==== M114: Get Current Position ====
				//?
				//? Example: M114
				//?
				//? This causes the RepRap machine to report its current X, Y, Z and E coordinates to the host.
				//?
				//? For example, the machine returns a string such as:
				//?
				//? <tt>ok C: X:0.00 Y:0.00 Z:0.00 E:0.00</tt>
#				ifdef ENFORCE_ORDER
					// wait for all moves to complete
					traject_wait_for_completion();
#				endif
				printf(  "current: X=%1.6lf, Y=%1.6lf, Z=%1.6lf, E=%1.6lf, F=%1.6lf\n",
					POS2MM( gcode_current_pos.X), POS2MM( gcode_current_pos.Y),
					POS2MM( gcode_current_pos.Z), POS2MM( gcode_current_pos.E),
					gcode_current_feed);
				// newline is sent from gcode_parse after we return

				break;
			// M115- capabilities string
			case 115:
				//? ==== M115: Get Firmware Version and Capabilities ====
				//?
				//? Example: M115
				//?
				//? Request the Firmware Version and Capabilities of the current microcontroller
				//? The details are returned to the host computer as key:value pairs separated by spaces and terminated with a linefeed.
				//?
				//? sample data from firmware:
				//?  FIRMWARE_NAME:Teacup FIRMWARE_URL:http%%3A//github.com/triffid/Teacup_Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 TEMP_SENSOR_COUNT:1 HEATER_COUNT:1

				printf( "FIRMWARE_NAME: BeBoPr FIRMWARE_URL:https//github.com/modmaker/BeBoPr/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:%d TEMP_SENSOR_COUNT:%d HEATER_COUNT:%d", 1, 2, 2);
				// newline is sent from gcode_parse after we return
				break;
			// M116 - Wait for all temperatures and other slowly-changing variables to arrive at their set values.
			case 116: {
				//? ==== M116: Wait ====
				//?
				//? Example: M116
				//?
				//? Wait for ''all'' temperatures and other slowly-changing variables to arrive at their set values.  See also M109.

				double setpoint;
				// wait for all moves to complete
				traject_wait_for_completion();
				// wait for all (active) heaters to stabilize
				if (heater_get_setpoint( heater_extruder, &setpoint) == 0) {
					if (setpoint > 0.0) {
						extruder_temp_wait = 1;
					}
				}
				if (heater_get_setpoint( heater_bed, &setpoint) == 0) {
					if (setpoint > 0.0) {
						bed_temp_wait = 1;
					}
				}
				wait_for_slow_signals( NULL);
				break;
			}
			case 130:
				//? ==== M130: heater P factor ====
			case 131:
				//? ==== M131: heater I factor ====
			case 132:
				//? ==== M132: heater D factor ====
			case 133:
				//? ==== M133: heater I limit ====

				//? P0: set for extruder
				//? P1: set for bed
				//? Snnn.nn: factor to set
				if (target->seen_S) {
					pid_settings pid;
					channel_tag channel;
					if (target->seen_P) {
						switch (target->P) {
						case 0:  channel = heater_extruder; break;
						case 1:  channel = heater_bed; break;
						default: channel = NULL;
						}
					} else {
						channel = heater_extruder;
					}
					heater_get_pid_values( channel, &pid);
					switch (target->M) {
					case 130:	// M130- heater P factor
						pid.P = target->S;
						break;
					case 131:	// M131- heater I factor
						pid.I = target->S;
						break;
					case 132:	// M132- heater D factor
						pid.D = target->S;
						break;
					case 133:	// M133- heater I limit
						pid.I_limit = target->S;
						break;
					}
					heater_set_pid_values( channel, &pid);
				}
				break;
			// M134- save PID settings to eeprom
			case 134:
				//? ==== M134: save PID settings to eeprom ====
				//? Undocumented.
				heater_save_settings();
				break;
			// M135- set heater output
			case 135:
				//? ==== M135: set heater output ====
				//? Undocumented.
				if (target->seen_S) {
					channel_tag heater;
					switch (target->P) {
					case 0:  heater = heater_extruder; break;
					case 1:  heater = heater_bed; break;
					default: heater = NULL;
					}
					heater_set_raw_pwm( heater, target->S);
					power_on();
				}
				break;
			#ifdef	DEBUG
			// M136- PRINT PID settings to host
			case 136: {
				//? ==== M136: PRINT PID settings to host ====
				//? Undocumented.
				//? This comand is only available in DEBUG builds.
				pid_settings pid;
				channel_tag heater;
				if (target->seen_P) {
					switch (target->P) {
					case 0:  heater = heater_extruder; break;
					case 1:  heater = heater_bed; break;
					default: heater = NULL;
					}
				} else {
					heater = heater_extruder;
				}
				heater_get_pid_values( heater, &pid);
				printf( "P:%1.3f I:%1.3f D:%1.3f Ilim:%1.3f FF_factor:%1.3f FF_offset:%1.3f",
					pid.P, pid.I, pid.D, pid.I_limit, pid.FF_factor, pid.FF_offset);
				break;
			}
			#endif

			// M191- power off
			case 191:
				//? ==== M191: Power Off ====
				//? Undocumented.
#				ifdef ENFORCE_ORDER
					// wait for all moves to complete
					traject_wait_for_completion();
#				endif
				x_disable();
				y_disable();
				z_disable();
				e_disable();
				power_off();
				break;

			// M200 - report endstop status
			case 200:
			{
				//? ==== M200: report endstop status ====
				//? Report the current status of the endstops configured in the firmware to the host.
				int no_limit_switches = 1;
				if (config_axis_has_min_limit_switch( x_axis)) {
				    printf( "x_min:%d ", limsw_min( x_axis));
				    no_limit_switches = 0;
				}
				if (config_axis_has_max_limit_switch( x_axis)) {
				    printf( "x_max:%d ", limsw_max( x_axis));
				    no_limit_switches = 0;
				}
				if (config_axis_has_min_limit_switch( y_axis)) {
				    printf( "y_min:%d ", limsw_min( y_axis));
				    no_limit_switches = 0;
				}
				if (config_axis_has_max_limit_switch( y_axis)) {
				    printf( "y_max:%d ", limsw_max( y_axis));
				    no_limit_switches = 0;
				}
				if (config_axis_has_min_limit_switch( z_axis)) {
				    printf( "z_min:%d ", limsw_min( z_axis));
				    no_limit_switches = 0;
				}
				if (config_axis_has_max_limit_switch( z_axis)) {
				    printf( "z_max:%d ", limsw_max( z_axis));
				    no_limit_switches = 0;
				}
				if (no_limit_switches) {
				    printf("no endstops defined");
				}
				break;
			}
			// M207 - Calibrate reference switch position (Z-axis)
			case 207:
			{
				double pos;
				int min_max = 0;
				if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
					fprintf( stderr, "M207: Z axis known position <-> reference switch calibration\n");
				}
				/*
				 *  Clear home offset, specifief current_pos is in machine coordinates (???)
				 *  NOTE: the calculations that follow use home_pos (that is set to zero),
				 *        do not optimize the code as this shows the correct calculations!
				 */
				gcode_home_pos.Z = 0;
				if (target->seen_Z) {
					gcode_current_pos.Z = target->target.Z;
				} else {
					gcode_current_pos.Z = 0;
				}
				pruss_queue_set_position( 3, gcode_home_pos.Z + gcode_current_pos.Z);
				/*
				 * Temporarily use machine coordinates during homing
				 */
				gcode_current_pos.Z += gcode_home_pos.Z;
				if (config_max_switch_pos( z_axis, &pos)) {
					home_axis_to_limit_switch( z_axis, &gcode_current_pos.Z, target->target.F, MAX_LIMIT_SWITCH);
					min_max = 1;
				} else if (config_min_switch_pos( z_axis, &pos)) {
					home_axis_to_limit_switch( z_axis, &gcode_current_pos.Z, target->target.F, MIN_LIMIT_SWITCH);
					min_max = -1;
				}
				gcode_current_pos.Z -= gcode_home_pos.Z;
				/*
				 *  If we found a calibration limit switch, set the origin
				 */
				if (min_max) {
					if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
						fprintf( stderr, "M207: update Z calibration switch position to: %lf [mm]\n",
							POS2MM( gcode_current_pos.Z));
					}
					// Clear home offset and set new calibration position
					config_set_cal_pos( z_axis, POS2SI( gcode_current_pos.Z));
					gcode_home_pos.Z = 0;
					pruss_queue_set_position( 3, gcode_home_pos.Z + gcode_current_pos.Z);
				}
				break;
			}
			case 220:
				//? ==== M220: speed override factor ====
			case 221:
				//? ==== M221: extruder override factor ====
				if (target->seen_S) {
					double old;
					double factor = 0.001 * target->S;
					if (factor < 0.001) {
						factor = 0.001;
					}
					if (target->M == 220) {
						old = traject_set_speed_override( factor);
					} else {
						old = traject_set_extruder_override( factor);
					}
					if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
						fprintf( stderr, "M%d: set %s override factor to %1.3lf, old value was %1.3lf\n",
							target->M, (target->M == 221) ? "extruder" : "speed", factor, old);
					}
				}
				break;
			#ifdef	DEBUG
			// M240- echo off
			case 240:
				//? ==== M240: echo off ====
				//? Disable echo.
				//? This command is only available in DEBUG builds.
				debug_flags &= ~DEBUG_ECHO;
				printf( "Echo off");
				// newline is sent from gcode_parse after we return
				break;
				// M241- echo on
			case 241:
				//? ==== M241: echo on ====
				//? Enable echo.
				//? This command is only available in DEBUG builds.
				debug_flags |= DEBUG_ECHO;
				printf( "Echo on");
				// newline is sent from gcode_parse after we return
				break;

			// DEBUG: return current position, end position, queue
			case 250:
				//? ==== M250: return current position, end position, queue ====
				//? Undocumented
				//? This command is only available in DEBUG builds.
				dump_position_info();
				break;

			// DEBUG: read arbitrary memory location
			case 253:
				//? ==== M253: read arbitrary memory location ====
				//? Undocumented
				//? This command is only available in DEBUG builds.

				// 2012-06-04 modmaker - not implemented, this is not an AVR!
				break;

			// DEBUG: write arbitrary memory location
			case 254:
				//? ==== M254: write arbitrary memory location ====
				//? Undocumented
				//? This command is only available in DEBUG builds.

				// 2012-06-04 modmaker - not implemented, this is not an AVR!
				break;
			#endif /* DEBUG */
				// unknown mcode: spit an error
			default:
				printf("E: Bad M-code %d", target->M);
				// newline is sent from gcode_parse after we return
		} // switch (target->M)
	} // else if (target->seen_M)
} // process_gcode_command()


struct move_target {
  TARGET target;
  TARGET current_pos;
  uint32_t current_feed;
  int data_valid;
  move5D data;
};

/*
 *  Move from current_pos to new position, update current_pos
 *
 *  Base actual move velocities on current 'move' and 'next_move'
 */
static void process_move_command( struct move_target* move, struct move_target* next_move)
{
  if (next_move /* && next_move->data_valid*/) {
   /*
    * move to resume
    */
#if 0
    move5D* p0 = &move->data;
    move5D* p1 = &next_move->data;
#endif
    traject_optimize( &move->data, &next_move->data);
#if 0
    if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
      printf( "MOVE[ %lu] process_move_command() - starting chainable move\n"
	      "MOVE ... current move velocity ( %1.6lf, %1.6lf, %1.6lf, %1.6lf) F=%1.3lf [mm/s]\n"
	      "MOVE ...... next move velocity ( %1.6lf, %1.6lf, %1.6lf, %1.6lf) F=%1.3lf [mm/s]\n",
	      p0->serno,
	      SI2MM( v0x), SI2MM( v0y), SI2MM( v0z), SI2MM( v0e), SI2MM( FEED2SI( p0->feed)),
	      SI2MM( v1x), SI2MM( v1y), SI2MM( v1z), SI2MM( v1e), SI2MM( FEED2SI( p1->feed)) );
      printf( "MOVE[ %lu] delta velocities ( %1.6lf, %1.6lf, %1.6lf, %1.6lf) F=%1.3lf [mm/s]\n",
	      p0->serno, SI2MM( dvx), SI2MM( dvy), SI2MM( dvz), SI2MM( dve), SI2MM( FEED2SI( dfeed)) );
      printf( "MOVE[ %lu] distances needed ( %1.6lf, %1.6lf, %1.6lf, %1.6lf) [mm]\n",
	      p0->serno, SI2MM( dsx), SI2MM( dsy), SI2MM( dsz), SI2MM( dse) );
    }
#endif
  } else {
   /*
    * move to stop
    */
    if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
      printf( "MOVE[ %lu] process_move_command()  - starting terminating move\n", move->data.serno);
    }
  }

  if (!move->data.null_move) {

    move_execute( &move->data);

   /*
    * For a 3D printer, the E-axis controls the extruder and for that axis
    * the +/- 2000 mm operating range is not sufficient as this axis moves
    * mostly into one direction.
    * If this axis is configured to use relative coordinates only, after
    * each move the origin is shifted to the current position restoring the
    * full +/- 2000 mm operating range.
    */
    if (config_e_axis_is_always_relative()) {
      if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
        printf( "MOVE[ %lu] process_move_command() - E compensation over %d, origin to %d\n",
                move->data.serno, move->target.E, gcode_home_pos.E + move->target.E);
      }
      pruss_queue_adjust_origin( 4, gcode_home_pos.E + move->target.E);
      move->target.E = 0;       // target->E -= target->E;
    }

   /*
    * Update global sense of position and feed
    */
    gcode_current_pos.X = move->target.X;
    gcode_current_pos.Y = move->target.Y;
    gcode_current_pos.Z = move->target.Z;
    gcode_current_pos.E = move->target.E;
    gcode_current_feed = move->target.F;        // ???

  } else {

    if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
      printf( "*** MOVE[ %lu] - Null move encountered, ignored ***\n",
              move->data.serno);
    }
    // TODO: is any global variable update necessary ???

  }
}

/*
 * Calculate move details from new 'command' and store these in 'move'
 *
 * Should have no side effects except on move contents !!!
 */
static void preprocess_move_command( GCODE_COMMAND* command, TARGET* start_pos, struct move_target* move)
{
 /*
  *  At entry, move->current_pos and move->current_feed shall be initialized
  */
  move->target = command->target;

 /*
  *  This routine will only be called for G-code 0,1,2 or 3.
  *  The G0 command will run as G1, always at maximum allowed feed.
  */
  if (command->G == 0) {
    move->target.F = 999999;    // will be limited by the limitations of the individual axes
  } else {
    if (command->seen_F) {
      move->target.F = command->target.F;
    } else {
      move->target.F = move->current_feed;
    }
  }

 /*
  *  convert relative to absolute
  */
  if (command->option_relative) {
    move->target.X += move->current_pos.X;
    move->target.Y += move->current_pos.Y;
    move->target.Z += move->current_pos.Z;
    move->target.E += move->current_pos.E;
  }

 /*
  *  Implement soft axis limits:
  *
  *  The soft axis limits define a safe operating zone.
  *  Coordinates are clipped in such a way that no moves are generate that would move
  *  from the inside to the outside of the safe operating zone. All moves from outside
  *  the safe operating zone directed towards the inside of the zone are allowed!
  */
  if (command->seen_X) {
    clip_move( x_axis, &move->target.X, move->current_pos.X, gcode_home_pos.X);
  } else {
    move->target.X = move->current_pos.X;
  }
  if (command->seen_Y) {
    clip_move( y_axis, &move->target.Y, move->current_pos.Y, gcode_home_pos.Y);
  } else {
    move->target.Y = move->current_pos.Y;
  }
  if (command->seen_Z) {
    clip_move( z_axis, &move->target.Z, move->current_pos.Z, gcode_home_pos.Z);
  } else {
    move->target.Z = move->current_pos.Z;
  }
  if (!command->seen_E) {
    move->target.E = move->current_pos.E;
  }

 /*
  *  Calculate trajectory speed etc.
  */
  unsigned long seqno = get_serno();
  
  if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
    printf( "\nMOVE[ #%lu] \"%s\"\n", seqno, &command->command_text[0]);
  }
  move_calc( seqno, &move->target, start_pos, &move->data);

 /*
  *  Update our sense of (end) position
  */
  move->current_pos = move->target;
  if (command->G != 0) {
    move->current_feed = move->target.F;
  }
}

static int gcode_process_ordering_required( GCODE_COMMAND* command)
{
  return 1;	// until implemented, assume worst
}

/*
 * After the last (non-null) target, one more call (with a null argument)
 * should be made to process the last (buffered) target !!! This call is
 * made from mendel.c after detection of EOF on the input stream.
 * TODO: gcode_parse now sends an "ok" on buffering a command, not on
 *       execution. determine if this leads to problems.
 */
void process_gcode_command( GCODE_COMMAND* command)
{
  static int move_pending = 0;
  static struct move_target pending_move;
  if (command) {
    if (command->seen_G && (command->G >= 0 && command->G <= 3)) {
      struct move_target next_move;

      if (move_pending) {
       /*
        *  'next_move' will start with the end position from the pending move.
        *  The end position is the target position of that move, but the
        *  E coordinate requires special treatment if it is always relative.
        */
        TARGET start_pos = {
          .X = pending_move.target.X,
          .Y = pending_move.target.Y,
          .Z = pending_move.target.Z,
          .E = pending_move.target.E,
        };

        if (config_e_axis_is_always_relative()) {
          start_pos.E = 0;
        }
       /*
        *  Make 'command' the next move, starting from full stop
        *  at gcode_current_pos and with gcode_current_feed.
        */
        next_move.current_pos = pending_move.current_pos;
        next_move.current_feed = pending_move.current_feed;
        preprocess_move_command( command, &start_pos, &next_move);
        if (next_move.data.null_move) {
          return;
        }
       /*
        * Execute the current pending move
        */
        process_move_command( &pending_move, &next_move);
      } else {
       /*
        *  Make 'command' the next move, starting from full stop
        *  at gcode_current_pos and with gcode_current_feed.
        */
        next_move.current_pos = gcode_current_pos;
        next_move.current_feed = gcode_current_feed;
        preprocess_move_command( command, &gcode_current_pos, &next_move);
        if (next_move.data.null_move) {
          return;
        }
      }
     /*
      * 'next_move' becomes the new pending move
      */
      pending_move = next_move;
      move_pending = 1;
    } else {  /* not G0123 */
      /*
       *  The new command is not a move command (G0123).
       *  In case strict command ordering is required, 
       *  execute any pending moves before this new command.
       */
      if (move_pending && gcode_process_ordering_required( command)) {
       /*
        * Execute the current pending move
        */
        process_move_command( &pending_move, NULL);
        move_pending = 0;
      }
      // execute non-G0123 move
      process_non_move_command( command);
    }
  } else if (move_pending) {
   /*
    * Execute the current pending move
    */
    process_move_command( &pending_move, NULL);
    move_pending = 0;
  } else {
    // NOP
    move_pending = 0;
  }
}


int gcode_process_init( void)
{
  int result = mendel_sub_init( "traject", traject_init);
  if (result != 0) {
    return result;
  }
#ifdef PEPPER
  // configure the pepper stepper driver add-on board
  result = mendel_sub_init( "PEPPER", pepper_init);
  if (result != 0) {
    return result;
  }
#endif
  heater_extruder = heater_lookup_by_name( "heater_extruder");
  heater_bed      = heater_lookup_by_name( "heater_bed");
  temp_extruder   = temp_lookup_by_name( "temp_extruder");
  temp_bed        = temp_lookup_by_name( "temp_bed");
  pwm_extruder    = pwm_lookup_by_name( "pwm_extruder");
  pwm_fan         = pwm_lookup_by_name( "pwm_fan");
  pwm_bed         = pwm_lookup_by_name( "pwm_bed");
  if (DBG( DEBUG_GCODE_PROCESS + DEBUG_VERBOSE)) {
    printf( "tag_name( heater_extruder) = '%s',  tag_name( heater_bed) = '%s',\n",
            tag_name( heater_extruder), tag_name( heater_bed));
    printf( "tag_name( temp_extruder) = '%s',  tag_name( temp_bed) = '%s'\n",
            tag_name( temp_extruder), tag_name( temp_bed));
    printf( "tag_name( pwm_extruder) = '%s',  tag_name( pwm_fan) = '%s',  tag_name( pwm_bed) = '%s'\n",
            tag_name( pwm_extruder), tag_name( pwm_fan), tag_name( pwm_bed));
  }
  // If there's no extruder, or no laser power there's probably a configuration error!
  if ((heater_extruder == NULL || temp_extruder == NULL) && pwm_extruder == NULL) {
//    return -1;
  }
  gcode_current_pos.X = gcode_home_pos.X = 0;
  gcode_current_pos.Y = gcode_home_pos.Y = 0;
  gcode_current_pos.Z = gcode_home_pos.Z = 0;
  gcode_current_pos.E = gcode_home_pos.E = 0;
  gcode_current_feed  = 3000;
  return 0;
}
