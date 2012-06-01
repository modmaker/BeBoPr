#include	"gcode_process.h"

/** \file
	\brief Work out what to do with received G-Code commands
*/

#include	<string.h>
#include	<stdio.h>
#include	<math.h>
#include	<unistd.h>

#include	"gcode_parse.h"
#include	"dda_queue.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"temp.h"
#include	"heater.h"
#include	"timer.h"
#include	"sersendf.h"
#include	"pinio.h"
#include	"debug.h"
#include	"clock.h"
#include	"config.h"
#include	"home.h"
#include	"traject.h"
#include	"pruss.h"
#include	"heater.h"
#include	"beaglebone-stubs.h"

/// the current tool
uint8_t tool;

/// the tool to be changed when we get an M6
uint8_t next_tool;


// 20111017 modmaker - use nanometers instead of steps for position
/// variable that holds the idea of 'current position' for the gcode interpreter.
/// the actual machine position will probably lag!
static TARGET gcode_current_pos;
static TARGET gcode_home_pos;

static channel_tag heater_extruder = NULL;
static channel_tag heater_bed = NULL;
static channel_tag temp_extruder = NULL;
static channel_tag temp_bed = NULL;


void gcode_trace_move( void)
{
  if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
    // current position
    sersendf_P(PSTR("Pos: %d,%d,%d,%d,%u\n"),
	       gcode_home_pos.X, gcode_home_pos.Y, gcode_home_pos.Z,
	       gcode_home_pos.E, gcode_home_pos.F);
    // target position
    {
      unsigned mb_tail = dda_queue_get_mb_tail();
      sersendf_P(PSTR("Dst: %d,%d,%d,%d,%u\n"),
		 movebuffer[mb_tail].endpoint.X, movebuffer[mb_tail].endpoint.Y,
		 movebuffer[mb_tail].endpoint.Z, movebuffer[mb_tail].endpoint.E,
		 movebuffer[mb_tail].endpoint.F);
    }
    // Queue
    dda_queue_print();
    // newline
    serial_writechar('\n');
  }
}

void gcode_set_pos( char axis, uint32_t pos)
{
  switch (axis) {
  case 'X':
    gcode_current_pos.X = pos;
    gcode_home_pos.X = pos;
    break;
  case 'Y':
    gcode_current_pos.Y = pos;
    gcode_home_pos.Y = pos;
    break;
  case 'Z':
    gcode_current_pos.Z = pos;
    gcode_home_pos.Z = pos;
    break;
  case 'E':
    gcode_current_pos.E = pos;
    gcode_home_pos.E = pos;
    break;
  default:
    break;
  }
}

/*
	private functions

	this is where we construct a move without a gcode command, useful for gcodes which require multiple moves eg; homing
*/

static void enqueue_pos( TARGET* target)
{
  if (target != NULL) {
    if (DEBUG_GCODE_PROCESS && (debug_flags & DEBUG_GCODE_PROCESS)) {
      fprintf( stderr,  "\ntraject_move_all_axes( target(%d,%d,%d,%d,%u) ]\n",
	       target->X, target->Y, target->Z, target->E, target->F);
    }
    traject5D traj;

    /* integer positions are in nm ! */
    traj.dx = (double)1.0E-9 * (target->X - gcode_current_pos.X);
    traj.dy = (double)1.0E-9 * (target->Y - gcode_current_pos.Y);
    traj.dz = (double)1.0E-9 * (target->Z - gcode_current_pos.Z);
    traj.de = (double)1.0E-9 * (target->E - gcode_current_pos.E);
    traj.feed = target->F;

    traject_delta_on_all_axes( &traj);
    // update our sense of position
    memcpy( &gcode_current_pos, target, sizeof( TARGET));
  }
}



#if E_STARTSTOP_STEPS > 0
/// move E by a certain amount at a certain speed
static void SpecialMoveE(int32_t e, uint32_t f) {
	TARGET t = gcode_current_pos;
	// TODO: make sure calculation does not overflow!
	// TODO: find a better solution/place for this code.
	t.E += (e * 1000000) / STEPS_PER_MM_E;
	t.F = f;
	dda_queue_enqueue( &t);
}
#endif /* E_STARTSTOP_STEPS > 0 */


/************************************************************************//**

  \brief Processes command stored in global \ref next_target.
  This is where we work out what to actually do with each command we
    receive. All data has already been scaled to integers in gcode_process.
    If you want to add support for a new G or M code, this is the place.


*//*************************************************************************/

void process_gcode_command() {
	uint32_t	backup_f;

	// convert relative to absolute
	if (next_target.option_relative) {
		next_target.target.X += gcode_current_pos.X;
		next_target.target.Y += gcode_current_pos.Y;
		next_target.target.Z += gcode_current_pos.Z;
		#ifdef	E_ABSOLUTE
			next_target.target.E += gcode_current_pos.E;
		#endif
	}
	// E ALWAYS relative, otherwise we overflow our registers after only a few layers
	// 	next_target.target.E += startpoint.E;
	// easier way to do this
	// 	startpoint.E = 0;
	// moved to dda.c, end of dda_create() and dda_queue.c, next_move()

	// implement axis limits
	#ifdef	X_MIN
	if (next_target.target.X < MM_TO_POS( X_MIN)) {
	  printf( "Limiting target.X (%d) to X_MIN (%d)\n", next_target.target.X, MM_TO_POS( X_MIN));
	  next_target.target.X = MM_TO_POS( X_MIN);
	}	
	#endif
	#ifdef	X_MAX
	if (next_target.target.X > MM_TO_POS( X_MAX)) {
	  printf( "Limiting target.X (%d) to X_MAX (%d)\n", next_target.target.X, MM_TO_POS( X_MAX));
	  next_target.target.X = MM_TO_POS( X_MAX);
	}
	#endif
	#ifdef	Y_MIN
	if (next_target.target.Y < MM_TO_POS( Y_MIN)) {
	  printf( "Limiting target.Y (%d) to Y_MIN (%d)\n", next_target.target.Y, MM_TO_POS( Y_MIN));
	  next_target.target.Y = MM_TO_POS( X_MIN);
	}
	#endif
	#ifdef	Y_MAX
	if (next_target.target.Y > MM_TO_POS( Y_MAX)) {
	  printf( "Limiting target.Y (%d) to Y_MAX (%d)\n", next_target.target.Y, MM_TO_POS( Y_MAX));
	  next_target.target.Y = MM_TO_POS( Y_MAX);
	}
	#endif
	#ifdef	Z_MIN
	if (next_target.target.Z < MM_TO_POS( Z_MIN)) {
	  printf( "Limiting target.Z (%d) to Z_MIN (%d)\n", next_target.target.Z, MM_TO_POS( Z_MIN));
	  next_target.target.Z = MM_TO_POS( Z_MIN);
	}
	#endif
	#ifdef	Z_MAX
	if (next_target.target.Z > MM_TO_POS( Z_MAX)) {
	  printf( "Limiting target.Z (%d) to Z_MAX (%d)\n", next_target.target.Z, MM_TO_POS( Z_MAX));
	  next_target.target.Z = MM_TO_POS( Z_MAX);
	}
	#endif

	// The GCode documentation was taken from http://reprap.org/wiki/Gcode .

	if (next_target.seen_T) {
	    //? ==== T: Select Tool ====
	    //?
	    //? Example: T1
	    //?
	    //? Select extruder number 1 to build with.  Extruder numbering starts at 0.

	    next_tool = next_target.T;
	}

	// if we didn't see an axis word, set it to gcode_current_pos. this fixes incorrect moves after homing TODO: fix homing ???
	if (next_target.seen_X == 0)
		next_target.target.X = gcode_current_pos.X;
	if (next_target.seen_Y == 0)
		next_target.target.Y = gcode_current_pos.Y;
	if (next_target.seen_Z == 0)
		next_target.target.Z = gcode_current_pos.Z;
	if (next_target.seen_E == 0)
		next_target.target.E = gcode_current_pos.E;

	if (next_target.seen_G) {
		uint8_t axisSelected = 0;
		switch (next_target.G) {
			// 	G0 - rapid, unsynchronised motion
			// since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
			case 0:
				//? ==== G0: Rapid move ====
				//?
				//? Example: G0 X12
				//?
				//? In this case move rapidly to X = 12 mm.  In fact, the RepRap firmware uses exactly the same code for rapid as it uses for controlled moves (see G1 below), as - for the RepRap machine - this is just as efficient as not doing so.  (The distinction comes from some old machine tools that used to move faster if the axes were not driven in a straight line.  For them G0 allowed any movement in space to get to the destination as fast as possible.)

				backup_f = next_target.target.F;
				next_target.target.F = MAXIMUM_FEEDRATE_X * 2L;
				enqueue_pos( &next_target.target);
				next_target.target.F = backup_f;
				break;

				//	G1 - synchronised motion
			case 1:
				//? ==== G1: Controlled move ====
				//?
				//? Example: G1 X90.6 Y13.8 E22.4
				//?
				//? Go in a straight line from the current (X, Y) point to the point (90.6, 13.8), extruding material as the move happens from the current extruded length to a length of 22.4 mm.
				enqueue_pos( &next_target.target);
				break;

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

				// wait for all moves to complete
				dda_queue_wait();
				// delay
				usleep( 1000* next_target.P);
				break;

				//	G20 - inches as units
			case 20:
				//? ==== G20: Set Units to Inches ====
				//?
				//? Example: G20
				//?
				//? Units from now on are in inches.
				//?
				next_target.option_inches = 1;
				break;

				//	G21 - mm as units
			case 21:
				//? ==== G21: Set Units to Millimeters ====
				//?
				//? Example: G21
				//?
				//? Units from now on are in millimeters.  (This is the RepRap default.)
				//?
				next_target.option_inches = 0;
				break;

				//	G30 - go home via point
			case 30:
				//? ==== G30: Go home via point ====
				//?
				//? Undocumented.
				enqueue_pos( &next_target.target);
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
				if (next_target.seen_X) {
					next_target.target.X = 0;
					axisSelected = 1;
				}
				if (next_target.seen_Y) {
					next_target.target.Y = 0;
					axisSelected = 1;
				}
				if (next_target.seen_Z) {
					next_target.target.Z = 0;
					axisSelected = 1;
				}
				if (axisSelected != 1) {
					next_target.target.X = 0;
					next_target.target.Y = 0;
					next_target.target.Z = 0;
				}
				backup_f = next_target.target.F;
				next_target.target.F = 99999;		// let the software clip this to the maximum allowed rate
				enqueue_pos( &next_target.target);
				next_target.target.F = backup_f;
				break;

			//	G90 - absolute positioning
			case 90:
				//? ==== G90: Set to Absolute Positioning ====
				//?
				//? Example: G90
				//?
				//? All coordinates from now on are absolute relative to the origin of the machine.  (This is the RepRap default.)
				next_target.option_relative = 0;
				break;

				//	G91 - relative positioning
			case 91:
				//? ==== G91: Set to Relative Positioning ====
				//?
				//? Example: G91
				//?
				//? All coordinates from now on are relative to the last position.
				next_target.option_relative = 1;
				break;

				//	G92 - set home
			case 92:
				//? ==== G92: Set Position ====
				//?
				//? Example: G92 X10 E90
				//?
				//? Allows programming of absolute zero point, by reseting the current position to the values specified.  This would set the machine's X coordinate to 10, and the extrude coordinate to 90. No physical motion will occur.

				// wait for queue to empty
				dda_queue_wait();

				if (next_target.seen_X) {
					gcode_current_pos.X = gcode_home_pos.X = next_target.target.X;
					axisSelected = 1;
				}
				if (next_target.seen_Y) {
					gcode_current_pos.Y = gcode_home_pos.Y = next_target.target.Y;
					axisSelected = 1;
				}
				if (next_target.seen_Z) {
					gcode_current_pos.Z = gcode_home_pos.Z = next_target.target.Z;
					axisSelected = 1;
				}
				if (next_target.seen_E) {
					#ifdef	E_ABSOLUTE
						gcode_current_pos.E = gcode_home_pos.E = next_target.target.E;
					#endif
					axisSelected = 1;
				}

				if (axisSelected == 0) {
					gcode_current_pos.X = gcode_home_pos.X = next_target.target.X =
					gcode_current_pos.Y = gcode_home_pos.Y = next_target.target.Y =
					gcode_current_pos.Z = gcode_home_pos.Z = next_target.target.Z = 0;
				}
				break;

			// G161 - Home negative
			case 161:
				//? ==== G161: Home negative ====
				//?
				//? Find the minimum limit of the specified axes by searching for the limit switch.
				if (next_target.seen_X)
					home_x_negative( next_target.target.F);
				if (next_target.seen_Y)
					home_y_negative( next_target.target.F);
				if (next_target.seen_Z)
					home_z_negative( next_target.target.F);
				break;
			// G162 - Home positive
			case 162:
				//? ==== G162: Home positive ====
				//?
				//? Find the maximum limit of the specified axes by searching for the limit switch.
				if (next_target.seen_X)
					home_x_positive( next_target.target.F);
				if (next_target.seen_Y)
					home_y_positive( next_target.target.F);
				if (next_target.seen_Z)
					home_z_positive( next_target.target.F);
				break;

				// unknown gcode: spit an error
			default:
				sersendf_P(PSTR("E: Bad G-code %d"), next_target.G);
				// newline is sent from gcode_parse after we return
				pruss_dump_state();
				return;
		}
		#ifdef	DEBUG
			if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION))
				dda_queue_print();
		#endif
	}
	else if (next_target.seen_M) {
		switch (next_target.M) {
			// M0- machine stop
			case 0:
			// M2- program end
			case 2:
				//? ==== M2: program end ====
				//?
				//? Undocumented.
				dda_queue_wait();
				// no break- we fall through to M112 below
			// M112- immediate stop
			case 112:
				//? ==== M112: Emergency Stop ====
				//?
				//? Example: M112
				//?
				//? Any moves in progress are immediately terminated, then RepRap shuts down.  All motors and heaters are turned off.
				//? It can be started again by pressing the reset button on the master microcontroller.  See also M0.

				timer_stop();
				dda_queue_flush();
				x_disable();
				y_disable();
				z_disable();
				e_disable();
				power_off();
				cli();
				for (;;)
					wd_reset();
				break;

			// M6- tool change
			case 6:
				//? ==== M6: tool change ====
				//?
				//? Undocumented.
				tool = next_tool;
				break;
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
#if 0
			  // FIXME:
				if (temp_achieved() == 0) {
					dda_queue_enqueue( NULL);
				}
#endif
				#ifdef DC_EXTRUDER
					heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
				#elif E_STARTSTOP_STEPS > 0
					do {
						// backup feedrate, move E very quickly then restore feedrate
						backup_f = gcode_current_pos.F;
						gcode_current_pos.F = MAXIMUM_FEEDRATE_E;
						SpecialMoveE( E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
						gcode_current_pos.F = backup_f;
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
						backup_f = gcode_current_pos.F;
						gcode_current_pos.F = MAXIMUM_FEEDRATE_E;
						SpecialMoveE( E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
						gcode_current_pos.F = backup_f;
					} while (0);
				#endif
				break;

			// M104- set temperature
			case 104: {
				//? ==== M104: Set Extruder Temperature (Fast) ====
				//?
				//? Example: M104 S190
				//?
				//? Set the temperature of the current extruder to 190<sup>o</sup>C and return control to the host immediately (''i.e.'' before that temperature has been reached by the extruder).  See also M109.
				//? Teacup supports an optional P parameter as a sensor index to address (eg M104 P1 S100 will set the bed temperature rather than the extruder temperature).
				channel_tag heater;
				if (next_target.seen_P) {
					switch (next_target.P) {
					case 0:  heater = heater_extruder; break;
					case 1:  heater = heater_bed; break;
					default: heater = NULL;
					}
				} else {
					heater = heater_extruder;
				}
				heater_set_setpoint( heater, next_target.S);
				if (next_target.S) {
					// if setpoint is not null, turn power on
					power_on();
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
				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					dda_queue_wait();
				#endif
				if (next_target.seen_P) {
					channel_tag temp_source;
					switch (next_target.P) {
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
					heater_get_celsius( heater_bed, &celsius);
					printf( " B:%1.1lf", celsius);
				}
				break;
			}
			// M7/M106- fan on
			case 7:
			case 106:
				//? ==== M106: Fan On ====
				//?
				//? Example: M106
				//?
				//? Turn on the cooling fan (if any).

				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					dda_queue_wait();
				#endif
				#ifdef HEATER_FAN
					heater_set(HEATER_FAN, 255);
				#endif
				break;
			// M107- fan off
			case 9:
			case 107:
				//? ==== M107: Fan Off ====
				//?
				//? Example: M107
				//?
				//? Turn off the cooling fan (if any).

				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					dda_queue_wait();
				#endif
				#ifdef HEATER_FAN
					heater_set(HEATER_FAN, 0);
				#endif
				break;

			// M109- set temp and wait
			case 109:
				//? ==== M109: Set Extruder Temperature ====
				//?
				//? Example: M109 S190
				//?
				//? Set the temperature of the current extruder to 190<sup>o</sup>C and wait for it to reach that value before sending an acknowledgment to the host.  In fact the RepRap firmware waits a while after the temperature has been reached for the extruder to stabilise - typically about 40 seconds.  This can be changed by a parameter in the firmware configuration file when the firmware is compiled.  See also M104 and M116.
				//?
				//? Teacup supports an optional P parameter as a sensor index to address.
				if (next_target.seen_S) {
					heater_set_setpoint( heater_extruder, next_target.S);
					power_on();
				}
				if (next_target.S) {
					heater_enable( heater_extruder, 1);
				} else {
					heater_enable( heater_extruder, 0);
				}
				dda_queue_enqueue( NULL);
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
				//? ==== M111: Set Debug Level ====
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

				debug_flags = next_target.S;
				printf( "New debug_flags setting: 0x%04x\n", debug_flags);
				break;
			#endif
			// M113- extruder PWM
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
				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					dda_queue_wait();
				#endif
				pruss_dump_position( 0);
				sersendf_P( PSTR( "X:%d,Y:%d,Z:%d,E:%d,F:%d"),
					gcode_current_pos.X, gcode_current_pos.Y, gcode_current_pos.Z, gcode_current_pos.E, gcode_current_pos.F);
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

				printf( "FIRMWARE_NAME: BeBoPr FIRMWARE_URL:n/a PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:%d TEMP_SENSOR_COUNT:%d HEATER_COUNT:%d", 1, 2, 2);
				// newline is sent from gcode_parse after we return
				break;
			// M116 - Wait for all temperatures and other slowly-changing variables to arrive at their set values.
			case 116:
				//? ==== M116: Wait ====
				//?
				//? Example: M116
				//?
				//? Wait for ''all'' temperatures and other slowly-changing variables to arrive at their set values.  See also M109.

				dda_queue_enqueue( NULL);
				break;

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
				if (next_target.seen_S) {
					pid_settings pid;
					channel_tag channel;
					if (next_target.seen_P) {
						switch (next_target.P) {
						case 0:  channel = heater_extruder; break;
						case 1:  channel = heater_bed; break;
						default: channel = NULL;
						}
					} else {
						channel = heater_extruder;
					}
					heater_get_pid_values( channel, &pid);
					switch (next_target.M) {
					case 130:	// M130- heater P factor
						pid.P = next_target.S;
						break;
					case 131:	// M131- heater I factor
						pid.I = next_target.S;
						break;
					case 132:	// M132- heater D factor
						pid.D = next_target.S;
						break;
					case 133:	// M133- heater I limit
						pid.I_limit = next_target.S;
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
				if (next_target.seen_S) {
					channel_tag heater;
					switch (next_target.P) {
					case 0:  heater = heater_extruder; break;
					case 1:  heater = heater_bed; break;
					default: heater = NULL;
					}
					heater_set_raw_pwm( heater, next_target.S);
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
				if (next_target.seen_P) {
					switch (next_target.P) {
					case 0:  heater = heater_extruder; break;
					case 1:  heater = heater_bed; break;
					default: heater = NULL;
					}
				} else {
					heater = heater_extruder;
				}
				heater_get_pid_values( heater, &pid);
				fprintf( stdout, "P:%1.3f I:%1.3f D:%1.3f Ilim:%1.3f",
					   pid.P, pid.I, pid.D, pid.I_limit);
				break;
			}
			#endif

			case 140: //Set heated bed temperature
				//? ==== M140: Set heated bed temperature ====
				//? Undocumented.
				heater_set_setpoint( heater_bed, next_target.S);
				if (next_target.S) {
					power_on();
				}
				break;

			// M190- power on
			case 190:
				//? ==== M190: Power On ====
				//? Undocumented.
				power_on();
				x_enable();
				y_enable();
				z_enable();
				e_enable();
				steptimeout = 0;
				break;
			// M191- power off
			case 191:
				//? ==== M191: Power Off ====
				//? Undocumented.
				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					dda_queue_wait();
				#endif
				x_disable();
				y_disable();
				z_disable();
				e_disable();
				power_off();
				break;

			// M200 - report endstop status
			case 200:
				//? ==== M200: report endstop status ====
				//? Report the current status of the endstops configured in the firmware to the host.
#if ARCH == arm
			  // TODO: implement
#else
                                #if defined(X_MIN_PIN)
				    sersendf_P(PSTR("x_min:%d "), x_min());
                                #endif
                                #if defined(X_MAX_PIN)
				    sersendf_P(PSTR("x_max:%d "), x_max());
                                #endif
                                #if defined(Y_MIN_PIN)
				    sersendf_P(PSTR("y_min:%d "), y_min());
                                #endif
                                #if defined(Y_MAX_PIN)
				    sersendf_P(PSTR("y_max:%d "), y_max());
                                #endif
                                #if defined(Z_MIN_PIN)
				    sersendf_P(PSTR("z_min:%d "), z_min());
                                #endif
                                #if defined(Z_MAX_PIN)
				    sersendf_P(PSTR("z_max:%d "), z_max());
                                #endif
                                #if !(defined(X_MIN_PIN) || defined(X_MAX_PIN) || defined(Y_MIN_PIN) || defined(Y_MAX_PIN) || defined(Z_MIN_PIN) || defined(Z_MAX_PIN))
				    sersendf_P(PSTR("no endstops defined"));
                                #endif
#endif
				break;

			#ifdef	DEBUG
			// M240- echo off
			case 240:
				//? ==== M240: echo off ====
				//? Disable echo.
				//? This command is only available in DEBUG builds.
				debug_flags &= ~DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo off"));
				// newline is sent from gcode_parse after we return
				break;
				// M241- echo on
			case 241:
				//? ==== M241: echo on ====
				//? Enable echo.
				//? This command is only available in DEBUG builds.
				debug_flags |= DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo on"));
				// newline is sent from gcode_parse after we return
				break;

			// DEBUG: return current position, end position, queue
			case 250: {
				//? ==== M250: return current position, end position, queue ====
				//? Undocumented
				//? This command is only available in DEBUG builds.
#if ARCH == arm
				unsigned mb_tail = dda_queue_get_mb_tail();
				sersendf_P( PSTR( "um{X:%d,Y:%d,Z:%d,E:%d,F:%u},steps{X:%d,Y:%d,Z:%d,E:%d,F:%u,c:%u}\t"),
#else
				sersendf_P( PSTR( "um{X:%lq,Y:%lq,Z:%lq,E:%lq,F:%lu},steps{X:%ld,Y:%ld,Z:%ld,E:%ld,F:%lu,c:%lu}\t"),
#endif
					gcode_current_pos.X,
					gcode_current_pos.Y,
					gcode_current_pos.Z,
					gcode_current_pos.E,
					gcode_current_pos.F,
					movebuffer[ mb_tail].endpoint.X,
					movebuffer[ mb_tail].endpoint.Y,
					movebuffer[ mb_tail].endpoint.Z,
					movebuffer[ mb_tail].endpoint.E,
					movebuffer[ mb_tail].endpoint.F,
					#ifdef ACCELERATION_REPRAP
						movebuffer[ mb_tail].end_c
					#else
						movebuffer[ mb_tail].c
					#endif
					);

				dda_queue_print();
				break;
			}

			// DEBUG: read arbitrary memory location
			case 253:
				//? ==== M253: read arbitrary memory location ====
				//? Undocumented
				//? This command is only available in DEBUG builds.
				if (next_target.seen_P == 0)
					next_target.P = 1;
				for (; next_target.P; next_target.P--) {
#if ARCH == arm
				  // TODO: implementation
#else
					serwrite_hex8(*(volatile uint8_t *)(next_target.S));
#endif
					next_target.S++;
				}
				// newline is sent from gcode_parse after we return
				break;

			// DEBUG: write arbitrary memory location
			case 254:
				//? ==== M254: write arbitrary memory location ====
				//? Undocumented
				//? This command is only available in DEBUG builds.
#if ARCH == arm
				  // TODO: implementation
#else
				sersendf_P(PSTR("%x:%x->%x"), next_target.S, *(volatile uint8_t *)(next_target.S), next_target.P);
				(*(volatile uint8_t *)(next_target.S)) = next_target.P;
#endif
				// newline is sent from gcode_parse after we return
				break;
			#endif /* DEBUG */
				// unknown mcode: spit an error
			default:
				sersendf_P(PSTR("E: Bad M-code %d"), next_target.M);
				// newline is sent from gcode_parse after we return
		} // switch (next_target.M)
	} // else if (next_target.seen_M)
} // process_gcode_command()

void gcode_process_init( void)
{
  traject_init();

  heater_extruder = heater_lookup_by_name( "heater_extruder");
  heater_bed      = heater_lookup_by_name( "heater_bed");
  temp_extruder   = temp_lookup_by_name( "temp_extruder");
  temp_bed        = temp_lookup_by_name( "temp_bed");
  if (debug_flags & DEBUG_GCODE_PROCESS) {
    printf( "tag_name( heater_extruder) = '%s',  tag_name( heater_bed) = '%s',\n"
	    "tag_name( temp_extruder) = '%s',  tag_name( temp_bed) = '%s'\n",
	    tag_name( heater_extruder), tag_name( heater_bed),
	    tag_name( temp_extruder), tag_name( temp_bed));
  }
}
