/** \file
	\brief Axis homing routines
*/

// 20120401 modmaker - First release.

#include	<sys/types.h>
#include	<sys/stat.h>
#include	<fcntl.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>
#include	<unistd.h>
#include	<string.h>
#include	<poll.h>
#include	<pthread.h>

#include	"pruss.h"
#include	"limit_switches.h"
#include	"home.h"
#include	"dda_queue.h"
#include	"pinio.h"
#ifdef DEBUG
#include	"sersendf.h"
#endif
#include	"memory_barrier.h"
#include	"debug.h"
#include	"gcode_process.h"
#include	"traject.h"
#include	"bebopr.h"


// Sanity checks for config.h settings:
#if !defined( X_DIR_PIN) || !defined( Y_DIR_PIN) || !defined( Z_DIR_PIN)
#error "Need direction signals for homing operation"
#endif


static const double fclk = 200000000.0;

static int limsw_axis( axis_e axis, int reverse)
{
  switch (axis) {
  case x_axis:	return (reverse) ? limsw_x_min() : limsw_x_max();
  case y_axis:	return (reverse) ? limsw_y_min() : limsw_y_max();
  case z_axis:	return (reverse) ? limsw_z_min() : limsw_z_max();
  default:	return -1;
  }
}

extern double traject_get_step_size( axis_e axis);

// Execute the actual homing operation. The hardware selected with the 'axis'
// variable must exist or we'll fail miserably, so filter before calling here!
static void run_home_one_axis( axis_e axis, int reverse, uint32_t feed)
{
  int direction = (reverse) ? -1 : 1;
  int steps = 0;
  const char axisNames[] = { '?', 'X', 'Y', 'Z' };
  // FIXME: remove code duplication from gcode_process.c !!!
  double step_size;
  int pruss_axis = 0;
  step_size = traject_get_step_size( axis);
  switch (axis) {
  case x_axis:
    pruss_axis = 1;
    break;
  case y_axis:
    pruss_axis = 2;
    break;
  case z_axis:
    pruss_axis = 3;
    break;
  default:
    fprintf( stderr, "BUG: trying to home illegal axis (%d)!\n", axis);
    pruss_axis = 0;
  }
  double ds = step_size;	/* delta per interation */
  const double c_acc = 191201673.632;	// 0.676 * fclk * sqrt( 2.0)

  if (feed > 3000) {
    feed = 3000;
  }
  feed = 60000;

  double dv = (double)feed / (60 * 1000);	/* [mm/min] -> [m/s] */
  double a = 1.5;		/* [m/s^s] */
  double a_max = (dv * dv) / (2.0 * ds);
  if (a > a_max) {
    a = a_max;
  }
  uint32_t c0;
  uint32_t cmin = fclk * step_size / dv ;
  double ramp_length = (dv * dv) / (2.0 * a);

  c0 = (uint32_t) (c_acc * sqrt( step_size / a));
  ds = step_size;
  if (direction < 0) {
    ds = -ds;
    dv = -dv;
    ramp_length = -ramp_length;
  }
  pruss_queue_set_origin( pruss_axis);
  pruss_queue_set_accel( pruss_axis, c0);
  printf( "  %c: ramp to speed %1.3lf [mm/s] in steps of %1.6lf [mm] with a=%1.3lf [m/s^2] over s=%1.6lf [mm], c0=%d, cmin=%d\n",
	  axisNames[ pruss_axis], 1000 * dv, 1000 * ds, a, 1000 * ramp_length, c0, cmin);
  int ramp_up = 1;
  steps = 0;
  while (limsw_axis( axis, reverse) == 0) {
    if (ramp_up) {
      pruss_queue_accel_more( pruss_axis, cmin, (int32_t)(1.0E9 * ds));
      ramp_length -= ds;
      if ((direction < 0 && ramp_length >= 0) || (direction >= 0 && ramp_length < 0)) {
	ramp_up = 0;
	printf( "  %c: dwelling to limitswitch in steps of %1.6lf [mm] at speed %1.3lf [mm/s], cmin=%d\n",
		axisNames[ pruss_axis], 1000 * ds, 1000 * dv, cmin);
      }
    } else {
      pruss_queue_dwell( pruss_axis, cmin, (int32_t)(1.0E9 * ds));
    }
    pruss_queue_execute();
    ++steps;
    sched_yield();
  }
#if 0
  // TODO: calculate deceleration ???
  printf( "  %c: limit switch reached in %d iterations, ramp down from speed %1.3lf [m/s] with a=-%1.3lf [m/s^2] over s=%1.6lf [m]\n",
 	  axisNames[ pruss_axis], steps, vx, ax, dx);
  pruss_queue_decel( pruss_axis, (int32_t)(1.0E9 * dx));
  pruss_queue_execute();
#else
  printf( "  %c: limit switch reached in %d iterations\n", axisNames[ pruss_axis], steps);
#endif
  printf( "  %c: pausing for switch debounce\n", axisNames[ pruss_axis]);
  sleep( 1);
  pruss_dump_position( pruss_axis);
  // Reverse direction
  direction = -direction;
  dv = -0.1 * dv;
  cmin = 10 * cmin;
  ds = (direction < 0) ? -step_size : step_size;
  a_max = (dv * dv) / fabs(2.0 * ds);
  if (a > a_max) {
    a = a_max;
  }
  c0 = (uint32_t) (c_acc * sqrt( step_size / a));
  ramp_length = (direction < 0) ? -(dv * dv) / (2.0 * a) : (dv * dv) / (2.0 * a);
  pruss_queue_set_accel( pruss_axis, c0);	// FIXME: uses old accel constant !
  steps = 0;
  ramp_up = 1;
  printf( "  %c: ramp to speed %1.3lf [mm/s] in steps of %1.6lf [mm] with a=%1.3lf [m/s^2] over s=%1.6lf [mm], c0=%d\n",
	  axisNames[ pruss_axis], 1000 * dv, 1000 * ds, a, 1000 * ramp_length, c0);
  while (limsw_axis( axis, (direction < 0) ? 0 : 1)) {
    if (ramp_up) {
      pruss_queue_accel_more( pruss_axis, cmin, (int32_t)(1.0E9 * ds));
      ramp_length -= ds;
      if ((direction < 0 && ramp_length >= 0) || (direction >= 0 && ramp_length < 0)) {
	ramp_up = 0;
	printf( "  %c: dwelling from limitswitch in steps of %1.6lf [mm] at speed %1.3lf [mm/s]\n",
		axisNames[ pruss_axis], 1000 * ds, 1000 * dv);
      }
    } else {
      pruss_queue_dwell( pruss_axis, cmin, (int32_t)(1.0E9 * ds));
    }
    pruss_queue_execute();
    sched_yield();
    ++steps;
  }
  printf( "  %c: limit switch was released after %d iterations\n", axisNames[ pruss_axis], steps);
  sleep( 1);
  pruss_dump_position( pruss_axis);
  pruss_queue_set_origin( pruss_axis);
}

/// home the selected axis to the selected limit switch.
// keep all preprocessor configuration stuff at or below this level.
static void home_one_axis( axis_e axis, int reverse, uint32_t feed)
{
#if 0
  //FIXME: needed ???
	// get ready for the action
	power_on();
	dda_queue_wait();
#endif
	// move to a limit switch or sensor
	run_home_one_axis( axis, reverse, feed);

}

/// find X MIN endstop
void home_x_negative( uint32_t feed) {
  if (limsw_x_has_min()) {
    uint32_t max_feed = traject_get_max_feed( x_axis);
    if (feed > max_feed) {
      feed = max_feed;
      fprintf( stderr, "Limiting feed on x axis to %d\n", feed);
    }
    home_one_axis( x_axis, 1, feed);
    // reference 'home' position to current position
    gcode_set_pos( 'X', MM_TO_POS( X_MIN));
  } else {
    // reference current position as 'home'
    gcode_set_pos( 'X', 0);
  }
}

/// find X_MAX endstop
void home_x_positive( uint32_t feed) {
  if (limsw_x_has_max()) {
    uint32_t max_feed = traject_get_max_feed( x_axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( x_axis, 0, feed);
    // reference 'home' position to current position
    gcode_set_pos( 'X', MM_TO_POS( X_MAX));
  } else {
    // reference current position as 'home'
    gcode_set_pos( 'X', 0);
  }
}

/// find Y MIN endstop
void home_y_negative( uint32_t feed) {
  if (limsw_y_has_min()) {
    uint32_t max_feed = traject_get_max_feed( y_axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( y_axis, 1, feed);
    // reference 'home' position to current position
    gcode_set_pos( 'Y', MM_TO_POS( Y_MIN));
  } else {
    // reference current position as 'home'
    gcode_set_pos( 'Y', 0);
  }
}

/// find Y MAX endstop
void home_y_positive( uint32_t feed) {
  if (limsw_y_has_max()) {
    uint32_t max_feed = traject_get_max_feed( y_axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( y_axis, 0, feed);
    // reference 'home' position to current position
    gcode_set_pos( 'Y', MM_TO_POS( Y_MAX));
  } else {
    // reference current position as 'home'
    gcode_set_pos( 'Y', 0);
  }
}

/// find Z MIN endstop
void home_z_negative( uint32_t feed) {
  if (limsw_z_has_min()) {
    uint32_t max_feed = traject_get_max_feed( z_axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( z_axis, 1, feed);
    // reference 'home' position to current position
    gcode_set_pos( 'Z', MM_TO_POS( Z_MIN));
  } else {
    // reference current position as 'home'
    gcode_set_pos( 'Z', 0);
  }
}

/// find Z MAX endstop
void home_z_positive( uint32_t feed) {
  if (limsw_z_has_max()) {
    uint32_t max_feed = traject_get_max_feed( z_axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( z_axis, 0, feed);
    // reference 'home' position to current position
    gcode_set_pos( 'Z', MM_TO_POS( Z_MAX));
  } else {
    // reference current position as 'home'
    gcode_set_pos( 'Z', 0);
  }
}
