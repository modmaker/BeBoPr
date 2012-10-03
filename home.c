/** \file
	\brief Axis homing routines
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

#include "beaglebone.h"
#include "home.h"
#include "limit_switches.h"
#include "pruss_stepper.h"
#include "gcode_process.h"
#include "traject.h"
#include "bebopr.h"
#include "debug.h"


static const double fclk = TIMER_CLOCK;

/*
 * Determine the state of the limit switch that's in the direction we're moving to.
 */
static int limsw_axis( axis_e axis, int reverse)
{
  switch (axis) {
  case x_axis:	return (reverse) ? limsw_min( x_axis) : limsw_max( x_axis);
  case y_axis:	return (reverse) ? limsw_min( y_axis) : limsw_max( y_axis);
  case z_axis:	return (reverse) ? limsw_min( z_axis) : limsw_max( z_axis);
  default:	return -1;
  }
}

// Execute the actual homing operation. The hardware selected with the 'axis'
// variable must exist or we'll fail miserably, so filter before calling here!
static void run_home_one_axis( axis_e axis, int reverse, uint32_t feed)
{
  int direction = (reverse) ? -1 : 1;
  int steps;
  const char axisNames[] = { '?', 'X', 'Y', 'Z' };

  int pruss_axis = 0;
  double step_size = config_get_step_size( axis);
  switch (axis) {
  case x_axis: pruss_axis = 1; break;
  case y_axis: pruss_axis = 2; break;
  case z_axis: pruss_axis = 3; break;
  default:
    fprintf( stderr, "BUG: trying to home illegal axis (%d)!\n", axis);
    pruss_axis = 0;
  }
  if (feed > config_get_home_max_feed( axis)) {
    feed = config_get_home_max_feed( axis);
    if (debug_flags & DEBUG_HOME) {
      printf( "  %c: limiting home speed to %d\n", axisNames[ pruss_axis], feed);
    }
  }
  const double c_acc = 282842712.5;	// = fclk * sqrt( 2.0);
  double speed = feed / 60000.0;
  uint32_t cmin = fclk * step_size / speed ;
  uint32_t c0 = (uint32_t) (c_acc * sqrt( step_size / config_get_max_accel( axis)));

  pruss_queue_set_origin( pruss_axis);
  /*
   * Run towards the switch
   */
  pruss_queue_set_accel( pruss_axis, c0);
  steps = 0;
  while (limsw_axis( axis, reverse) == 0) {
    pruss_queue_dwell( pruss_axis, cmin, (int32_t)(direction * 1.0E9 * step_size));
    pruss_queue_execute();
    sched_yield();
    ++steps;
  }
  /*
   * Debounce and reverse direction
   */
  if (debug_flags & DEBUG_HOME) {
    printf( "  %c: limit switch reached in %d iterations\n",
	    axisNames[ pruss_axis], steps);
    printf( "  %c: pausing for switch debounce\n", axisNames[ pruss_axis]);
  }
  sleep( 1);
  if (debug_flags & DEBUG_HOME) {
    pruss_dump_position( pruss_axis);
  }
  feed = config_get_home_release_feed( axis);
  speed = feed / 60000.0;
  cmin = fclk * step_size / speed ;
  c0 = cmin;
  direction = -direction;
  /*
   * Run away from the switch
   */
  pruss_queue_set_origin( pruss_axis);
  pruss_queue_set_accel( pruss_axis, c0);
  steps = 0;
  while (limsw_axis( axis, reverse) != 0) {
    pruss_queue_dwell( pruss_axis, cmin, (int32_t)(direction * 1.0E9 * step_size));
    pruss_queue_execute();
    sched_yield();
    ++steps;
  }
  if (debug_flags & DEBUG_HOME) {
    printf( "  %c: limit switch was released after %d iterations\n",
	    axisNames[ pruss_axis], steps);
    pruss_dump_position( pruss_axis);
  }
  pruss_queue_set_origin( pruss_axis);
}

/// home the selected axis to the selected limit switch.
// keep all preprocessor configuration stuff at or below this level.
static void home_one_axis( axis_e axis, int reverse, uint32_t feed)
{
  traject_wait_for_completion();

  struct sched_param old_param;
  int old_scheduler;
  pthread_t self = pthread_self();
  pthread_getschedparam( self, &old_scheduler, &old_param);
  struct sched_param new_param = {
    .sched_priority = HOME_PRIO
  };
  // elevate priority for undisturbed operation
  pthread_setschedparam( self, HOME_SCHED, &new_param); 
  // move to a limit switch or sensor
  run_home_one_axis( axis, reverse, feed);
  // return to normal scheduling
  pthread_setschedparam( self, old_scheduler, &old_param); 
}

/// Position at a switch or sensor, if that switch is present. If not, keep
/// the current position and do not move. This can be used for manually
/// assigning a home position.
/// If the switch is configured as home / reference, set the current position
/// from the reference value. Otherwise the current position is not changed.
void home_axis_to_min_limit_switch( axis_e axis, double feed)
{
  if (config_axis_has_min_limit_switch( axis)) {
    double max_feed = config_get_max_feed( axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( axis, 1 /* reverse */, feed);
  }
}

void home_axis_to_max_limit_switch( axis_e axis, double feed)
{
  if (config_axis_has_max_limit_switch( axis)) {
    double max_feed = config_get_max_feed( axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( axis, 0 /* forward */, feed);
  }
}
