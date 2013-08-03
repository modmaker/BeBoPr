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


static const double fclk = PRUSS_CLOCK;

// new_state is 1 when running towards the switch, 0 when running away from the switch!
static int step_until_switch_change( axis_e axis, int reverse, int new_state, int pruss_axis,
				uint32_t c0, uint32_t cmin, int direction, int32_t* position, int32_t ramp)
{
  /* Clear internal position information */
  {
    int32_t virtPosI;
    int32_t virtPosI_new;
    uint8_t mask;
    uint8_t invert;
    int     gpiobit;
    int32_t delta;

    switch (axis) {
    case x_axis: gpiobit = (reverse) ? XMIN_GPIO : XMAX_GPIO; break;
    case y_axis: gpiobit = (reverse) ? YMIN_GPIO : YMAX_GPIO; break;
    case z_axis: gpiobit = (reverse) ? ZMIN_GPIO : ZMAX_GPIO; break;
    default: return 0;
    }
#ifdef BONE_BRIDGE
    mask = (gpiobit < 32) ? (1 << ((gpiobit % 8) + 4)) : (1 << ((gpiobit % 8)));
#else
    mask = 1 << ((gpiobit % 16) - 8);	// bit magic, map gpio bit to PRUSS positions
#endif
    if ( ( reverse && config_min_limit_switch_is_active_low( axis)) ||
         (!reverse && config_max_limit_switch_is_active_low( axis)) ) {
      invert = mask;
    } else {
      invert = 0;
    }
    pruss_get_positions( pruss_axis, &virtPosI, NULL);
    if (DEBUG_HOME && (debug_flags & DEBUG_HOME)) {
      printf( "  Home: axis %d, starting at virtPos= %d\n", pruss_axis, virtPosI);
    }
    delta = SI2POS( (new_state) ? 0.500 : 0.010);
    if (delta > ramp) {
      delta -= ramp;
    } else {
      delta = 0;
    }
    /*
     * Start a ramp followed by a dwell that will be terminated by a limitswitch state change.
     */
    incMoveNr( pruss_axis);
    pruss_queue_accel( pruss_axis, 0, c0, cmin, *position + direction * ramp);
    pruss_queue_exec_limited( mask, (new_state) ? invert : ~invert);
    incMoveNr( pruss_axis);
    pruss_queue_dwell( pruss_axis, cmin, *position + direction * (ramp + delta));
    pruss_queue_exec_limited( mask, (new_state) ? invert : ~invert);
    pruss_wait_for_completion();

    pruss_get_positions( pruss_axis, &virtPosI_new, NULL);
    if (DEBUG_HOME && (debug_flags & DEBUG_HOME)) {
      printf( "  Home: axis %d, ended at virtPos= %d\n",
              pruss_axis, virtPosI_new);
    }
    int32_t delta_pos = virtPosI_new - virtPosI;
    if (DEBUG_HOME && (debug_flags & DEBUG_HOME)) {
      printf( "  %c: limit switch %s detected after %1.6lf [mm]\n",
	      axisName( axis), (new_state) ? "activation" : "release", POS2MM( delta_pos));
    }
    *position += delta_pos;
  }
  usleep( 500 * 1000);	// debounce electrical and mechanical
  return 1;
}

// Execute the actual homing operation. The hardware selected with the 'axis'
// variable must exist or we'll fail miserably, so filter before calling here!
// reverse determines the direction: towards minimum (1) or maximum switch (0)
static int run_home_one_axis( axis_e axis, int reverse, int32_t* position, uint32_t feed)
{
  int direction = (reverse) ? -1 : 1;	// absolute direction of current move
  int pruss_axis = 0;
  double si_step_size = config_get_step_size( axis);
  double a_max = config_get_max_accel( axis);
  const double c_acc = 282842712.5;	// = fclk * sqrt( 2.0);
  uint32_t c0;
  uint32_t cmin;
  double si_speed;
  uint32_t ramp;

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
      printf( "  %c: limiting home speed to %d\n", axisName( axis), feed);
    }
  }
  /*
   * Calculate settings for acceleration at 25% of maximum
   */
  double a = 0.25 * a_max;
  si_speed = feed / 60000.0;
  cmin = fclk * si_step_size / si_speed ;
  c0   = (uint32_t) (c_acc * sqrt( si_step_size / a));
  ramp = SI2POS( si_speed * si_speed / ( 2 * a));
  /*
   * If we can start running faster than the target speed, no acceleration is needed
   */
  if (c0 < cmin) {
    c0 = cmin;
    ramp = 0;
  }
  /*
   * Run towards the switch, reversing direction
   */
  int32_t old_position = *position;
  if (!step_until_switch_change( axis, reverse, 1, pruss_axis, c0, cmin, direction, position, ramp)) {
    return 0;
  }
  si_speed = config_get_home_release_feed( axis) / 60000.0;
  c0   = (uint32_t) (c_acc * sqrt( si_step_size / a));
  cmin = fclk * si_step_size / si_speed ;
  ramp = SI2POS( si_speed * si_speed / ( 2 * a));
  if (c0 < cmin) {
    c0 = cmin;
    ramp = 0;
  }
  direction = -direction;
  /*
   * Run away from the switch
   */
  if (!step_until_switch_change( axis, reverse, 0, pruss_axis, c0, cmin, direction, position, ramp)) {
    return 0;
  }
  int32_t new_position = *position;
  fprintf( stderr, "Home operation on %c-axis resulted in netto move of %1.6lf [mm]\n",
	  axisName( axis), POS2MM( new_position - old_position));
  return 1;
}

/// home the selected axis to the selected limit switch.
static void home_one_axis( axis_e axis, int reverse, int32_t* position, uint32_t feed)
{
  pruss_wait_for_completion();
  // move to a limit switch or sensor
  run_home_one_axis( axis, reverse, position, feed);
}

/// Position at a switch or sensor, if that switch is present. If not, keep
/// the current position and do not move. This can be used for manually
/// assigning a home position.
/// If the switch is configured as home / reference, set the current position
/// from the reference value. Otherwise the current position is not changed.
static void home_axis_to_min_limit_switch( axis_e axis, int32_t* position, uint32_t feed)
{
  if (config_axis_has_min_limit_switch( axis)) {
    uint32_t max_feed = (uint32_t) config_get_max_feed( axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( axis, 1 /* reverse */, position, feed);
  }
}

static void home_axis_to_max_limit_switch( axis_e axis, int32_t* position, uint32_t feed)
{
  if (config_axis_has_max_limit_switch( axis)) {
    uint32_t max_feed = (uint32_t) config_get_max_feed( axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( axis, 0 /* forward */, position, feed);
  }
}

void home_axis_to_limit_switch( axis_e axis, int32_t* position, uint32_t feed, int reverse)
{
  if (reverse) {
    home_axis_to_min_limit_switch( axis, position, feed);
  } else {
    home_axis_to_max_limit_switch( axis, position, feed);
  }
}
