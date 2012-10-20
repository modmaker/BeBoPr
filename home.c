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
static const char axisNames[] = { '?', 'X', 'Y', 'Z', 'E' };

// new_state is 1 when running towards the switch, 0 when running away from the switch!
static int step_until_switch_change( axis_e axis, int reverse, int new_state, int pruss_axis,
					uint32_t c0, uint32_t cmin, int direction, int32_t* position)
{
  /* Clear internal position information */
  {
    int16_t virtPosT;
    int16_t virtPosN;
    int32_t virtPosI;
    int16_t virtPosT_new;
    int32_t virtPosI_new;
    uint8_t mask;
    uint8_t invert;
    int     gpiobit;
    double  delta;

    switch (axis) {
    case x_axis: gpiobit = (reverse) ? XMIN_GPIO : XMAX_GPIO; break;
    case y_axis: gpiobit = (reverse) ? YMIN_GPIO : YMAX_GPIO; break;
    case z_axis: gpiobit = (reverse) ? ZMIN_GPIO : ZMAX_GPIO; break;
    default: return 0;
    }
    mask = 1 << ((gpiobit % 16) - 8);	// bit magic, map gpio bit to PRUSS positions
    if ( ( reverse && config_min_limit_switch_is_active_low( axis)) ||
         (!reverse && config_max_limit_switch_is_active_low( axis)) ) {
      invert = mask;
    } else {
      invert = 0;
    }
    pruss_get_positions( pruss_axis, &virtPosI, &virtPosT, &virtPosN, 0);
    if (DEBUG_HOME && (debug_flags & DEBUG_HOME)) {
      printf( "  Home: axis %d, starting at virtPos= %d+%d/%d\n",
              pruss_axis, virtPosI, virtPosT, virtPosN);
    }
    delta = (new_state) ? 0.500 : 0.010;
    pruss_queue_accel( pruss_axis, c0, cmin, *position + direction * SI2POS( delta));
    pruss_queue_exec_limited( mask, (new_state) ? invert : ~invert);

    traject_wait_for_completion();
    pruss_get_positions( pruss_axis, &virtPosI_new, &virtPosT_new, 0, 0);
    if (DEBUG_HOME && (debug_flags & DEBUG_HOME)) {
      printf( "  Home: axis %d, ended at virtPos= %d+%d/%d\n",
              pruss_axis, virtPosI_new, virtPosT_new, virtPosN);
    }
   /*
    *  For a move terminated by a limitswitch state change, part of the
    *  internal position information is now wrong. Fix this now.
    */
    pruss_queue_set_position( pruss_axis, virtPosI_new);
    int32_t delta_pos = virtPosI_new - virtPosI + (virtPosT_new - virtPosT + virtPosN / 2) / virtPosN;
    if (DEBUG_HOME && (debug_flags & DEBUG_HOME)) {
      printf( "  %c: limit switch %s detected after %1.6lf [mm]\n",
	      axisNames[ pruss_axis], (new_state) ? "activation" : "release", POS2MM( delta_pos));
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
  /*
   * Calculate settings for acceleration at 25% of maximum
   */
  si_speed = feed / 60000.0;
  cmin = fclk * si_step_size / si_speed ;
  c0   = (uint32_t) (c_acc * sqrt( si_step_size / (0.25 * a_max)));
  /*
   * If we can start running faster than the target speed, no acceleration is needed
   */
  if (c0 < cmin) {
    c0 = cmin;
  }
  /*
   * Run towards the switch, reversing direction
   */
  int32_t old_position = *position;
  if (!step_until_switch_change( axis, reverse, 1, pruss_axis, c0, cmin, direction, position)) {
    return 0;
  }
  si_speed = config_get_home_release_feed( axis) / 60000.0;
  c0   = (uint32_t) (c_acc * sqrt( si_step_size / (0.25 * a_max)));
  cmin = fclk * si_step_size / si_speed ;
  if (c0 < cmin) {
    c0 = cmin;
  }
  direction = -direction;
  /*
   * Run away from the switch
   */
  if (!step_until_switch_change( axis, reverse, 0, pruss_axis, c0, cmin, direction, position)) {
    return 0;
  }
  int32_t new_position = *position;
  fprintf( stderr, "Home operation on %c-axis resulted in netto move of %1.6lf [mm]\n",
	  axisNames[ pruss_axis], POS2MM( new_position - old_position));
  return 1;
}

/// home the selected axis to the selected limit switch.
static void home_one_axis( axis_e axis, int reverse, int32_t* position, uint32_t feed)
{
  traject_wait_for_completion();
  // move to a limit switch or sensor
  run_home_one_axis( axis, reverse, position, feed);
}

/// Position at a switch or sensor, if that switch is present. If not, keep
/// the current position and do not move. This can be used for manually
/// assigning a home position.
/// If the switch is configured as home / reference, set the current position
/// from the reference value. Otherwise the current position is not changed.
void home_axis_to_min_limit_switch( axis_e axis, int32_t* position, uint32_t feed)
{
  if (config_axis_has_min_limit_switch( axis)) {
    uint32_t max_feed = (uint32_t) config_get_max_feed( axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( axis, 1 /* reverse */, position, feed);
  }
}

void home_axis_to_max_limit_switch( axis_e axis, int32_t* position, uint32_t feed)
{
  if (config_axis_has_max_limit_switch( axis)) {
    uint32_t max_feed = (uint32_t) config_get_max_feed( axis);
    if (feed > max_feed) {
      feed = max_feed;
    }
    home_one_axis( axis, 0 /* forward */, position, feed);
  }
}
