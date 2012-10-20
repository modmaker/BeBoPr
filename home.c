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

#define PRUSS_HOMING	1


static const double fclk = TIMER_CLOCK;
static const char axisNames[] = { '?', 'X', 'Y', 'Z' };

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

static inline void update_position( int32_t* position, int direction, int count, double step_size)
{
  *position += SI2POS( (direction * count) * step_size);
}

// new_state is 1 when running towards the switch, 0 when running away from the switch!
static inline int step_until_switch_change( axis_e axis, int reverse, int new_state, double si_delta,
					int pruss_axis, uint32_t c0, uint32_t cmin, int direction,
					int32_t* position, int32_t pos_delta, const char* dir_txt)
{
#if PRUSS_HOMING
  /* Clear internal position information */
  {
    int16_t virtPosT;
    int16_t virtPosN;
    int32_t virtPosI;
    int16_t virtPosT_new;
    int32_t virtPosI_new;
    uint8_t mask;
    uint8_t invert;
    int gpiobit;
    double delta;

    switch (axis) {
    case x_axis: gpiobit = (reverse) ? XMIN_GPIO : XMAX_GPIO; break;
    case y_axis: gpiobit = (reverse) ? YMIN_GPIO : YMAX_GPIO; break;
    case z_axis: gpiobit = (reverse) ? ZMIN_GPIO : ZMAX_GPIO; break;
    default: return 0;
    }
    mask = 1 << ((gpiobit % 16) - 8);	// bit magic
    if ( ( reverse && config_min_limit_switch_is_active_low( axis)) ||
         (!reverse && config_max_limit_switch_is_active_low( axis)) ) {
      invert = mask;
    } else {
      invert = 0;
    }
//  printf( "  Home: axis %d, mask= %02x, invert= %02x, gpiobit= %d\n",
//          pruss_axis, mask, invert, gpiobit);
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
//  pruss_stepper_dump_state();
   /*
    *  For a move terminated by a limitswitch state change, part of the
    *  internal position information is now wrong.
    */
    pruss_queue_set_position( pruss_axis, virtPosI_new);	// fix requestedPos
    int32_t delta_pos = virtPosI_new - virtPosI + (virtPosT_new - virtPosT + virtPosN / 2) / virtPosN;
    if (DEBUG_HOME && (debug_flags & DEBUG_HOME)) {
      printf( "  %c: limit switch %s detected after %1.6lf [mm]\n",
	      axisNames[ pruss_axis], (new_state) ? "activation" : "release", POS2MM( delta_pos));
    }
    *position += (virtPosI_new - virtPosI);
  }
#else
  int iter = 0;			/* count interations for exact position */
  uint32_t pos_move_len = 0.0;	/* for soft limit */
  uint32_t pos_max_move = 0;	// FIXME: needs proper initialization to work !

  while (limsw_axis( axis, reverse) != new_state) {
    pruss_queue_dwell( pruss_axis, cmin, *position + pos_delta);
    *position += pos_delta;
    pos_move_len += pos_delta;
    /* Do not queue moves to prevent stepping after reaching position */
    while (!pruss_queue_empty()) {
      sched_yield();
    }
    pruss_queue_execute();
    ++iter;
    if (pos_max_move > 0.0 && pos_move_len > pos_max_move) {
      printf( "  ERROR: %c - aborting home%s after %d iterations / %1.6lf [mm].\n",
	      axisNames[ pruss_axis], dir_txt, iter, POS2MM( pos_move_len));
      return 0;
    }
    sched_yield();
  }
  if (DEBUG_HOME && (debug_flags & DEBUG_HOME)) {
    printf( "  %c: limit switch state change detected after %d iterations, %1.6lf [mm]\n",
	    axisNames[ pruss_axis], iter, (double)(direction * iter) * SI2MM( si_delta));
  }
#endif
  usleep( 500 * 1000);
  return 1;
}

// Execute the actual homing operation. The hardware selected with the 'axis'
// variable must exist or we'll fail miserably, so filter before calling here!
// reverse determines the direction: towards minimum (1) or maximum switch (0)
static int run_home_one_axis( axis_e axis, int reverse, int32_t* position, uint32_t feed)
{
  int direction = (reverse) ? -1 : 1;
  int pruss_axis = 0;
  double si_step_size = config_get_step_size( axis);
  double si_iter_size = si_step_size;
  double a_max = config_get_max_accel( axis);
  const double c_acc = 282842712.5;	// = fclk * sqrt( 2.0);

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
  double speed = feed / 60000.0;
  /*
   * Note: we're running at fixed speed, no acceleration is done
   * We assume that the homing feed is low enough so that we can
   * start stepping at that speed without acceleration.
   * FIXME: This has been proven wrong!!! Even starting at relatively
   * low speed without acceleration will give step loss!
   */
  uint32_t cmin = fclk * si_step_size / speed ;
  uint32_t c0   = (uint32_t) (c_acc * sqrt( si_step_size / (0.25 * a_max)));
  if (cmin > c0) {
    cmin = c0;
  }
  /*
   * Run towards the switch
   */
  int32_t pos_delta = (int32_t)(direction * SI2POS( si_iter_size));
  int32_t old_position = *position;
  if (!step_until_switch_change( axis, reverse, 1, si_iter_size, pruss_axis,
				  c0, cmin, direction, position, pos_delta, " move")) {
    return 0;
  }
  if (debug_flags & DEBUG_HOME) {
    printf( "  %c: pausing for switch debounce\n", axisNames[ pruss_axis]);
  }
  speed = config_get_home_release_feed( axis) / 60000.0;
  cmin = fclk * si_iter_size / speed ;
  c0   = (uint32_t) (c_acc * sqrt( si_step_size / (0.25 * a_max)));
  direction = -direction;
  pos_delta = (int32_t)(direction * SI2POS( si_iter_size));
  /*
   * Run away from the switch
   */
  if (!step_until_switch_change( axis, reverse, 0, si_iter_size, pruss_axis,
				  c0, cmin, direction, position, pos_delta, " release")) {
    return 0;
  }
  int32_t new_position = *position;
  if (DEBUG_HOME && (debug_flags & DEBUG_HOME)) {
    printf( "  position delta= %1.6lf [mm]\n", POS2MM( new_position - old_position));
  }
  return 1;
}

/// home the selected axis to the selected limit switch.
// keep all preprocessor configuration stuff at or below this level.
static void home_one_axis( axis_e axis, int reverse, int32_t* position, uint32_t feed)
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
  run_home_one_axis( axis, reverse, position, feed);
  // return to normal scheduling
  pthread_setschedparam( self, old_scheduler, &old_param); 
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
