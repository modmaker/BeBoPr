
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <time.h>

#include "bebopr.h"
#include "traject.h"
#include "pruss_stepper.h"
#include "debug.h"
#include "beaglebone.h"
#include "mendel.h"

/*
 *  Settings that are changed during initialization.
 *  Silly defaults to prevent division-by-zero or similar
 *  while not initialized (TODO: remove)
 */
static double step_size_x;	/* [m] */
static double step_size_y;
static double step_size_z;
static double step_size_e;

static double recipr_a_max_x;	/* [s^2/m] */
static double recipr_a_max_y;
static double recipr_a_max_z;
static double recipr_a_max_e;

static double vx_max;		/* [m/s] */
static double vy_max;
static double vz_max;
static double ve_max;

static const double fclk = 200000000.0;
static const double c_acc = 282842712.5;	// = fclk * sqrt( 2.0);

static inline int queue_move( const char* axis_name, double ramp, double a, double v, double dwell, uint32_t c0, uint32_t cmin)
{
  if (v != 0.0) {
    char aname = *axis_name;
    if (islower( aname)) {
      aname = toupper( aname);
    }
    int axis = (aname < 'X') ? aname - 'E' + 4 : aname - 'X' + 1;
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      if (c0 != cmin) {
        printf( "Queue %c: ramping to and from %1.3lf [mm/s] "
		"with a=%1.3lf [m/s^2] over %1.6lf [mm] (c0=%u,cmin=%u)\n",
		aname, SI2MM( v), a, SI2MM( ramp + dwell + ramp), c0, cmin);
      } else {
        printf( "Queue %c: running at %1.3lf [mm/s] "
		"over %1.6lf [mm] (c0=%u,cmin=%u)\n",
		aname, SI2MM( v), SI2MM( ramp + dwell + ramp), c0, cmin);
      }
    }
    pruss_queue_accel( axis, c0, cmin, (int32_t)(1.0E9 * (ramp + dwell)));
    return 1;
  }
  return 0;
}

#define QUEUE_MOVE( axis) queue_move( #axis, ramp_d##axis, a##axis, v##axis, dwell_d##axis, c0##axis, cmin##axis)

static inline void axis_calc( const char* axis_name, double step_size_, double d, double double_s, double* ramp_d, double a, double* v, double* dwell_d, uint32_t* c0, uint32_t* cmin, double* recipr_t_acc, double* recipr_t_move)
{
  if (d != 0.0) {
    char aname = *axis_name;
    if (islower( aname)) {
      aname = toupper( aname);
    }
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      printf( "%c move : ", aname);
    }
    if (double_s > d) {
      /*
       * Move length is too short to reach full speed.
       * Recalculate new (lower) top speed and remove the dwell.
       *
       * Ramp length becomes half the move length.
       */
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "(can't reach full speed) ");
      }
      *v = sqrt( a * d);
      *ramp_d = 0.5 * d;
      *dwell_d = 0.0;
      *recipr_t_acc = *v / *ramp_d;
    } else {
     /*
      * Move has ramp up, constant velocity and ramp down phases
      */
      *ramp_d = 0.5 * double_s;
      *dwell_d = d - double_s;
    }
   /*
    * Update the time it takes for the entire move to complete.
    * (All axes will generate the same duration).
    */
    if (*recipr_t_move == 0.0) {
      *recipr_t_move = *v / (4 * *ramp_d + *dwell_d);
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "(set move duration to %1.3lf [ms]) ",
		SI2MS( RECIPR( *recipr_t_move)));
      }
    }
    if (*ramp_d < step_size_) {
     /*
      * Replace a move with ramps that are too short to execute by
      * a single constant (slightly lower) velocity move.
      */
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "(removed ramps smaller than stepsize) ");
      }
      *ramp_d  = 0.0;
      *dwell_d = d;
      *v = *dwell_d * *recipr_t_move;
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "\n   no ramps, dwell= %3.6lf [mm], velocity= %3.3lf [mm/s], duration= %1.3lf [ms]\n",
		SI2MM( *dwell_d), SI2MM( *v), SI2MS( RECIPR( *recipr_t_move)));
      }
      *cmin = fclk * step_size_ / *v ;
      *c0   = *cmin;
    } else {
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "\n   ramp= %3.6lf [mm], dwell= %3.6lf [mm], velocity= %3.3lf [mm/s], duration= %1.3lf [ms]\n",
		SI2MM( *ramp_d), SI2MM( *dwell_d), SI2MM( *v), SI2MS( (4 * *ramp_d + *dwell_d) / *v));
      }
      *cmin = fclk * step_size_ / *v ;
      *c0   = (uint32_t) (c_acc * sqrt( step_size_ / a));
      if (*c0 < *cmin) {
       /*
	* Does this really happen?
	*/
        if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
		printf( "   *** motor can start at dwell speed, no acceleration needed (setting c0 to cmin)\n");
        }
        *ramp_d  = 0.0;
        *dwell_d = d;
        *v = *dwell_d * *recipr_t_move;
	if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
	  printf( "   ramp: %3.6lf [mm], dwell: %3.6lf [mm], velocity: %3.3lf [mm/s], duration: %1.3lf [ms]\n",
		  SI2MM( *ramp_d), SI2MM( *dwell_d), SI2MM( *v), SI2MS( (4 * *ramp_d + *dwell_d) / *v));
	}
	*cmin = fclk * step_size_ / *v ;
	*c0   = *cmin;
      }
    }
  } else {
   /*
    * NOP
    */
    *ramp_d  = 0.0;
    *dwell_d = 0.0;
    *cmin = 0;
    *c0   = 0;
  }
}

#define AXIS_CALC( axis) axis_calc( #axis, step_size_##axis, d##axis, double_s##axis, &ramp_d##axis, a##axis, &v##axis, &dwell_d##axis, &c0##axis, &cmin##axis, &recipr_t_acc, &recipr_t_move)

/*
 * All dimensions are in SI units and relative
 */
void traject_delta_on_all_axes( traject5D* traject)
{
  static unsigned long int serno = 0;
  static time_t t0;
  if (traject == NULL) {
    return;
  }
  if (serno++ == 0) {
    time( &t0);
  }
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "\nMOVE[ #%lu %ds] traject_delta_on_all_axes( traject( %0.9lf, %1.9lf, %1.9lf, %1.9lf, F=%u) [m])\n",
	    serno, (int)time( NULL)-(int)t0,
	    traject->dx, traject->dy, traject->dz, traject->de, traject->feed);
  }

  double dx = traject->dx;
  double dy = traject->dy;
  double dz = traject->dz;
  double de = traject->de;

  int reverse_x = 0;
  if (dx < 0.0) {
    dx = -dx;
    reverse_x = 1;
  }
  int reverse_y = 0;
  if (dy < 0.0) {
    dy = -dy;
    reverse_y = 1;
  }
  int reverse_z = 0;
  if (dz < 0.0) {
    dz = -dz;
    reverse_z = 1;
  }
  int reverse_e = 0;
  if (de < 0.0) {
    de = -de;
    reverse_e = 1;
  }
  // We're only moving in 3D space, e-axis isn't part of this!
  double distance = sqrt( dx * dx + dy * dy + dz * dz);
  if (distance < 2.0E-9) {
    if (de == 0.0) {
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "*** Null move, distance = %1.9lf\n", distance);
      }
      return;	// TODO: will this suffice ?
    }
    // If E is only moving axis, set distance from E
    distance = de;
  }
 /*
  * Travel distance and requested velocity are now known.
  * Determine the velocities for the individual axes
  * using the distances and total duration of the move.
  * If a calculated velocity is higher than the maximum
  * allowed, slow down the entire move.
  */
  double recipr_dt = traject->feed / ( 60000.0 * distance);	/* [m/s] / [m] */
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Request: total distance = %1.6lf [mm], vector velocity = %1.3lf [mm/s] => est. time = %1.3lf [ms]\n",
	    SI2MM( distance), SI2MS( traject->feed / 60000.0), SI2MS( RECIPR( recipr_dt)));
  }
  int v_change = 0;
  double vx = dx * recipr_dt;
  if (vx > vx_max) {	  // clip feed !
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      printf( "*** clipping vx (%1.6lf) to vx_max (%1.6lf)\n", vx, vx_max);
    }
    recipr_dt = vx_max / dx;
    v_change = 1;
  }
  double vy = dy * recipr_dt;
  if (vy > vy_max) {	  // clip feed !
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      printf( "*** clipping vy (%1.6lf) to vy_max (%1.6lf)\n", vy, vy_max);
    }
    recipr_dt = vy_max / dy;
    v_change = 1;
  }
  double vz = dz * recipr_dt;
  if (vz > vz_max) {	  // clip feed !
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      printf( "*** clipping vz (%1.6lf) to vz_max (%1.6lf)\n", vz, vz_max);
    }
    recipr_dt = vz_max / dz;
    v_change = 1;
  }
  double ve = de * recipr_dt;
  if (ve > ve_max) {	  // clip feed !
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      printf( "*** clipping ve (%1.6lf) to ve_max (%1.6lf)\n", ve, ve_max);
    }
    recipr_dt = ve_max / de;
    v_change = 1;
  }
 /*
  * If one or more velocity were limited by its maximum,
  * some of the other values may be incorrect. Recalculate all.
  */
  if (v_change) {
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      printf( "Velocity changed to %1.3lf [mm/s] and duration to %1.3lf [ms] due to this clipping\n",
	      SI2MM( distance * recipr_dt), SI2MS( RECIPR( recipr_dt)));
    }
    vx = dx * recipr_dt;
    vy = dy * recipr_dt;
    vz = dz * recipr_dt;
    ve = de * recipr_dt;
  }
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Velocities - X: %1.3lf, Y: %1.3lf, Z %1.3lf, E: %1.3lf [mm/s]\n",
	    SI2MM( vx), SI2MM( vy), SI2MM( vz), SI2MM( ve));
  }
 /*
  * For a neat linear move, all ramps must start and end at the same moment
  * and have constant (or synchronized) accelation.
  * Now that the targeted velocity is now known for each axis, determine
  * how long it takes for that axis to reach its target speed using maximum
  * acceleration. The slowest axis then scales the acceleration used for all axes.
  */
  double tx_acc = vx * recipr_a_max_x;
  double ty_acc = vy * recipr_a_max_y;
  double tz_acc = vz * recipr_a_max_z;
  double te_acc = ve * recipr_a_max_e;
 /*
  * determine the largest period and scale the acceleration for all axes.
  */
  double t_acc = fmax( fmax( tx_acc, ty_acc), fmax( tz_acc, te_acc));
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Time needed to reach velocity: X= %1.3lf, Y= %1.3lf, Z= %1.3lf, E= %1.3lf => MAX= %1.3lf [ms]\n",
	   SI2MS( tx_acc), SI2MS( ty_acc), SI2MS( tz_acc), SI2MS( te_acc), SI2MS( t_acc));
  }
  double recipr_t_acc = 1.0 / t_acc;
  double ax = vx * recipr_t_acc;
  double ay = vy * recipr_t_acc;
  double az = vz * recipr_t_acc;
  double ae = ve * recipr_t_acc;
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Synchronized acceleration constants: X= %1.3lf, Y= %1.3lf, Z= %1.3lf, E= %1.3lf [m/s^2]\n", 
	    ax, ay, az, ae);
  }
 /*
  *  Length of acceleration/deceleration traject:
  *    s = v^2/2a or s = a.t^2/2
  */
  double t_square  = t_acc * t_acc;
  double double_sx = ax * t_square;
  double double_sy = ay * t_square;
  double double_sz = az * t_square;
  double double_se = ae * t_square;
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Distance to reach full speed: X= %1.6lf Y= %1.6lf Z= %1.6lf E= %1.6lf [mm]\n",
	    SI2MM( 0.5 * double_sx), SI2MM( 0.5 * double_sy), SI2MM( 0.5 * double_sz), SI2MM( 0.5 * double_se));
  }
  double ramp_dx, ramp_dy, ramp_dz, ramp_de;
  double dwell_dx, dwell_dy, dwell_dz, dwell_de;

  uint32_t c0x, c0y, c0z, c0e;
  uint32_t cminx, cminy, cminz, cmine;
  double recipr_t_move = 0.0;	// means: not set
 /*
  * Calculate the timing for all axes
  */
  AXIS_CALC( x);
  AXIS_CALC( y);
  AXIS_CALC( z);
  AXIS_CALC( e);
 /*
  * Put the sign back into the deltas
  */
  if (reverse_x) {
    ramp_dx = -ramp_dx;
    dwell_dx = -dwell_dx;
  }
  if (reverse_y) {
    ramp_dy = -ramp_dy;
    dwell_dy = -dwell_dy;
  }
  if (reverse_z) {
    ramp_dz = -ramp_dz;
    dwell_dz = -dwell_dz;
  }
  if (reverse_e) {
    ramp_de = -ramp_de;
    dwell_de = -dwell_de;
  }
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Ramps: X= %1.6lf, Y= %1.6lf, Z= %1.6lf, E= %1.6lf [mm], ramp duration= %1.3lf [ms]\n",
	    SI2MM( ramp_dx), SI2MM( ramp_dy), SI2MM( ramp_dz), SI2MM( ramp_de), SI2MS( RECIPR( recipr_t_acc)));
  }

 /*
  * Up from version v3.0 of the stepper firmware, the stepper driver does acceleration
  * and deceleration timing and switching all by itself. Only one command needs to be
  * queued to accelerate from zero speed to max speed, dwell at max speed and decelerate
  * back to zero speed.
  */
  int any_move = 0;

  any_move += QUEUE_MOVE( x);
  any_move += QUEUE_MOVE( y);
  any_move += QUEUE_MOVE( z);
  any_move += QUEUE_MOVE( e);

  if (any_move) {
    pruss_queue_execute();
    any_move = 0;
  }
  if (ramp_dx != 0.0) {
    pruss_queue_adjust_for_ramp( 1, (int32_t)(1.0E9 * ramp_dx));
  }
  if (ramp_dy != 0.0) {
    pruss_queue_adjust_for_ramp( 2, (int32_t)(1.0E9 * ramp_dy));
  }
  if (ramp_dz != 0.0) {
    pruss_queue_adjust_for_ramp( 3, (int32_t)(1.0E9 * ramp_dz));
  }
  if (ramp_de != 0.0) {
    pruss_queue_adjust_for_ramp( 4, (int32_t)(1.0E9 * ramp_de));
  }
  if (config_e_axis_is_always_relative()) {
    pruss_queue_adjust_origin( 4);
  }
}

static void pruss_axis_config( int axis, double step_size, int reverse)
{
  uint32_t ssi = (int) SI2NM( step_size);
  uint16_t ssn = 1000;
  uint16_t sst = (int) ssn * (SI2NM( step_size) - ssi);

  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Set axis nr %d step size (%1.6lf) to %u + %u / %u [nm] and %s direction\n",
	    axis, step_size, ssi, sst, ssn, (reverse) ? "reversed" : "normal");
  }
  pruss_queue_config_axis( axis, ssi, sst, ssn, reverse);
}

int traject_wait_for_completion( void)
{
  while (!pruss_queue_empty()) {
    if (pruss_stepper_halted()) {
      return -1;
    }
    sched_yield();
  }
  return 0;
}

int traject_abort( void)
{
  // FIXME: implementation
  return 1;
}

int traject_status_print( void)
{
  printf( "traject_status_print - TODO:implementation\n");
  return 0;
}

int traject_init( void)
{
  /*
   *  Configure 'constants' from configuration
   */
  vx_max = FEED2SI( config_get_max_feed( x_axis));
  vy_max = FEED2SI( config_get_max_feed( y_axis));
  vz_max = FEED2SI( config_get_max_feed( z_axis));
  ve_max = FEED2SI( config_get_max_feed( e_axis));

  recipr_a_max_x = RECIPR( config_get_max_accel( x_axis));
  recipr_a_max_y = RECIPR( config_get_max_accel( y_axis));
  recipr_a_max_z = RECIPR( config_get_max_accel( z_axis));
  recipr_a_max_e = RECIPR( config_get_max_accel( e_axis));

  step_size_x = config_get_step_size( x_axis);
  step_size_y = config_get_step_size( y_axis);
  step_size_z = config_get_step_size( z_axis);
  step_size_e = config_get_step_size( e_axis);

  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "  step: X = %9.3lf, Y = %9.3lf, Z = %9.3lf, E = %9.3lf [um]\n",
	    SI2UM( step_size_x), SI2UM( step_size_y), SI2UM( step_size_z), SI2UM( step_size_e)); 
    printf( "  amax: X = %9.3lf, Y = %9.3lf, Z = %9.3lf, E = %9.3lf [mm/s^2]\n",
	    SI2MM( RECIPR( recipr_a_max_x)), SI2MM( RECIPR( recipr_a_max_y)),
	    SI2MM( RECIPR( recipr_a_max_z)), SI2MM( RECIPR( recipr_a_max_e)));
    printf( "  vmax: X = %9.3lf, Y = %9.3lf, Z = %9.3lf, E = %9.3lf [mm/s]\n",
	    SI2MM( vx_max), SI2MM( vy_max), SI2MM( vz_max), SI2MM( ve_max)); 
  }
  /*
   *  Configure PRUSS and propagate stepper configuration
   */
  if (mendel_sub_init( "pruss_stepper", pruss_stepper_init) < 0) {
    return -1;
  }

  // Set per axis step size and reversal bit
  pruss_axis_config( 1, step_size_x, config_reverse_axis( x_axis));
  pruss_axis_config( 2, step_size_y, config_reverse_axis( y_axis));
  pruss_axis_config( 3, step_size_z, config_reverse_axis( z_axis));
  pruss_axis_config( 4, step_size_e, config_reverse_axis( e_axis));

  /* Set the duration of the active part of the step pulse */
  pruss_queue_set_pulse_length( 1, 10 * 200);
  pruss_queue_set_pulse_length( 2, 10 * 200);
  pruss_queue_set_pulse_length( 3, 10 * 200);
  pruss_queue_set_pulse_length( 4, 10 * 200);

  /* Set internal reference for all axis to current position */
  pruss_queue_set_origin( 1);
  pruss_queue_set_origin( 2);
  pruss_queue_set_origin( 3);
  pruss_queue_set_origin( 4);

  pruss_queue_set_idle_timeout( 30);	// set a 3 seconds timeout
  return 0;
}

