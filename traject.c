
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <sys/time.h>

#include "bebopr.h"
#include "traject.h"
#include "pruss_stepper.h"
#include "debug.h"
#include "beaglebone.h"
#include "mendel.h"
#include "limit_switches.h"
#include "timestamp.h"


/*
 *  Settings that are changed during initialization.
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

static double speed_override_factor = 1.0;
static double extruder_override_factor = 1.0;

static unsigned int moveNrs[ 1 + 5];	/* 5 axes, 1-based */

void incMoveNr( unsigned int pruss_axis)
{
  ++moveNrs[ pruss_axis];
}

/* ---------------------------------- */

static inline int queue_accel( const char* axis_name, double ramp, double a, double v, uint32_t n0, uint32_t c0, uint32_t cmin, double origin)
{
  if (v != 0.0 && ramp != 0.0) {
    char aname = *axis_name;
    if (islower( aname)) {
      aname = toupper( aname);
    }
    int pruss_axis = (aname < 'X') ? aname - 'E' + 4 : aname - 'X' + 1;
    ++moveNrs[ pruss_axis];
    if (c0 > cmin) {
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "Queue ACCEL %c%u: ramping up to v=%1.3lf [mm/s] with a=%1.3lf [m/s^2] to %1.6lf [mm]"
		" (from n0=%u, c0=%u up to cmin=%u)\n",
		aname, moveNrs[ pruss_axis], SI2MM( v), a, SI2MM( origin + ramp), n0, c0, cmin);
      }
      pruss_queue_accel( pruss_axis, n0, c0, cmin, SI2POS( origin + ramp));
    } else {
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "Queue ACCEL %c%u: running at v=%1.3lf [mm/s] to %1.6lf [mm] (at c=%u)\n",
		aname, moveNrs[ pruss_axis], SI2MM( v), SI2MM( origin + ramp), cmin);
      }
      pruss_queue_dwell( pruss_axis, cmin, SI2POS( origin + ramp));
    }
    return 1;
  }
  return 0;
}

#ifdef PRU_ABS_COORDS
#define QUEUE_ACCEL( axis) queue_accel( #axis, ramp_up_d##axis, a##axis, v##axis, n0##axis, c0##axis, cmin##axis, axis##0)
#else
#define QUEUE_ACCEL( axis) queue_accel( #axis, ramp_up_d##axis, a##axis, v##axis, n0##axis, c0##axis, cmin##axis, 0)
#endif

/* ---------------------------------- */

static inline int queue_dwell( const char* axis_name, double v, double ramp, double dwell, uint32_t cdwell, double origin)
{
  if (v != 0.0 && dwell != 0.0) {
    char aname = *axis_name;
    if (islower( aname)) {
      aname = toupper( aname);
    }
    int pruss_axis = (aname < 'X') ? aname - 'E' + 4 : aname - 'X' + 1;
    ++moveNrs[ pruss_axis];
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      printf( "Queue DWELL %c%u: running at v=%1.3lf [mm/s] to %1.6lf [mm] (at c=%u)\n",
	      aname, moveNrs[ pruss_axis], SI2MM( v), SI2MM( origin + ramp + dwell), cdwell);
    }
    pruss_queue_dwell( pruss_axis, cdwell, SI2POS( origin + ramp + dwell));
    return 1;
  }
  return 0;
}

#ifdef PRU_ABS_COORDS
#define QUEUE_DWELL( axis) queue_dwell( #axis, v##axis, ramp_up_d##axis, dwell_d##axis, cdwell##axis, axis##0)
#else
#define QUEUE_DWELL( axis) queue_dwell( #axis, v##axis, ramp_up_d##axis, dwell_d##axis, cdwell##axis, 0)
#endif

/* ---------------------------------- */

static inline int queue_decel( const char* axis_name, double a, double v, double ramp_up, double dwell, double ramp_down,
			uint32_t nmin, uint32_t c0, uint32_t cmin, double origin)
{
  if (v != 0.0 && ramp_up != 0.0) {
    char aname = *axis_name;
    if (islower( aname)) {
      aname = toupper( aname);
    }
    int pruss_axis = (aname < 'X') ? aname - 'E' + 4 : aname - 'X' + 1;
    ++moveNrs[ pruss_axis];
    if (c0 > cmin) {
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "Queue DECEL %c%u: ramping down from v=%1.3lf [mm/s] with a=%1.3lf [m/s^2] to %1.6lf [mm]"
		" (down from nmin=%u, cmin=%u)\n",
		aname, moveNrs[ pruss_axis], SI2MM( v), a, SI2MM( origin + ramp_up + dwell + ramp_down), nmin, cmin);
      }
      pruss_queue_decel( pruss_axis, nmin, cmin, SI2POS( origin + ramp_up + dwell + ramp_down));
    } else {
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "Queue DECEL %c%u: running at v=%1.3lf [mm/s] to %1.6lf [mm] (at c=%u)\n",
		aname, moveNrs[ pruss_axis], SI2MM( v), SI2MM( origin + ramp_up + dwell + ramp_down), cmin);
      }
      pruss_queue_dwell( pruss_axis, cmin, SI2POS( origin + ramp_up + dwell + ramp_down));
    }
    return 1;
  }
  return 0;
}

#ifdef PRU_ABS_COORDS
#define QUEUE_DECEL( axis) queue_decel( #axis, a##axis, v##axis, ramp_up_d##axis, dwell_d##axis, ramp_down_d##axis, \
					nmin##axis, c0##axis, cmin##axis, axis##0)
#else
#define QUEUE_DECEL( axis) queue_decel( #axis, a##axis, v##axis, ramp_up_d##axis, dwell_d##axis, ramp_down_d##axis, \
					nmin##axis, c0##axis, cmin##axis, 0)
#endif

/* ---------------------------------- */

static inline void axis_calc( const char* axis_name, double step_size_, double d, double double_s, double* ramp_up_d, double* ramp_down_d,
			double a, double* v, double* dwell_d, uint32_t* n0, uint32_t* nmin,
			uint32_t* c0, uint32_t* cmin, uint32_t* cdwell, double* recipr_t_acc, double* recipr_t_move)
{
  if (d == 0.0) {
   /*
    * NOP
    */
    *ramp_up_d  = 0.0;
    *ramp_down_d  = 0.0;
    *dwell_d = 0.0;
    *cmin = 0;
    *c0   = 0;
    *cdwell = 0;
    *n0 = 0;
    *nmin = 0;
  } else {
    char aname = *axis_name;
    if (islower( aname)) {
      aname = toupper( aname);
    }
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      printf( "%c move : ", aname);
    }
    if (d <= double_s) {
      /*
       * Move length is too short to reach full speed.
       * Recalculate new (lower) top speed and remove the dwell.
       *
       * Ramp length becomes half the move length.
       */
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "(can't reach full speed) ");
      }
      // 2012-10-31 SJL - runs with new pruss code that doesn't need all the work-arounds
      *ramp_down_d = d / 2;
      *ramp_up_d = d - *ramp_down_d;
      *dwell_d = 0.0;
      *v = sqrt( a * d);
      // Used for debug output only:
      *recipr_t_acc = 2 * *v / (*ramp_up_d + *ramp_down_d);
    } else {
     /*
      * Move has ramp up, constant velocity and ramp down phases
      */
      *ramp_up_d = double_s / 2;
      *ramp_down_d = *ramp_up_d;
      *dwell_d = d - *ramp_up_d - *ramp_down_d;
    }

   /*
    * Update the time it takes for the entire move to complete.
    * (All axes should generate the same duration, so only do this once).
    */
    if (*recipr_t_move == 0.0) {
      *recipr_t_move = *v / (2 * (*ramp_up_d + *ramp_down_d) + *dwell_d);
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "(set move duration to %1.3lf [ms]) ",
		SI2MS( RECIPR( *recipr_t_move)));
      }
    }

   /*
    * Calculate timing parameters for stepper
    */
    *cmin = fclk * step_size_ / *v ;
    *c0   = (uint32_t) (c_acc * sqrt( step_size_ / a));
    *cdwell = *cmin;
    if (*c0 < *cmin) {
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "(can start at dwell speed, no ramping needed) ");
      }
      /* replace the ramp by a dwell at half the speed for the same distance */
      *c0   = 2 * *cmin;
      *cmin = *c0;
    } else if (*ramp_up_d < step_size_) {
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "(replacing tiny ramp by dwell) ");
      }
      /* replace the ramp by a dwell at half the speed for the same distance */
      *c0   = 2 * *cmin;
      *cmin = *c0;
    }
    if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
      printf( "\n   ramp-up= %3.6lf [mm], dwell= %3.6lf [mm], ramp-down= %3.6lf [mm], velocity= %3.3lf [mm/s], duration= %1.3lf [ms]\n",
	      SI2MM( *ramp_up_d), SI2MM( *dwell_d), SI2MM( *ramp_down_d), SI2MM( *v),
	      SI2MS( (2 * (*ramp_up_d + *ramp_down_d) + *dwell_d) / *v));
    }
    *n0 = 0;	// start acceleration from zero speed
    *nmin = 0;	// zero will use the end value from the acceleration phase
  }
}

#define AXIS_CALC( axis) axis_calc( #axis, step_size_##axis, d##axis, double_s##axis, &ramp_up_d##axis, &ramp_down_d##axis, \
					a##axis, &v##axis, &dwell_d##axis, &n0##axis, &nmin##axis, \
					&c0##axis, &cmin##axis, &cdwell##axis, &recipr_t_acc, &recipr_t_move)

/*
 * All dimensions are in SI units and relative
 */
void traject_delta_on_all_axes( traject5D* traject)
{
  static unsigned long int serno = 0;
  static double queued_time = 0;
  static double calc_start_old;
  double feed = speed_override_factor * traject->feed;

  if (traject == NULL) {
    return;
  }
  if (serno++ == 0) {
    calc_start_old = timestamp_get();
  }

#ifdef PRU_ABS_COORDS
  double dx = traject->x1 - traject->x0;
  double dy = traject->y1 - traject->y0;
  double dz = traject->z1 - traject->z0;
  double de = traject->e1 - traject->e0;
#else
  double dx = traject->dx;
  double dy = traject->dy;
  double dz = traject->dz;
  double de = traject->de;
#endif

  double move_start = timestamp_get();
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "\nMOVE[ #%lu %1.3fs] traject_delta_on_all_axes( traject( %0.9lf, %1.9lf, %1.9lf, %1.9lf, F=%1.3lf) [m])\n",
	    serno, move_start, dx, dy, dz, de, feed);
  }

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
 /*
  * The E-axis is not part of the (3D) movement vector. The velocity
  * of the E-axis is directly determined by the feed of the G1 move,
  * unless reduced by an axis velocity above its limit.
  */
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
  double recipr_dt = feed / ( 60000.0 * distance);	/* [m/s] / [m] */
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Request: total distance = %1.6lf [mm], vector velocity = %1.3lf [mm/s] => est. time = %1.3lf [ms]\n",
	    SI2MM( distance), SI2MS( feed / 60000.0), SI2MS( RECIPR( recipr_dt)));
  }
 /*
  * FIXME: Either the calculations, Linux scheduling or (?) causes some large gaps here:
  * Sometimes it takes up to 50 ms before a new move is ready to be queued here. To
  * (try to) prevent a gap falling between the two moves, slow down short moves.
  * NOTE: This is highly experimental and probably the scheduling should be looked at first!
  */
  double calc_start = timestamp_get();
  double elapsed_time = calc_start - calc_start_old;
  calc_start_old = calc_start;
  const double est_calc_time = 0.010; /* [s] */
  queued_time -= elapsed_time;
  if (!pruss_stepper_busy() || queued_time < 0) {
    queued_time = 0;
  }
  double slack = queued_time - est_calc_time;
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "%sElapsed_time= %1.3lfms slack= %1.3fms, duration of moves in queue= %1.3fms\n",
	    (slack < 0) ? "GAP - " : "", SI2MS( elapsed_time), SI2MS( slack), SI2MS( queued_time));
  }
  if (slack < 0) {
   /*
    * No slack. If the next move is short one, slow it down to keep the stepper
    * busy until the next move will be queued. Note: this is not exact science
    * but based on best guess!
    */
    if (-slack * recipr_dt > 1.0) {
      recipr_dt = RECIPR( -slack);
      if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
        printf( "*** Short move requested, slowing down average velocity to %1.3lf [mm/s], duration %1.3lf [ms] to prevent gap\n",
		SI2MS( recipr_dt * distance), SI2MS( RECIPR( recipr_dt)));
      }
    }
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
  * and have constant (or at least synchronized) accelation.
  * Now that the targeted velocity is known for each axis, determine
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
  double ramp_up_dx, ramp_up_dy, ramp_up_dz, ramp_up_de;
  double ramp_down_dx, ramp_down_dy, ramp_down_dz, ramp_down_de;
  double dwell_dx, dwell_dy, dwell_dz, dwell_de;

  uint32_t c0x, c0y, c0z, c0e;
  uint32_t cminx, cminy, cminz, cmine;
  uint32_t cdwellx, cdwelly, cdwellz, cdwelle;
  uint32_t n0x, n0y, n0z, n0e;
  uint32_t nminx, nminy, nminz, nmine;
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
    ramp_up_dx = -ramp_up_dx;
    ramp_down_dx = -ramp_down_dx;
    dwell_dx = -dwell_dx;
  }
  if (reverse_y) {
    ramp_up_dy = -ramp_up_dy;
    ramp_down_dy = -ramp_down_dy;
    dwell_dy = -dwell_dy;
  }
  if (reverse_z) {
    ramp_up_dz = -ramp_up_dz;
    ramp_down_dz = -ramp_down_dz;
    dwell_dz = -dwell_dz;
  }
  if (reverse_e) {
    ramp_up_de = -ramp_up_de;
    ramp_down_de = -ramp_down_de;
    dwell_de = -dwell_de;
  }
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Ramps: X= %1.6lf/%1.6lf, Y= %1.6lf/%1.6lf, Z= %1.6lf/%1.6lf, E= %1.6lf/%1.6lf [mm], ramp duration= %1.3lf [ms]\n",
	    SI2MM( ramp_up_dx), SI2MM( ramp_down_dx), SI2MM( ramp_up_dy), SI2MM( ramp_down_dy), SI2MM( ramp_up_dz), SI2MM( ramp_down_dz),
	    SI2MM( ramp_up_de), SI2MM( ramp_down_de), SI2MS( RECIPR( recipr_t_acc)));
  }

// If for at least one axis, there is a move but no dwell,
// all axes that have a move should have no dwell!
  int x_move_but_no_dwell = (dx != 0.0 && dwell_dx == 0.0);
  int y_move_but_no_dwell = (dy != 0.0 && dwell_dy == 0.0);
  int z_move_but_no_dwell = (dz != 0.0 && dwell_dz == 0.0);
  int e_move_but_no_dwell = (de != 0.0 && dwell_de == 0.0);
  int x_no_move_or_no_dwell = (dx == 0.0 || dwell_dx == 0.0);
  int y_no_move_or_no_dwell = (dy == 0.0 || dwell_dy == 0.0);
  int z_no_move_or_no_dwell = (dz == 0.0 || dwell_dz == 0.0);
  int e_no_move_or_no_dwell = (de == 0.0 || dwell_de == 0.0);
  if (x_move_but_no_dwell || y_move_but_no_dwell || z_move_but_no_dwell || e_move_but_no_dwell) {
    if (x_no_move_or_no_dwell && y_no_move_or_no_dwell && z_no_move_or_no_dwell && e_no_move_or_no_dwell) {
    } else {
      printf( "*** UNEXPECTED DWELL TROUBLES !!!!\n");
    }
  }

  double queue_start = timestamp_get();
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Calculations took %1.3lf ms\n", SI2MS( queue_start - calc_start));
  }
  queued_time += RECIPR( recipr_t_move);
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Duration of moves in queue= %1.3lf [ms] after adding move of %1.3lf [ms]\n",
	    SI2MS( queued_time), SI2MS( RECIPR( recipr_t_move)));
  }

#ifdef PRU_ABS_COORDS
  double x0 = traject->x0;
  double y0 = traject->y0;
  double z0 = traject->z0;
  double e0 = traject->e0;
#endif

  int any_move;

 /*
  * Up from version v6.0 of the stepper firmware, the stepper driver strings together
  * the individual acceleration, dwell and deceleration moves.
  */
// RAMP UP
  any_move = 0;
  any_move += QUEUE_ACCEL( x);
  any_move += QUEUE_ACCEL( y);
  any_move += QUEUE_ACCEL( z);
  any_move += QUEUE_ACCEL( e);
  if (any_move) {
    pruss_queue_execute();
  }

// DWELL
  any_move = 0;
  any_move += QUEUE_DWELL( x);
  any_move += QUEUE_DWELL( y);
  any_move += QUEUE_DWELL( z);
  any_move += QUEUE_DWELL( e);
  if (any_move) {
    pruss_queue_execute();
  }

// RAMP DOWN
  any_move = 0;
  any_move += QUEUE_DECEL( x);
  any_move += QUEUE_DECEL( y);
  any_move += QUEUE_DECEL( z);
  any_move += QUEUE_DECEL( e);
  if (any_move) {
    pruss_queue_execute();
  }

  // this command act as NOP that will generate a sync point for the axes !
  pruss_queue_set_pulse_length( 4, 20 * 200);

  double end_time = timestamp_get();
  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Queueing took %1.3lf ms\n", 1000 * (end_time - queue_start));
  }
}

static void pruss_axis_config( int axis, double step_size, int reverse)
{
  uint32_t ssi = (int) SI2NM( step_size) & ~1;	// make even for symetry

  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    printf( "Set axis nr %d step size to %u [nm] and %s direction\n",
	    axis, ssi, (reverse) ? "reversed" : "normal");
  }
  pruss_queue_config_axis( axis, ssi, reverse);
}

int traject_wait_for_completion( void)
{
  while (!pruss_queue_empty() || pruss_stepper_busy()) {
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

double traject_set_speed_override( double factor)
{
  double old = speed_override_factor;
  speed_override_factor = factor;
  return old;
}

double traject_set_extruder_override( double factor)
{
  double old = extruder_override_factor;
  extruder_override_factor = factor;
  pruss_axis_config( 4, step_size_e / factor, config_reverse_axis( e_axis));
  return old;
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

 /*
  * Duration of active part of step pulse. Note that the real generated
  * value also depends on the cycletime of the stepper code and can be
  * less than the value specified here.
  * actual = FLOOR( setting / cycletime) * cycletime
  * Setting a value less than the cycletime will generate zero-length
  * step pulses!
  */
  const int step_pulse_time = 16;	/* [us] */
  /* Set the duration of the active part of the step pulse */
  pruss_queue_set_pulse_length( 1, step_pulse_time * 200);
  pruss_queue_set_pulse_length( 2, step_pulse_time * 200);
  pruss_queue_set_pulse_length( 3, step_pulse_time * 200);
  pruss_queue_set_pulse_length( 4, step_pulse_time * 200);

  /* Set internal reference for all axis to current position */
  pruss_queue_set_origin( 1);
  pruss_queue_set_origin( 2);
  pruss_queue_set_origin( 3);
  pruss_queue_set_origin( 4);

  /* Configure limit switches in the PRUSS */
#define CONFIG_AXIS_LIMSW( axis, prussaxis, gpiomin, gpiomax)	\
  do {									\
    uint8_t min_gpio = (config_axis_has_min_limit_switch( axis)) ? gpiomin : 255; \
    uint8_t max_gpio = (config_axis_has_max_limit_switch( axis)) ? gpiomax : 255;	\
    uint8_t min_invert = config_min_limit_switch_is_active_low( axis);	\
    uint8_t max_invert = config_max_limit_switch_is_active_low( axis);	\
    pruss_queue_config_limsw( prussaxis, min_gpio, min_invert, max_gpio, max_invert); \
  } while (0)

  CONFIG_AXIS_LIMSW( x_axis, 1, XMIN_GPIO, XMAX_GPIO);
  CONFIG_AXIS_LIMSW( y_axis, 2, YMIN_GPIO, YMAX_GPIO);
  CONFIG_AXIS_LIMSW( z_axis, 3, ZMIN_GPIO, ZMAX_GPIO);

  pruss_queue_set_idle_timeout( 30);	// set a 3 seconds timeout
  return 0;
}

