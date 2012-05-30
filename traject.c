
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "traject.h"
#include "pruss.h"
#include "debug.h"


/* convert [mm/min] into [m/s] */
#define	FEED2SI( f) ((f) / 60000.0)
#define RECIPR( x) (1.0 / (x))
#define SI2MS( x) (1000.0 * (x))
#define SI2MM( x) (1000.0 * (x))

static const double fclk = 200000000.0;
static double vx_max;
static double vy_max;
static double vz_max;
static double ve_max;

static const double recipr_a_max_x = RECIPR( 14.9);	/* [s^2/m] */
static const double recipr_a_max_y = RECIPR( 2.5);
static const double recipr_a_max_z = RECIPR( 2.4);
static const double recipr_a_max_e = RECIPR( 2.4);
static const double step_size_x = 6250.0E-9;		/* [m] */
static const double step_size_y = 6250.0E-9;
static const double step_size_z =  390.125E-9;
static const double step_size_e = 1000.0E-9;
static const double c_acc = 191201673.632;	// = 0.676 * fclk * sqrt( 2.0);

uint32_t traject_get_max_feed( axis_e axis)
{
  switch (axis) {
  case x_axis:	return 68571;
  case y_axis:	return 68571;
  case z_axis:	return   900;
  case e_axis:	return  1000;	// TBD
  default:	return 0;
  }
}

double traject_get_step_size( axis_e axis)
{
  switch (axis) {
  case x_axis:	return step_size_x;
  case y_axis:	return step_size_y;
  case z_axis:	return step_size_z;
  case e_axis:	return step_size_e;
  default:	return 0.0;
  }
}

void traject_move_one_axis( double delta, uint32_t feed)
{
}

/*
 * All dimensions are in SI units !
 */
void traject_delta_on_all_axes( traject5D* traject)
{
  if (traject == NULL) {
    return;
  }
  //  if (DEBUG_TRAJECT && (debug_flags & DEBUG_TRAJECT)) {
    fprintf( stderr,  "\ntraject_delta_on_all_axes( traject(%lf,%lf,%lf,%lf,%u) ]\n",
	     traject->dx, traject->dy, traject->dz, traject->de, traject->feed);
    //  }
  //  printf( "Acceleration constant c_acc = %1.6f\n", c_acc);

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
  if (distance < 5.0E-9) {
    printf( "*** Null move, distance = %1.9lf\n", distance);
    return;	// TODO: will this suffice ?
  }
  /*
   *  Vector length and requested feed are known now.
   *  Determine feed per axis and clip to limits.
   */
  double recipr_dt = traject->feed / ( 60000.0 * distance);	/* [m/s] / [m] */
  printf( "Request: total distance = %1.6lf [mm], speed = %1.3lf [mm/s] => time = %1.3lf [s]\n",
	  SI2MM( distance), SI2MS( traject->feed / 60000.0), RECIPR( recipr_dt));
  double vx = dx * recipr_dt;
  if (vx > vx_max) {	  // clip feed !
    recipr_dt = vx_max / dx;
    printf( "*** clipping vx (%1.6lf) to vx_max (%1.6lf)\n", vx, vx_max);
  }
  double vy = dy * recipr_dt;
  if (vy > vy_max) {	  // clip feed !
    recipr_dt = vy_max / dy;
    printf( "*** clipping vy (%1.6lf) to vy_max (%1.6lf)\n", vy, vy_max);
  }
  double vz = dz * recipr_dt;
  if (vz > vz_max) {	  // clip feed !
    recipr_dt = vz_max / dz;
    printf( "*** clipping vz (%1.6lf) to vz_max (%1.6lf)\n", vz, vz_max);
  }
  double ve = de * recipr_dt;
  if (ve > ve_max) {	  // clip feed !
    recipr_dt = ve_max / de;
    printf( "*** clipping ve (%1.6lf) to ve_max (%1.6lf)\n", ve, ve_max);
  }
  //  double dt = RECIPR( recipr_dt);	/* [s] */	// nodig ???
  printf( "Limited values: time = %1.3lf [ms] => speed = %1.3lf [mm/s]\n",
	  SI2MS( RECIPR( recipr_dt)), SI2MM( distance * recipr_dt));
  vx = dx * recipr_dt;
  vy = dy * recipr_dt;
  vz = dz * recipr_dt;
  ve = de * recipr_dt;
  printf( "Velocities - X: %1.3lf, Y: %1.3lf, Z %1.3lf, E: %1.3lf [mm/s]\n",
	  SI2MM( vx), SI2MM( vy), SI2MM( vz), SI2MM( ve));
  /*
   * For each axis the targeted speed is known now.
   *
   * Determine how long it takes the slowest axis to reach its target speed.
   * Calculate the acceleration for each axis from this value.
   */

  /* determine per axis acceleration and clip to max value */
  double tx_acc = vx * recipr_a_max_x;
  double ty_acc = vy * recipr_a_max_y;
  double tz_acc = vz * recipr_a_max_z;
  double te_acc = ve * recipr_a_max_e;
  /* For a true linear move, all acceleration periods must be equal and constant,
   * determine the largest period and scale the acceleration for each axis. */

  double t_acc = fmax( fmax( tx_acc, ty_acc), fmax( tz_acc, te_acc));
  printf( "Acceleration duration - X: %1.3lf, Y: %1.3lf, Z: %1.3lf, E: %1.3lf => MAX=%1.3lf [ms]\n",
	  SI2MS( tx_acc), SI2MS( ty_acc), SI2MS( tz_acc), SI2MS( te_acc), SI2MS( t_acc));
  double recipr_t_acc = 1.0 / t_acc;

  double ax = vx * recipr_t_acc;
  double ay = vy * recipr_t_acc;
  double az = vz * recipr_t_acc;
  double ae = ve * recipr_t_acc;
  printf( "Acceleration factor - X: %1.3lf, Y: %1.3lf, Z: %1.3lf, E: %1.3lf [m/s^2]\n", ax, ay, az, ae);

  /*
   * s = v^2/2a or s = a.t^2/2
   */
  //  printf( "sqrt( dx/ax)=%1.6lf [s], sqrt( dx/ax)=%1.6lf [s], sqrt( dx/ax)=%1.6lf [s], sqrt( dx/ax)=%1.6lf [s]\n",
  //	  sqrt( dx / ax), sqrt( dy / ay), sqrt( dz / az), sqrt( de / ae));
  double t_square  = t_acc * t_acc;
  double double_sx = ax * t_square;
  double double_sy = ay * t_square;
  double double_sz = az * t_square;
  double double_se = ae * t_square;
  printf( "Distance to reach full speed: - X:%1.6lf Y:%1.6lf Z:%1.6lf E:%1.6lf [mm]\n",
	  SI2MM( 0.5 * double_sx), SI2MM( 0.5 * double_sy), SI2MM( 0.5 * double_sz), SI2MM( 0.5 * double_se));
  //
  //  double half_t_square = 0.5 * t_acc * t_acc;
  double ramp_dx, ramp_dy, ramp_dz, ramp_de;
  double dwell_dx, dwell_dy, dwell_dz, dwell_de;
  //

  // Split traject into ramp-up, dwell and ramp-down
  if ((double_sx > dx) || (double_sy > dy) || (double_sz > dz) || (double_se > de)) {
    printf( "*** Move is too short to accelerate to full speed!\n");

    ramp_dx = 0.5 * dx;
    ramp_dy = 0.5 * dy;
    ramp_dz = 0.5 * dz;
    ramp_de = 0.5 * de;

    dwell_dx = 0.0;
    dwell_dy = 0.0;
    dwell_dz = 0.0;
    dwell_de = 0.0;

    vx = sqrt( ax * dx);
    vy = sqrt( ay * dy);
    vz = sqrt( az * dz);
    ve = sqrt( ae * de);

    printf( "Maximum velocities - X: %1.3lf, Y: %1.3lf, Z %1.3lf, E: %1.3lf [mm/s]\n",
	    SI2MM( vx), SI2MM( vy), SI2MM( vz), SI2MM( ve));
  } else {

    ramp_dx = 0.5 * double_sx;
    ramp_dy = 0.5 * double_sy;
    ramp_dz = 0.5 * double_sz;
    ramp_de = 0.5 * double_se;

    if (ramp_dx < step_size_x) {
      ramp_dx  = 0.0;
      dwell_dx = dx;
    } else {
      dwell_dx = dx - double_sx;
    }
    if (ramp_dy < step_size_y) {
      ramp_dy  = 0.0;
      dwell_dy = dy;
    } else {
      dwell_dy = dy - double_sy;
    }
    if (ramp_dz < step_size_z) {
      ramp_dz  = 0.0;
      dwell_dz = dz;
    } else {
      dwell_dz = dz - double_sz;
    }
    if (ramp_de < step_size_e) {
      ramp_de  = 0.0;
      dwell_de = de;
    } else {
      dwell_de = de - double_se;
    }
  }
  // Put sign back into distances
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
  printf( "Slopes - X: %1.6lf, Y: %1.6lf, Z: %1.6lf, E: %1.6lf [mm] (duration=%1.3lf [ms])\n",
	  SI2MM( ramp_dx), SI2MM( ramp_dy), SI2MM( ramp_dz), SI2MM( ramp_de), SI2MS( RECIPR( recipr_t_acc)));
  // TODO: make conditional: only for used axes ???
  uint32_t c0x =  (uint32_t) (c_acc * sqrt( step_size_x / ax));
  uint32_t c0y =  (uint32_t) (c_acc * sqrt( step_size_y / ay));
  uint32_t c0z =  (uint32_t) (c_acc * sqrt( step_size_z / az));
  uint32_t c0e =  (uint32_t) (c_acc * sqrt( step_size_e / ae));
  uint32_t cminx = fclk * step_size_x / vx ;
  uint32_t cminy = fclk * step_size_y / vy ;
  uint32_t cminz = fclk * step_size_z / vz ;
  uint32_t cmine = fclk * step_size_e / ve ;
  //
  //  printf( "ramping up in %1.3lf [ms]\n", 1000 / dt);
  int any_move = 0;
  if (vx != 0.0) {
    printf( "Queue X: accelerate to %1.3lf [mm/s] with a=%1.3lf [m/s^2] over %1.3lf [mm] (c0=%u,cmin=%u)\n",
	    SI2MM( vx), ax, SI2MM( ramp_dx), c0x, cminx);
    pruss_queue_accel( 1, c0x, cminx, (int32_t)(1.0E9 * ramp_dx));
    any_move = 1;
  }
  if (vy != 0.0) {
    printf( "Queue Y: accelerate to %1.3lf [mm/s] with a=%1.3lf [m/s^2] over %1.3lf [mm] (c0=%d,cmin=%u)\n",
	    SI2MM( vy), ay, SI2MM( ramp_dy), c0y, cminy);
    pruss_queue_accel( 2, c0y, cminy, (int32_t)(1.0E9 * ramp_dy));
    any_move = 1;
  }
  if (vz != 0.0) {
    printf( "Queue Z: accelerate to %1.3lf [mm/s] with a=%1.3lf [m/s^2] over %1.3lf [mm] (c0=%d,cmin=%u)\n",
	    SI2MM( vz), az, SI2MM( ramp_dz), c0z, cminz);
    pruss_queue_accel( 3, c0z, cminz, (int32_t)(1.0E9 * ramp_dz));
    any_move = 1;
  }
  if (ve != 0.0) {
    printf( "Queue E: accelerate to %1.3lf [mm/s] with a=%1.3lf [m/s^2] over %1.3lf [mm] (c0=%d,cmin=%u)\n",
	    SI2MM( ve), ae, SI2MM( ramp_de), c0e, cmine);
    pruss_queue_accel( 4, c0e, cmine, (int32_t)(1.0E9 * ramp_de));
    any_move = 1;
  }
  if (any_move) {
    pruss_queue_execute();
    any_move = 0;
  }
  //
  if (dwell_dx != 0.0) {
    printf( "Queue X: dwell at speed %1.3lf [mm/s] over %1.6f [mm] (cmin=%u)\n",
	    SI2MM( vx), SI2MM( dwell_dx), cminx);
    pruss_queue_dwell( 1, cminx, (int32_t)(1.0E9 * dwell_dx));
    any_move = 1;
  }
  if (dwell_dy != 0.0) {
    printf( "Queue Y: dwell at speed %1.3lf [mm/s] over %1.6f [mm] (cmin=%u)\n",
	    SI2MM( vy), SI2MM( dwell_dy), cminy);
    pruss_queue_dwell( 2, cminy, (int32_t)(1.0E9 * dwell_dy));
    any_move = 1;
  }
  if (dwell_dz != 0.0) {
    printf( "Queue Z: dwell at speed %1.3lf [mm/s] over %1.6f [mm] (cmin=%u)\n",
	    SI2MM( vz), SI2MM( dwell_dz), cminz);
    pruss_queue_dwell( 3, cminz, (int32_t)(1.0E9 * dwell_dz));
    any_move = 1;
  }
  if (dwell_de != 0.0) {
    printf( "Queue E: dwell at speed %1.3lf [mm/s] over %1.6f [mm] (cmin=%u)\n",
	    SI2MM( ve), SI2MM( dwell_de), cmine);
    pruss_queue_dwell( 4, cmine, (int32_t)(1.0E9 * dwell_de));
    any_move = 1;
  }
  if (any_move) {
    pruss_queue_execute();
    any_move = 0;
  }
  //
  if (vx != 0.0) {
    printf( "Queue X: decelerate over %1.6lf [mm] with a=-%1.3lf [m/s^2]\n",
	    SI2MM( ramp_dx), ax);
    pruss_queue_decel( 1, (int32_t)(1.0E9 * ramp_dx));
    any_move = 1;
  }
  if (vy != 0.0) {
    printf( "Queue Y: decelerate over %1.6lf [mm] with a=-%1.3lf [m/s^2]\n",
	    SI2MM( ramp_dy), ay);
    pruss_queue_decel( 2, (int32_t)(1.0E9 * ramp_dy));
    any_move = 1;
  }
  if (vz != 0.0) {
    printf( "Queue Z: decelerate over %1.6lf [mm] with a=-%1.3lf [m/s^2]\n",
	    SI2MM( ramp_dz), az);
    pruss_queue_decel( 3, (int32_t)(1.0E9 * ramp_dz));
    any_move = 1;
  }
  if (ve != 0.0) {
    printf( "Queue E: decelerate over %1.6lf [mm] with a=-%1.3lf [m/s^2]\n",
	    SI2MM( ramp_de), ae);
    pruss_queue_decel( 4, (int32_t)(1.0E9 * ramp_de));
    any_move = 1;
  }
  if (any_move) {
    pruss_queue_execute();
    any_move = 0;
  }
}

static void pruss_axis_config( int axis, double step_size, int reverse)
{
  uint32_t ssi = (int)(step_size * 1.0E9);
  uint16_t ssn = 1024;
  uint16_t sst = (int) ssn * (step_size * 1.0E9 - ssi);

  printf( "Set axis nr %d step size (%1.6lf) to %u + %u / %u [nm] and %s direction\n",
	  axis, step_size, ssi, sst, ssn, (reverse) ? "reversed" : "normal");
  pruss_queue_config_axis( axis, ssi, sst, ssn, reverse);
}

void traject_init( void)
{
  vx_max = FEED2SI( traject_get_max_feed( x_axis));
  vy_max = FEED2SI( traject_get_max_feed( y_axis));
  vz_max = FEED2SI( traject_get_max_feed( z_axis));
  ve_max = FEED2SI( traject_get_max_feed( e_axis));

  pruss_init();

  // Set per axis step size and reversal bit
  pruss_axis_config( 1, step_size_x, 1);
  pruss_axis_config( 2, step_size_y, 0);
  pruss_axis_config( 3, step_size_z, 1);
  pruss_axis_config( 4, step_size_e, 0);
  pruss_queue_set_idle_timeout( 30);	// set a 3 seconds timeout
}

