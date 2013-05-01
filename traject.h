#ifndef _TRAJECT_H
#define _TRAJECT_H

#include <stdint.h>

#include "bebopr.h"

typedef struct {
  double                s0x, s0y, s0z, s0e;
  double                s1x, s1y, s1z, s1e;
  uint32_t              feed;
} traject5D;

typedef struct {
  double                s0x, s0y, s0z, s0e;
  double                dx, dy, dz, de;
  double                vx, vy, vz, ve;
  int                   reverse_x, reverse_y, reverse_z, reverse_e;
  int                   chainable;
  int                   null_move;
  long unsigned int     serno;
  double                feed;
} move5D;

extern void traject_delta_on_all_axes( const traject5D* delta);
extern void traject_calc_all_axes( const traject5D* delta, move5D* move);
extern void traject_move_all_axes( move5D* move);
extern void traject_optimize( move5D* move0, move5D* move1);

extern int traject_wait_for_completion( void);
extern int traject_abort( void);
extern int traject_status_print( void);

extern double traject_set_speed_override( double factor);
extern double traject_set_extruder_override( double factor);

extern void incMoveNr( unsigned int pruss_axis);

extern int traject_init( void);

#endif
