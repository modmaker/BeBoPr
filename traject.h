#ifndef _TRAJECT_H
#define _TRAJECT_H

#include <stdint.h>

#include "bebopr.h"

typedef struct {
  double		x0;
  double		y0;
  double		z0;
  double		e0;
  double		x1;
  double		y1;
  double		z1;
  double		e1;
  uint32_t		feed;
} traject5D;

extern void traject_delta_on_all_axes( traject5D* delta);

extern int traject_wait_for_completion( void);
extern int traject_abort( void);
extern int traject_status_print( void);

extern double traject_set_speed_override( double factor);
extern double traject_set_extruder_override( double factor);

extern void incMoveNr( unsigned int pruss_axis);

extern int traject_init( void);

#endif
