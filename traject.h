#ifndef _TRAJECT_H
#define _TRAJECT_H

#include <stdint.h>

#include "bebopr.h"

typedef struct {
#ifdef PRU_ABS_COORDS
  double		x0;
  double		y0;
  double		z0;
  double		e0;
  double		x1;
  double		y1;
  double		z1;
  double		e1;
#else
  double		dx;
  double		dy;
  double		dz;
  double		de;
#endif
  uint32_t		feed;
} traject5D;

extern void traject_delta_on_all_axes( traject5D* delta);

extern int traject_wait_for_completion( void);
extern int traject_abort( void);
extern int traject_status_print( void);

extern int traject_init( void);

#endif
