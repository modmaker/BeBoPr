#ifndef _TRAJECT_H
#define _TRAJECT_H

#include <stdint.h>

typedef enum {
  x_axis, y_axis, z_axis, e_axis
} axis_e;

typedef struct {
  double		dx;
  double		dy;
  double		dz;
  double		de;
  uint32_t		feed;
} traject5D;

extern void traject_delta_on_one_axis( double delta, uint32_t feed);
extern void traject_delta_on_all_axes( traject5D* delta);

extern uint32_t traject_get_max_feed( axis_e axis);

extern void traject_init( void);

#endif
