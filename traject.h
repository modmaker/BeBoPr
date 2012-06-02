#ifndef _TRAJECT_H
#define _TRAJECT_H

#include <stdint.h>

#include "bebopr.h"

typedef struct {
  double		dx;
  double		dy;
  double		dz;
  double		de;
  uint32_t		feed;
} traject5D;

extern void traject_delta_on_all_axes( traject5D* delta);

extern int traject_wait_for_completion( void);
extern int traject_abort( void);
extern int traject_status_print( void);

extern int traject_init( void);

#endif
