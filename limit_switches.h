#ifndef _LIMIT_SWITCHES_H
#define _LIMIT_SWITCHES_H

#include "traject.h"

// This are the (fixed) assignments for the BeBoPr
#define XMIN_GPIO 10
#define XMAX_GPIO 11
#define YMIN_GPIO  8
#define YMAX_GPIO  9
#define ZMIN_GPIO 79
#define ZMAX_GPIO 78

// Runtime
// return 1 if switch is activated, 0 otherwise
extern int limsw_max( axis_e axis);
extern int limsw_min( axis_e axis);

extern int limsw_init( void);

#endif
