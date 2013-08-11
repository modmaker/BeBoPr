#ifndef _LIMIT_SWITCHES_H
#define _LIMIT_SWITCHES_H

#include "traject.h"

#ifdef BONE_BRIDGE
// This are the (fixed) assignments for the BeBoPr with Bridge
#define XMIN_GPIO 67
#define XMAX_GPIO 69
#define YMIN_GPIO 68
#define YMAX_GPIO 26
#define ZMIN_GPIO 27
#define ZMAX_GPIO 65
#else
// This are the (fixed) assignments for the BeBoPr
#define XMIN_GPIO 10
#define XMAX_GPIO 11
#define YMIN_GPIO  8
#define YMAX_GPIO  9
#define ZMIN_GPIO 79
#define ZMAX_GPIO 78
#endif

// Runtime
// return 1 if switch is activated, 0 otherwise
extern int limsw_max( axis_e axis);
extern int limsw_min( axis_e axis);

extern int limsw_init( void);

#endif
