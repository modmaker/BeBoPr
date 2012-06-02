#ifndef _LIMIT_SWITCHES_H
#define _LIMIT_SWITCHES_H

#include "traject.h"

// Runtime
// return 1 if switch is activated, 0 otherwise
extern int limsw_max( axis_e axis);
extern int limsw_min( axis_e axis);

extern int limsw_init( void);

#endif
