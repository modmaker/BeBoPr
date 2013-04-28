#ifndef	_HOME_H
#define _HOME_H

#include "bebopr.h"

extern void home_axis_to_limit_switch( axis_e axis, int32_t* position, uint32_t feed, int reverse);

#endif	/* _HOME_H */
