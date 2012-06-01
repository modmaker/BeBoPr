#ifndef _BEAGLEBONE_H
#define _BEAGLEBONE_H

#include <stdint.h>

#define NR_ITEMS( x) (sizeof( (x)) / sizeof( *(x)))

/* convert [mm/min] into [m/s] */
#define	FEED2SI( f) ((f) / 60000.0)
/* convert into reciprocal */
#define RECIPR( x) (1.0 / (x))
/* convert SI unit [s] into [ms] */
#define SI2MS( x) (1000.0 * (x))
/* convert SI unit [m] into [mm] */
#define SI2MM( x) (1000.0 * (x))
/* convert SI unit [m] into [nm] */
#define SI2NM( x) (1.0E9 * (x))

typedef const char* channel_tag;
static inline const char* tag_name( channel_tag tag) { return (char*)tag; }

#endif
