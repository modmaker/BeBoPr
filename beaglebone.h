#ifndef _BEAGLEBONE_H
#define _BEAGLEBONE_H

#include <stdint.h>

#define NR_ITEMS( x) (sizeof( (x)) / sizeof( *(x)))

typedef const char* channel_tag;
static inline const char* tag_name( channel_tag tag) { return (char*)tag; }

#endif
