#ifndef _PLATFORM_H
#define _PLATFORM_H

#if ARCH == arm
# include "beaglebone.h"
#else
# include "arduino.h"
#endif


#endif
