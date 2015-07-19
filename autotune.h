#ifndef _AUTOTUNE_H
#define _AUTOTUNE_H
//#define PID_TUNING


#include <stdint.h>

uint64_t millis(void);

/* Invoke with
   PID_autotune(150.0, 0, 8); */
void PID_autotune(float temp, int extruder, int ncycles);

#endif // _AUTOTUNE_H
