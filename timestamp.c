
#include <unistd.h>
#include <time.h>

#include "timestamp.h"


#ifdef _POSIX_MONOTONIC_CLOCK
static clockid_t clk = CLOCK_MONOTONIC;
#else
# error NO SUITABLE CLOCK SOURCE AVAILABLE
#endif

static struct timespec t0;


void timestamp_init( void)
{
  clock_gettime( clk, &t0);
}

double timestamp_get( void)
{
  struct timespec t1;
  clock_gettime( clk, &t1);
  int nsecs = t1.tv_nsec - t0.tv_nsec;
  int secs  = t1.tv_sec  - t0.tv_sec;
  return secs + 0.000000001 * nsecs;
}

