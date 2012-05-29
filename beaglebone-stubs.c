
#include <unistd.h>

#include "beaglebone.h"
#include "beaglebone-stubs.h"

void sei( void)
{
  // TODO: implementation ?
}

void cli( void)
{
  // TODO: implementation ?
}

void wd_init( void)
{
  // TODO: implementation ?
}

void wd_reset( void)
{
  // TODO: implementation ?
}

void delay_ms( unsigned int count)
{
  usleep( 1000 * count);
}

