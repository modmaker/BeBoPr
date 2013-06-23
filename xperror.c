
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "xperror.h"

/*
 * Thread safe perror with variable format and arguments
 */
void xperror( const char* format, ...)
{
  va_list args;
  char buf[ 200];
  va_start( args, format);
  vfprintf( stderr, format, args); 
  va_end( args);
  if (strerror_r( errno, buf, sizeof( buf)) == -1) {
    perror( "xperror internal failure:");
    return;
  }
  fprintf( stderr, ": %s\n", buf);
}

