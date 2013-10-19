
#include <string.h>
#include <stdio.h>
#include <glob.h>

#include "sys_paths.h"

/*
 *  Use glob to find sys device paths with suffixes.
 *  
 *  Check return value or result in buffer for success.
 */
const char* sys_path_finder( char* buffer, size_t max_size, const char* path)
{
  glob_t globbuf;

  int result = glob( path, GLOB_ERR | GLOB_NOSORT, NULL, &globbuf);
  if (result == GLOB_NOMATCH) {
    buffer = NULL;    		// error: none found
    *buffer = '\0';
  } else {
    if (globbuf.gl_pathc != 1) {
      buffer = NULL;      	// error: more than one found
      *buffer = '\0';
    } else {
      strncpy( buffer, globbuf.gl_pathv[ 0], max_size);
      printf( "sys_path_finder( '%s') returns '%s'\n", path, buffer);
    }
  }
  globfree( &globbuf);
  return buffer;
}

