/* Copyright (c) 2011, RidgeRun
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the RidgeRun.
 * 4. Neither the name of the RidgeRun nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "gpio.h"


#define MAX_BUF 100


/****************************************************************
 * gpio_open_value_file
 ****************************************************************/

int gpio_open_file( unsigned int gpio, const char* file)
{
  char path[ MAX_BUF];

  (void) snprintf( path, sizeof( path), "/sys/class/gpio/gpio%d/%s", gpio, file);
  int fd = open( path, O_RDONLY | O_NONBLOCK );
  if (fd < 0) {
    perror( "gpio_open_value_file - ");
  }
  return fd;
}

/****************************************************************
 * gpio_write_value_to_file
 ****************************************************************/

int gpio_write_value_to_file( const char* file, const char* value)
{
  char path[ MAX_BUF];

  (void) snprintf( path, sizeof( path), "/sys/class/gpio/%s", file); 
  int fd = open( path, O_WRONLY);
  if (fd < 0) {
    perror( "gpio_set failed");
    return fd;
  }
  // Include terminating null in write!
  write( fd, value, strlen( value) + 1); 
  close( fd);
  return 0;
}

/****************************************************************
 * gpio_write_value_to_pin_file
 ****************************************************************/
/*
 *  Write 'value' to 'file' in the directory subtree for 'gpio'
 */
int gpio_write_value_to_pin_file( unsigned int gpio, const char* file, const char* value)
{
  char gpio_file[ MAX_BUF];

  (void) snprintf( gpio_file, sizeof( gpio_file), "/gpio%d/%s", gpio, file);
  return gpio_write_value_to_file( gpio_file, value);
}


int gpio_write_int_value_to_file( const char* file, int value)
{
  char buffer[ 8];

  (void) snprintf( buffer, sizeof( buffer), "%d", value); 
  return  gpio_write_value_to_file( file, buffer);
}

