
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include "pwm.h"
#include "debug.h"
#include "bebopr.h"
#include "sys_paths.h"


struct pwm_channel_record {
  channel_tag		id;
  const char*           device_path;
  unsigned int          frequency;
  int			duty_fd;
  unsigned int		duty;
  unsigned int		period;
};

static struct pwm_channel_record* pwm_channels;
static unsigned int num_pwm_channels;

static int pwm_index_lookup( channel_tag pwm_channel)
{
  for (int ix = 0 ; ix < num_pwm_channels ; ++ix) {
    if (pwm_channels[ ix].id == pwm_channel) {
      return ix;
    }
  }
  if (debug_flags & DEBUG_PWM) {
    fprintf( stderr, "pwm_index_lookup failed for '%s'\n", tag_name( pwm_channel));
  }
  return -1;
}

/*
 * Configuration settings are stored seperately (bebopr_rx.c) and
 * a configuration call is used to communicate these with this
 * code.
 */
static pwm_config_record* pwm_config_data = NULL;
static int pwm_config_items = 0;

int pwm_config( pwm_config_record* config_data, int nr_config_items)
{
  if (debug_flags & DEBUG_PWM) {
    printf( "pwm_config called with %d records'\n", nr_config_items);
  }
  pwm_config_data  = config_data;
  pwm_config_items = nr_config_items;
  // this mustn't be called more than once, so keep the code simple
  pwm_channels     = calloc( nr_config_items, sizeof( struct pwm_channel_record));
  for (int ch = 0 ; ch < nr_config_items ; ++ch) {
    pwm_channels[ ch].duty_fd = -1;
  }
  num_pwm_channels = 0;
  return 0;
}

static int pwm_write_int_to_file( const char* path, const char* fname, int value)
{
  char s[ 100];
  snprintf( s, sizeof( s), "%s/%s", path, fname);
  int fd = open( s, O_WRONLY);
  if (fd < 0) {
    fprintf( stderr, "pwm_write_int_to_file: open for file '%s' failed: %s\n", s, strerror( errno));
    return -1;
  }
  char s2[ 32];
  snprintf( s2, sizeof( s2), "%d", value);
  int count = strlen( s2);
  int result = write( fd, s2, count);
  if (result < 0) {
    // TODO: do we need strerror_r ?
    fprintf( stderr, "pwm_write_int_to_file: write to '%s' failed: %s\n", s, strerror( errno));
  } else if (result == count) {
    result = 0;
  } else {
    fprintf( stderr, "pwm_write_int_to_file: short write on file '%s'\n", s);
    result = -1;
  }
  close( fd);
  return result;
}

static int pwm_read_int_from_file( const char* path, const char* fname, int* value)
{
  char s[ 100];
  snprintf( s, sizeof( s), "%s/%s", path, fname);
  int fd = open( s, O_RDONLY);
  if (fd < 0) {
    perror( "pwm_read_int_from_file: open failed");
    fprintf( stderr, "pwm_read_int_from_file: open failed for file '%s'\n", fname);
    return -1;
  }
  lseek( fd, 0, SEEK_SET);
  char s2[ 32];
  int result = read( fd, s2, sizeof( s2) - 1);
//  fprintf( stderr, "pwm_read_int_from_file: read returns %d, '%s'\n", result, s2);
  if (result < 0) {
    // TODO: do we need strerror_r ?
    fprintf( stderr, "pwm_read_int_from_file: read from '%s' failed: %s (%d)\n", s, strerror( errno), errno);
  } else if (result > 0) {
    *value = atoi( s2);
//    fprintf( stderr, "pwm_read_int_from_file: *value = %d\n", *value);
    result = 0;
  } else {
    fprintf( stderr, "pwm_read_int_from_file: short read on file '%s'\n", s);
    result = -1;
  }
  close( fd);
  return result;
}

void pwm_exit( void)
{
  if (debug_flags & DEBUG_PWM) {
    printf( "pwm_exit called, releasing PWM subsystem\n");
  }
  for (int ch = 0 ; ch < num_pwm_channels ; ++ch) {
    if (pwm_channels[ ch].duty_fd != -1) {
      struct pwm_channel_record* pd = &pwm_channels[ ch];
      pwm_set_output( pd->id, 0);
      pwm_write_int_to_file( pd->device_path, "run", 0);
      if (get_kernel_type() == e_kernel_3_2) {
        pwm_write_int_to_file( pd->device_path, "request", 0);
      }
      close (pd->duty_fd);
      free( (void*)pd->device_path);
      pwm_channels[ ch].duty_fd = -1;
    }
  }
}

int pwm_init( void)
{
  if (debug_flags & DEBUG_PWM) {
    printf( "pwm_init called'\n");
  }
  if (pwm_config_data) {
    char s[ 100];
    for (int ch = 0 ; ch < pwm_config_items ; ++ch) {
      pwm_config_record*         ps = &pwm_config_data[ ch];
      struct pwm_channel_record* pd = &pwm_channels[ ch];
      char*                    path = malloc( NAME_MAX);

      pd->id                = ps->tag;
      pd->device_path       = sys_path_finder( path, NAME_MAX, ps->device_path);
      pd->frequency         = ps->frequency;
      pd->duty_fd           = -1;

      ++num_pwm_channels;

      if (get_kernel_type() == e_kernel_3_2) {
        pwm_write_int_to_file( pd->device_path, "request", 1);
      }

      pwm_write_int_to_file( pd->device_path, "polarity", 0);

      if (get_kernel_type() == e_kernel_3_8) {
        // FIXME: quick hack to work around differences between ehrpwm implementations
        pwm_write_int_to_file( pd->device_path, "run", 0);
        unsigned int duty;
        if (pwm_read_int_from_file( pd->device_path, "duty", (int*)&duty) >= 0) {
	  if (debug_flags & DEBUG_PWM) {
            fprintf( stderr, "pwm_init: '%s' has duty set to %d\n", pd->id, duty);
	  }
        } else {
          fprintf( stderr, "pwm_init: '%s' could not read duty file\n", pd->id);
        }
	if (debug_flags & DEBUG_PWM) {
          fprintf( stderr, "pwm_init: '%s' disabled pwm, setting duty to 0\n", pd->id);
	}
        pwm_write_int_to_file( pd->device_path, "duty", 0);
        pd->duty = 0;
        // if a frequency is specified in the configuration, use it
        if (pd->frequency > 0) {
          pd->period = 1000000000 / pd->frequency;
	  if (debug_flags & DEBUG_PWM) {
            fprintf( stderr, "pwm_init: '%s' set frequency to %d Hz, period of %u [ns]\n",
		    pd->id, pd->frequency, pd->period);
	  }
          pwm_write_int_to_file( pd->device_path, "period", pd->period);
        }
        if (pwm_read_int_from_file( pd->device_path, "period", (int*)&pd->period) >= 0 /*&& period > 0*/) {
	  pd->frequency = 1000000000 / pd->period;
	  if (debug_flags & DEBUG_PWM) {
            fprintf( stderr, "pwm_init: '%s' has period set to %u\n", pd->id, pd->period);
	    fprintf( stderr, "pwm_init: '%s' set frequency for to %d Hz\n", pd->id, pd->frequency);
	  }
        } else {
          fprintf( stderr, "pwm_init: '%s' could not read period file\n", pd->id);
        }
        // now open duty file for further reference
        snprintf( s, sizeof( s), "%s/duty", pd->device_path);
        pd->duty_fd = open( s, O_WRONLY);
        if (pd->duty_fd < 0) {
          perror( "pwm_init: failed to open 'duty' file");
        }
      }

      if (get_kernel_type() == e_kernel_3_2) {
        pwm_write_int_to_file( pd->device_path, "duty_percent", 0);
        if (pd->frequency) {
          pwm_write_int_to_file( pd->device_path, "period_freq", pd->frequency);
        }
        snprintf( s, sizeof( s), "%s/duty_percent", pd->device_path);
        pd->duty_fd = open( s, O_WRONLY);
        if (pd->duty_fd < 0) {
          perror( "pwm_init: failed to open 'duty_percent' file");
        }
      }

      pwm_set_output( pd->id, 0);
      pwm_write_int_to_file( pd->device_path, "run", 1);
    }
    return 0;
  }
  fprintf( stderr, "pwm_init: no configuration data!\n");
  return -1;
}

int pwm_set_output( channel_tag pwm_channel, unsigned int percentage)
{
  int ix = pwm_index_lookup( pwm_channel);
  if (ix >= 0 && percentage <= 100) {
    int fd = pwm_channels[ ix].duty_fd;
    // Only write to the file if it is (still) available
    if (fd < 0) {
      return -1;
    }
    char s[ 30];
    int result = 0;
    int count = 0;

    if (get_kernel_type() == e_kernel_3_8) {
      // Black uses cycletime [ns] instread of percentage
      struct pwm_channel_record* pd = &pwm_channels[ ix];
      unsigned int duty = (percentage * (1000000000UL / 100)) / pd->frequency;
      if (pd->duty != duty) {
        pd->duty = duty;
	if (debug_flags & DEBUG_PWM) {
	  fprintf( stderr, "pwm_set_output: '%s' changing duty into %u (%u%% of %u)\n",
		  pd->id, pd->duty, percentage, pd->period);
	}
	snprintf( s, sizeof( s), "%u", pd->duty);
	count = strlen( s);
	lseek( fd, 0, SEEK_SET);
	result = write( fd, s, count);
      }
    }

    if (get_kernel_type() == e_kernel_3_2) {
      // White uses percentage
      snprintf( s, sizeof( s), "%u", percentage);
      count = strlen( s);
      result = write( fd, s, count);
    }

    if (result < 0) {
      perror( "pwm_set_output: error writing to duty_fd file");
    } else if (result == count) {
      return 0;
    }
  }
  return -1;
}

channel_tag pwm_lookup_by_name( const char* name)
{
  for (int ix = 0 ; ix < num_pwm_channels ; ++ix) {
    channel_tag tag = pwm_channels[ ix].id;
    if (strcmp( tag_name( tag), name) == 0) {
      return tag;
    }
  }
  return NULL;
}

