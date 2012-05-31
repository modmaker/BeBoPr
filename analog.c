
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>

#include "analog.h"
#include "beaglebone.h"
#include "mendel.h"
#include "debug.h"


struct analog_data_record {
  const char*		device_path;
  int			channel;
  update_callback* 	callback;
  unsigned int          value;		// last value
  int			filter_length;	// set to value > 0 for running average
  struct average_data {
    unsigned int	value;		// integer part average
    unsigned int	remainder;	// remainder
    unsigned int	count;		// divisor for average
  } average;	  
};

static struct analog_data_record* analog_data = NULL;
static unsigned int analog_num_channels = 0;

/*
 * Set a callback for temperature updates, one callback for each analog channel
 * can be registered. Any previous value will be overwritten.
 * Set a callback to NULL to disable it.
 */
int analog_set_update_callback( unsigned int analog_channel, update_callback* temp_update, update_channel_t temp_channel)
{
  fprintf( stderr, "analog_set_update_callback for analog channel %d, temperature channel %d\n", analog_channel, temp_channel);
  if (analog_channel >= 0 && analog_channel <= analog_num_channels) {
    analog_data[ analog_channel].callback = temp_update;
    analog_data[ analog_channel].channel  = temp_channel;
  }
  return 0;
}

#define ANALOG_CYCLE_TIME	 200000 /* usecs, sensor readout cycle */
#define UPDATE_CYCLE_TIME       2000000 /* usecs, update interval callbacks */

/*
 * This is the worker thread that reads the analog inputs and
 * calls the callbacks to export the read values.
 */
void* analog_worker( void* arg)
{
  int* fd = NULL;
  int i;
  char buf[ 10];
  int ret;
  int cycle = 0;
  
  fprintf( stderr, "analog_thread: started\n");
  fd = calloc( analog_num_channels, sizeof( *fd));
  for (i = 0 ; i < analog_num_channels ; ++i) {
    ret = fd[ i] = open( analog_data[ i].device_path, O_RDONLY);
    if (ret < 0) {
      perror( "analog_thread: opening of ADC file failed");
      goto failure;
    }
  }
  i = 0;
  for (;;) {
    /*
     * Update all adc values at a fixed rate, determined by ANALOG_CYCLE_TIME.
     * Spread the reads evenly over the cycle.
     */
    usleep( ANALOG_CYCLE_TIME / analog_num_channels);
    ret = read( fd[ i], buf, sizeof( buf));
    if (ret > 0) {
      int val = atoi( buf);
      struct analog_data_record* p = &analog_data[ i];
      if (p->filter_length > 0) {
        int avg = p->average.value;
        int rem = p->average.remainder;
        int cnt = p->average.count;
	if (cnt < p->filter_length) {
	  ++cnt;
	  p->average.count = cnt;
	}
	val = (cnt - 1) * avg + val + rem;
	p->average.value = val / cnt;
	p->average.remainder = val % cnt;
        p->value = (val + rem + cnt / 2) / cnt;
      } else {
	p->value = val;
      }
      lseek( fd[ i], 0, SEEK_SET);
    } else if (ret < 0) {
      perror( "analog thread: ADC read failed -");
      goto failure;
    }
    ++i;
    if (i == analog_num_channels) {
      // Once every this often, push values to clients
      if (++cycle >= UPDATE_CYCLE_TIME / ANALOG_CYCLE_TIME) {
	unsigned int ch;
	for (ch = 0 ; ch < analog_num_channels ; ++ch) {
          struct analog_data_record* p = &analog_data[ ch];
	  if (p->callback != NULL) {
	    if (debug_flags & DEBUG_ANALOG) {
	      fprintf( stderr, "analog_worker, calling temp_update for channel %d with value %d\n",
		      p->channel, p->value);
	    }
	    (void) (p->callback)( p->channel, p->value);
	  }
	}
        if (debug_flags & DEBUG_ANALOG) {
          int ch;
          printf( "ADC values:");
          for (ch = 0 ; ch < analog_num_channels ; ++ch) {
            printf( " avg[ %d]=%d", ch, analog_data[ ch].value);
          }
          printf( "\n");
        }
        cycle = 0;
      }
      i = 0;
    }
  }
failure:
  pthread_exit( NULL);
}

/*
 * Configuration settings are stored seperately (bebopr_rx.c) and
 * a configuration call is used to communicate these with this
 * code.
 */
static const analog_config_struct* analog_config_data = NULL;
static int analog_config_items = 0;

int analog_config( const analog_config_struct* config_data, int nr_config_items)
{
  analog_config_data  = config_data;
  analog_config_items = nr_config_items;
  // this shouldn't be called more than once, so keep the code simple
  analog_data         = calloc( nr_config_items, sizeof( struct analog_data_record));
  analog_num_channels = 0;
  return 0;
}


static pthread_t worker;

int analog_init( void)
{
  unsigned int ch;
  if (analog_config_data) {
    for (ch = 0 ; ch < analog_config_items ; ++ch) {
      struct analog_data_record* p = &analog_data[ ch];
      p->device_path       = analog_config_data[ ch].device_path;
      p->filter_length     = analog_config_data[ ch].filter_length;
      p->callback          = NULL;
      p->value             = 0;
      p->average.count     = 0;
      p->average.value     = 0;
      p->average.remainder = 0;
      ++analog_num_channels;
    }
    return mendel_thread_create( "analog", &worker, NULL, &analog_worker, NULL);
  }
  fprintf( stderr, "analog_init: no configuration data!\n");
  return -1;
}

int analog_get_raw_value( unsigned int channel, int* value)
{
  if (value != NULL && channel >= 0 && channel < analog_num_channels) {
    // TODO: lock?
    *value = analog_data[ channel].value;
    // TODO: unlock?
    return 0;
  }
  return -1;
}
