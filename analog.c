
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>

#include "analog.h"
#include "beaglebone.h"
#include "mendel.h"
#include "debug.h"


#define RUNNING_AVG_COUNT	16
#define ANALOG_CYCLE_TIME	200000 /* usecs */


static int avg[ e_analog_num_channels] = { 0 };

/*
 * Array with registered callbacks for temperature updates,
 * one for each analog channel.
 */
static struct update_callback {
  update_callback* func;
  int channel;
} update_callbacks[ e_analog_num_channels];

/*
 * Set a callback for temperature updates, one callback for each analog channel
 * can be registered. Any previous value will be overwritten.
 * Set to function pointer to NULL to disable.
 */
int analog_set_update_callback( analog_channel_e channel, update_callback* temp_update, update_channel_t temp_channel)
{
  fprintf( stderr, "analog_set_update_callback for channel %d, temp_channel %d\n", channel, temp_channel);
  if (channel >= 0 && channel <= e_analog_num_channels) {
    update_callbacks[ channel].func = temp_update;
    update_callbacks[ channel].channel = temp_channel;
  }
  return 0;
}

/*
 * This is the worker thread that reads the analog inputs and
 * calls the callbacks to export the read values.
 */
void* analog_worker( void* arg)
{
#if 0
  // Old kernel 3.2.0 has these here:
  const char* fname[ e_analog_num_channels] = {
    "/sys/devices/platform/tsc/ain2",	// BEBOPR_R0_J7 - THRM0 (mcu_ain1)
    "/sys/devices/platform/tsc/ain4",	// BEBOPR_R0_J8 - THRM1 (mcu_ain3)
    "/sys/devices/platform/tsc/ain6",	// BEBOPR_R0_J9 - THRM2 (mcu_ain5)
  };
#else
  // New kernel 3.2.16 has these here:
  const char* fname[ e_analog_num_channels] = {
    "/sys/devices/platform/omap/tsc/ain2",	// BEBOPR_R0_J7 - THRM0 (mcu_ain1)
    "/sys/devices/platform/omap/tsc/ain4",	// BEBOPR_R0_J8 - THRM1 (mcu_ain3)
    "/sys/devices/platform/omap/tsc/ain6",	// BEBOPR_R0_J9 - THRM2 (mcu_ain5)
  };
#endif
  int fd[ e_analog_num_channels];
  int i;
  char buf[ 10];
  int ret;
  int cycle = 0;
  
  fprintf( stderr, "analog_thread: started\n");
  // TODO: determine where 'tsc' devices are located !
  for (i = 0 ; i < NR_ITEMS( fd) ; ++i) {
    ret = fd[ i] = open( fname[ i], O_RDONLY);
    if (ret < 0) {
      perror( "analog_thread: opening of ADC file failed");
      goto failure;
    }
  }
  i = 0;
  for (;;) {
    /*
     * Update all adc values at a fixed rate, determined by ANALOG_CYCLE_TIME.
     * Take a running average, adding each new reading with a weight of
     * 1 / RUNNING_AVG_COUNT.
     */
    usleep( ANALOG_CYCLE_TIME / e_analog_num_channels);
    ret = read( fd[ i], buf, sizeof( buf));
    if (ret > 0) {
      int val = atoi( buf);
      avg[ i] = ((RUNNING_AVG_COUNT - 1) * avg[ i] + val) / RUNNING_AVG_COUNT;
      lseek( fd[ i], 0, SEEK_SET);
    } else if (ret < 0) {
      perror( "analog thread: ADC read failed -");
      goto failure;
    }
    ++i;
    if (i == NR_ITEMS( avg)) {
      if (++cycle >= 2000000 / ANALOG_CYCLE_TIME) {
	analog_channel_e ch;
	for (ch = 0 ; ch < e_analog_num_channels ; ++ch) {
	  if (update_callbacks[ ch].func != NULL) {
	    if (debug_flags & DEBUG_TEMPERATURE) {
	      fprintf( stderr, "analog_worker, calling temp_update for channel %d with value %d\n",
		       update_callbacks[ ch].channel, avg[ ch]);
	    }
	    (void) (update_callbacks[ ch].func)( update_callbacks[ ch].channel, avg[ ch]);
	  }
	}
	if (debug_flags & DEBUG_TEMPERATURE) {
	  printf( "ADC values: %d, %d, %d\n", avg[ 0], avg[ 1], avg[ 2]);
	}
	cycle = 0;
      }
      i = 0;
    }
  }
failure:
  pthread_exit( NULL);
}

static pthread_t worker;

int analog_init( void)
{
  analog_channel_e i;
  for (i = 0 ; i < e_analog_num_channels ; ++i) {
    update_callbacks[ i].func = NULL;
    avg[ i] = 0;
  }
  int result = mendel_thread_create( "analog", &worker, NULL, &analog_worker, NULL);
  return result;
}

#if 0
int analog_get_value( analog_channel_e channel, int* value)
{
  if (value != NULL && channel >= 0 && channel < NR_ITEMS( avg)) {
    // TODO: lock
    *value = avg[ channel];
    // TODO: unlock
    return 0;
  }
  return -1;
}
#endif
