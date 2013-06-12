
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>

#include "analog.h"
#include "beaglebone.h"
#include "mendel.h"
#include "debug.h"


struct analog_channel_record {
  channel_tag              id;
  const char*           device_path;
  channel_tag              update_channel;
  update_callback*      callback;
  unsigned int          value;          // last value
  int                   filter_length;  // set to value > 0 for running average
  struct average_data {
    unsigned int        value;          // integer part average
    unsigned int        remainder;      // remainder
    unsigned int        count;          // divisor for average
  } average;      
};

static struct analog_channel_record* analog_channels = NULL;
static unsigned int num_analog_channels = 0;

static int analog_index_lookup( channel_tag analog_channel)
{
  for (int ix = 0 ; ix < num_analog_channels ; ++ix) {
    if (analog_channels[ ix].id == analog_channel) {
      return ix;
    }
  }
  if (debug_flags & DEBUG_ANALOG) {
    fprintf( stderr, "analog_index_lookup failed for '%s'\n", tag_name( analog_channel));
  }
  return -1;
}

/*
 * Set a callback for temperature updates, one callback for each analog channel
 * can be registered. Any previous value will be overwritten.
 * Set a callback to NULL to disable it.
 */
int analog_set_update_callback( channel_tag analog_channel, update_callback* ptemp_update, channel_tag update_channel)
{
  if (debug_flags & DEBUG_ANALOG) {
    printf( "analog_set_update_callback from '%s' to '%s'\n", analog_channel, update_channel);
  }
  int ix = analog_index_lookup( analog_channel);
  if (ix >= 0) {
    analog_channels[ ix].callback       = ptemp_update;
    analog_channels[ ix].update_channel = update_channel;
    return 0;
  }
  if (debug_flags & DEBUG_ANALOG) {
    fprintf( stderr, "analog_set_update_callback failed for '%s'\n", tag_name( analog_channel));
  }
  return -1;
}

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
  fd = calloc( num_analog_channels, sizeof( *fd));
  for (i = 0 ; i < num_analog_channels ; ++i) {
    ret = fd[ i] = open( analog_channels[ i].device_path, O_RDONLY);
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
    if (num_analog_channels == 0) {
      usleep( ANALOG_CYCLE_TIME);
      continue;
    }
    usleep( ANALOG_CYCLE_TIME / num_analog_channels);
    ret = read( fd[ i], buf, sizeof( buf));
    if (ret > 0) {
      int val = atoi( buf);
      struct analog_channel_record* p = &analog_channels[ i];
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
#ifdef BBB
      /* FIXME: for now ignore these recoverable errors! */
      if (debug_flags & DEBUG_ANALOG) {
        fprintf( stderr, "analog read failed: fd=%d, buf=%p, size=%u, ret=%d\n", fd[ i], buf, sizeof( buf), ret);
      }
      continue;
#else
      perror( "analog thread: ADC read failed -");
      goto failure;
#endif
    }
    ++i;
    if (i == num_analog_channels) {
      // Once every this often, push values to clients
      if (++cycle >= ANALOG_UPDATE_CYCLE_TIME / ANALOG_CYCLE_TIME) {
        unsigned int ch;
        for (ch = 0 ; ch < num_analog_channels ; ++ch) {
          struct analog_channel_record* p = &analog_channels[ ch];
          if (p->callback != NULL) {
            if (debug_flags & DEBUG_ANALOG) {
              fprintf( stderr, "analog_worker, calling temp_update for %s with value %d\n",
                      p->update_channel, p->value);
            }
            (void) (p->callback)( p->update_channel, p->value);
          }
        }
        if (debug_flags & DEBUG_ANALOG) {
          int ch;
          printf( "ADC values:");
          for (ch = 0 ; ch < num_analog_channels ; ++ch) {
            printf( " avg[ %d]=%d", ch, analog_channels[ ch].value);
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
static analog_config_record* analog_config_data = NULL;
static int analog_config_items = 0;

int analog_config( analog_config_record* config_data, int nr_config_items)
{
  if (debug_flags & DEBUG_ANALOG) {
    printf( "analog_config called with %d records'\n", nr_config_items);
  }
  analog_config_data  = config_data;
  analog_config_items = nr_config_items;
  // this mustn't be called more than once, so keep the code simple
  analog_channels     = calloc( nr_config_items, sizeof( struct analog_channel_record));
  num_analog_channels = 0;
  return 0;
}


static pthread_t worker;

int analog_init( void)
{
  unsigned int ch;
  if (debug_flags & DEBUG_ANALOG) {
    printf( "analog_init called'\n");
  }
  if (analog_config_data) {
    for (ch = 0 ; ch < analog_config_items ; ++ch) {
      analog_config_record*         ps = &analog_config_data[ ch];
      struct analog_channel_record* pd = &analog_channels[ ch];

      pd->id                = ps->tag;
      pd->device_path       = ps->device_path;
      pd->filter_length     = ps->filter_length;
      pd->callback          = NULL;
      pd->value             = 0;
      pd->average.count     = 0;
      pd->average.value     = 0;
      pd->average.remainder = 0;
      ++num_analog_channels;
    }
    if (mendel_thread_create( "analog", &worker, NULL, &analog_worker, NULL) != 0) {
      return -1;
    }
    struct sched_param param = {
      .sched_priority = ANALOG_PRIO
    };
    pthread_setschedparam( worker, ANALOG_SCHED, &param);
    return 0;
  }
  fprintf( stderr, "analog_init: no configuration data!\n");
  return -1;
}

int analog_get_raw_value( channel_tag analog_channel, int* pvalue)
{
  if (pvalue != NULL) {
    int ix = analog_index_lookup( analog_channel);
    if (ix >= 0) {
      // TODO: lock?
      *pvalue = analog_channels[ ix].value;
      // TODO: unlock?
      return 0;
    }
  }
  return -1;
}
