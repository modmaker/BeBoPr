#include <stdlib.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include "heater.h"
#include "debug.h"
#include "temp.h"
#include "beaglebone.h"
#include "mendel.h"


struct heater {
  channel_tag		id;
  channel_tag		input;
  channel_tag		output;
  double		setpoint;
  pid_settings		pid_settings;
  double		(*get_temperature)( void);
  double		pid_integral;
  double		celsius_history[ 8];
  unsigned int          history_ix;
  int			log_fd;
};

static struct heater* heaters = NULL;
static unsigned int num_heater_channels = 0;

static int heater_index_lookup( channel_tag heater_channel)
{
  for (int ix = 0 ; ix < num_heater_channels ; ++ix) {
    if (heaters[ ix].id == heater_channel) {
      return ix;
    }
  }
  if (debug_flags & DEBUG_HEATER) {
    fprintf( stderr, "heater_index_lookup failed for '%s'\n", tag_name( heater_channel));
  }
  return -1;
}

static double clip( double min, double val, double max)
{
  if (val < min) {
    return min;
  } else if (val > max) {
    return max;
  }
  return val;
}

// Use control_lock for access to control loop settings
static pthread_rwlock_t	control_lock;

static int log_file_open( const char* fname)
{
  char s[ 270];
  snprintf( s, sizeof( s), "./pid-%s.log", fname);
  if (debug_flags & DEBUG_HEATER) {
    printf( "log_file_open - looking for existing logfile named '%s'\n", s);
  }
  int fd = open( s, O_WRONLY|O_APPEND);
  if (fd < 0) {
    perror( "Failed to open logile for append, logging disabled");
    return -1;
  }
  if (debug_flags & DEBUG_HEATER) {
    printf( "log_file_open - appending to file '%s'\n", s);
  }
  snprintf( s, sizeof( s), "--------------------------------------------------------------------------------------\n"
                           "   time   channel         setpoint   temp       ff       p        i        d     out%%\n"
                           "--------------------------------------------------------------------------------------\n");
  write( fd, s, strlen( s));
  return fd;
}

static void log_entry( const char* name, int fd, time_t time, double setpoint, double celsius, double error,
                double out_ff, double out_p, double out_i, double out_d, int duty_cycle)
{
  char s[ 120];
  snprintf( s, sizeof( s), "%7ld   %-14s   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %3d\n",
          time, name, setpoint, celsius, out_ff, out_p, out_i, out_d, duty_cycle);
  write( fd, s, strlen( s));
  if (debug_flags & DEBUG_HEATER) {
    printf( s);
  }
}

#define PID_LOOP_FREQUENCY	5 /* Hz */
#define TIMER_CLOCK CLOCK_MONOTONIC
#define NS_PER_SEC  (1000*1000*1000)

static void ns_sleep( struct timespec* ts, unsigned int ns)
{
  struct timespec tso;
  ts->tv_nsec += ns;
  if (ts->tv_nsec >= NS_PER_SEC) {
    ts->tv_nsec -= NS_PER_SEC;
    ts->tv_sec  += 1;
  }
  for (;;) {
    if (clock_nanosleep( TIMER_CLOCK, TIMER_ABSTIME, ts, &tso) == -1) {
      if (errno == EINTR) {
        printf( "  sleep interrupted, tso = %ld.%09ld\n", tso.tv_sec, tso.tv_nsec);
        *ts = tso;
      } else {
        perror( "clock_nanoslaap failed");
	break;
      }
    } else {
      break;
    }
  }
}

/*
 * This is the worker thread that controls the heaters
 * depending on the setpoint and temperature measured.
 */
void* heater_thread( void* arg)
{
  struct timespec ts;
  unsigned int timer_period;
  int log_scaler;
  if (num_heater_channels < 1) {
    // nothing to do !
    pthread_exit( NULL);
  }
  timer_period = NS_PER_SEC / (PID_LOOP_FREQUENCY * num_heater_channels);
  fprintf( stderr, "heater_thread: started\n");
  clock_getres( TIMER_CLOCK, &ts); 
  printf( "  timer resolution is %ld [ns]\n", ts.tv_nsec);
  clock_gettime( TIMER_CLOCK, &ts);
  log_scaler = 0;
  while (1) {
    for (int ix = 0 ; ix < num_heater_channels ; ++ix) {
      struct heater* p = &heaters[ ix];
      channel_tag input_channel  = p->input;
      channel_tag output_channel = p->output;
      double celsius;
      // Sleep until the next mark passes, distribute the load
      ns_sleep( &ts, timer_period);
      if (temp_get_celsius( input_channel, &celsius) < 0) {
        fprintf( stderr, "heater_thread - failed to read temperature from '%s'\n", tag_name( input_channel));
      } else {
        if (p->setpoint == 0.0) {
          // A setpoint of 0.0 means: disable heater
          // TODO: should this be done over and over again ?
          pwm_set_output( output_channel, 0);
        } else {
          double t_error = p->setpoint - celsius;
          const double dt = 1.0 / PID_LOOP_FREQUENCY;

          // proportional part
          double heater_p = t_error;

          // integral part (prevent integrator wind-up)
          double heater_i = clip( -p->pid_settings.I_limit,
                                   p->pid_integral + t_error * dt,
                                   p->pid_settings.I_limit);
          p->pid_integral = heater_i;

          // derivative (note: D follows temp rather than error so there's
          // no large derivative when the target changes)
          p->celsius_history[ p->history_ix] = celsius;
          if (++(p->history_ix) >= NR_ITEMS( p->celsius_history)) {
            p->history_ix = 0;
          }
          double old_celsius = p->celsius_history[ p->history_ix];
          if (old_celsius == 0.0) {
            // if no history, force heater_d to 0.0
            old_celsius = celsius;
          }
          double heater_d = (celsius - old_celsius) / ((NR_ITEMS( p->celsius_history) - 1) * dt);

          // combine factors
          double out_p = heater_p * p->pid_settings.P;
          double out_i = heater_i * p->pid_settings.I;
          double out_d = heater_d * p->pid_settings.D;
          double out_ff= (p->setpoint - p->pid_settings.FF_offset) * p->pid_settings.FF_factor;
          double out   = out_p + out_i + out_d + out_ff;
          int duty_cycle = (int) clip( 0.0, out, 100.0);
          // Do not log every cycle, but only once in a while
          if (log_scaler == 0) {
            log_entry( tag_name( input_channel), p->log_fd, ts.tv_sec,
		    p->setpoint, celsius, t_error, out_ff, out_p, out_i, out_d, duty_cycle);
          }
          pwm_set_output( output_channel, duty_cycle);
        }
      }
    }
    // create a log entry every second
    if (++log_scaler >= 1 * PID_LOOP_FREQUENCY) {
      log_scaler = 0;
    }
  }
}

static pthread_t worker;

/*
 * Configuration settings are stored seperately (bebopr_rx.c) and
 * a configuration call is used to communicate these with this
 * code.
 */
static heater_config_record* heater_config_data = NULL;
static int heater_config_items = 0;

int heater_config( heater_config_record* config_data, int nr_config_items)
{
  heater_config_data  = config_data;
  heater_config_items = nr_config_items;
  heaters = calloc( nr_config_items, sizeof( struct heater));
  num_heater_channels = 0;
  return 0;
}

void heater_exit( void)
{
  pwm_exit();
//  temp_exit();
}

/*
 * set defaults, init pwm hardware, set heater state to off.
 * create the heaters thread
 * 
 */
int heater_init( void)
{
  if (heater_config_data != NULL) {
    atexit( heater_exit);
    // First initialize input and output subsystems
    mendel_sub_init( "temp", temp_init);
    mendel_sub_init( "pwm", pwm_init);
    // No need to lock as there's no thread running yet!
    for (int ch = 0 ; ch < heater_config_items ; ++ch) {
      struct heater*	    pd  = &heaters[ ch];
      heater_config_record* ps  = &heater_config_data[ ch];
      pd->id			= ps->tag;
      pd->input			= ps->analog_input;
      pd->output		= ps->analog_output;
      pd->pid_settings.P	= ps->pid.P;
      pd->pid_settings.I	= ps->pid.I;
      pd->pid_settings.D	= ps->pid.D;
      pd->pid_settings.FF_factor= ps->pid.FF_factor;
      pd->pid_settings.FF_offset= ps->pid.FF_offset;
      pd->pid_settings.I_limit	= ps->pid.I_limit;
      pd->setpoint		= 0.0;
      pd->history_ix		= 0;
      pd->pid_integral		= 0.0;
      pd->log_fd		= -1;
      ++num_heater_channels;
    }
    // Start worker thread
    if (mendel_thread_create( "heater", &worker, NULL, &heater_thread, NULL) != 0) {
      return -1;
    }
    struct sched_param param = {
      .sched_priority = HEATER_PRIO
    };
    pthread_setschedparam( worker, HEATER_SCHED, &param);
    return 0;
  }
  fprintf( stderr, "temp_init: no configuration data!\n");
  return -1;
}

/*
 * get current temperature for a heater
 */
int heater_get_celsius( channel_tag heater, double* pcelsius)
{
  if (pcelsius != NULL) {
    int ix = heater_index_lookup( heater);
    if (ix >= 0) {
      return temp_get_celsius( heaters[ ix].input, pcelsius);
    }
  }
  return -1;
}
/*
 * turn heater channel on or off
 */
int heater_enable( channel_tag heater, int state)
{
  return -1;
}

/*
 * set heater pwm output for given channel to value
 */
int heater_set_raw_pwm( channel_tag heater, double percentage)
{
  return -1;
}

/*
 * set setpoint for a heater
 */
int heater_set_setpoint( channel_tag heater, double setpoint)
{
  int ix = heater_index_lookup( heater);
  if (ix >= 0) {
    struct heater* p = &heaters[ ix];
    pthread_rwlock_wrlock( &control_lock);
    p->setpoint = setpoint;
    pthread_rwlock_unlock( &control_lock);
    if (setpoint == 0.0) {
      // stop logging & close file if open
      if (p->log_fd >= 0) {
        close( p->log_fd);
        if (debug_flags & DEBUG_HEATER) {
          printf( "heater_set_setpoint - logfile closed.\n");
        }
      }
      p->log_fd = -1;
    } else {
      if (p->log_fd == -1) {
        p->log_fd = log_file_open( tag_name( p->id));
      }
    }
    /*
     * Activate setpoint and in-range watch in temperature code
     * TODO: improve settings / add setting for limits
     */
    temp_set_setpoint( p->input, setpoint, -2.5, 1.5);
    return 0;
  }
  return -1;
}

/*
 * get setpoint for a heater
 */
int heater_get_setpoint( channel_tag heater, double* setpoint)
{
  if (setpoint != NULL) {
    int ix = heater_index_lookup( heater);
    if (ix >= 0) {
      pthread_rwlock_rdlock( &control_lock);
      *setpoint = heaters[ ix].setpoint;
      pthread_rwlock_unlock( &control_lock);
      return 0;
    }
  }
  return -1;
}

/*
 * set PID values for a heater
 */
int heater_set_pid_values( channel_tag heater, const pid_settings* pid_settings)
{
  int ix = heater_index_lookup( heater);
  if (ix >= 0) {
    pthread_rwlock_wrlock( &control_lock);
    heaters[ ix].pid_settings = *pid_settings;
    pthread_rwlock_unlock( &control_lock);
    return 0;
  }
  return -1;
}

/*
 * get PID values for a heater
 */
int heater_get_pid_values( channel_tag heater, pid_settings* pid_settings)
{
  if (pid_settings != NULL) {
    int ix = heater_index_lookup( heater);
    if (ix >= 0) {
      pthread_rwlock_rdlock( &control_lock);
      *pid_settings = heaters[ ix].pid_settings;
      pthread_rwlock_unlock( &control_lock);
      return 0;
    }
  }
  return -1;
}

const char* fname = "./heater-pid-factors";

/*
 * Write PID factors to persistent storage
 */
int heater_save_settings( void)
{
  int fd;
  int ret;
  
  fd = open( fname, O_WRONLY);
  if (fd < 0) {
    perror( "heater: opening of file '%s' failed");
    return -1;
  }
  pthread_rwlock_rdlock( &control_lock);
  ret = write( fd, heaters, sizeof( heaters));
  pthread_rwlock_unlock( &control_lock);
  if (ret < 0) {
    perror( "heater: writing file '%s' failed");
    return ret;
  } else if (ret != sizeof( heaters)) {
    return -1;
  }
  return 0;
}


/*
 * Read PID factors from persistent storage
 */
int heater_load_settings( void)
{
  int fd;
  int ret;
  
  fd = open( fname, O_WRONLY);
  if (fd < 0) {
    perror( "heater: opening of file '%s' failed");
    return -1;
  }
  pthread_rwlock_wrlock( &control_lock);
  ret = read( fd, heaters, sizeof( heaters));
  pthread_rwlock_unlock( &control_lock);
  if (ret < 0) {
    perror( "heater: reading file '%s' failed");
    return ret;
  } else if (ret != sizeof( heaters)) {
    return -1;
  }
  return 0;
}

channel_tag heater_lookup_by_name( const char* name)
{
  for (int ix = 0 ; ix < num_heater_channels ; ++ix) {
    channel_tag tag = heaters[ ix].id;
    if (strcmp( tag_name( tag), name) == 0) {
      return tag;
    }
  }
  return NULL;
}

int heater_temp_reached( channel_tag heater)
{
  if (heater) {
    int ix = heater_index_lookup( heater);
    if (ix >= 0) {
      return temp_achieved( heaters[ ix].input);
    }
    return 0;
  }
  return 1;	// dummy heater has always the right temperature
}

