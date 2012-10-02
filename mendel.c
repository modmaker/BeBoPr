/** \file
	\brief Main file - this is where it all starts, and ends
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>

#include "heater.h"
#include "bebopr.h"
#include "mendel.h"
#include "gcode_process.h"
#include "gcode_parse.h"
#include "limit_switches.h"
#include "pruss_stepper.h"
#include "comm.h"
#include "debug.h"


static int arm_init( void)
{
  if (geteuid() != 0) {
    // Only root can set scheduling parameters, not running as root
    // this may create all kinds of problems later on, so bail out!
    fprintf( stderr, "FATAL ERROR - This program can only be run as root!\n");
    exit( EXIT_FAILURE);
  }

  int policy = MENDEL_SCHED;
  int prio_max = sched_get_priority_max( policy);
  int prio_min = sched_get_priority_min( policy);

  struct sched_param sp = { 
    .sched_priority = MENDEL_PRIO
  };
  // Set realtime process scheduling using the Round Robin scheduler
  // with absolute priority halfway between min and max.
  if (sched_setscheduler( 0, policy, &sp) < 0) {
  }
  fprintf( stderr, "Scheduler set to %d, priority to %d (min:%d,max:%d)\n", policy, sp.sched_priority, prio_min, prio_max);

  struct timespec clock_resolution;
  clock_getres( CLOCK_MONOTONIC, &clock_resolution);
  if (clock_resolution.tv_sec == 0) {
    printf( "Clock resolution = %ld ns.\n", clock_resolution.tv_nsec);
  } else {
    printf( "Clock resolution = %ld.%09ld s.\n", clock_resolution.tv_sec, clock_resolution.tv_nsec);
  }
  return 0;
}

static void signal_handler( int signal)
{
  fprintf( stderr, "Terminating on signal %d\n", signal);
  exit( EXIT_SUCCESS);
}

/// Startup code, run when we come out of reset
int init( void)
{
  int result;

  signal( SIGINT, signal_handler);
  signal( SIGHUP, signal_handler);
  signal( SIGTERM, signal_handler);

  // set up arm & linux specific stuff
  result = mendel_sub_init( "arm", arm_init);
  if (result != 0) {
    return result;
  }
  // configure
  result = mendel_sub_init( "bebopr (early)", bebopr_pre_init);
  if (result != 0) {
    return result;
  }
  // keep connection alive
  result = mendel_sub_init( "comm", comm_init);
  if (result != 0) {
    return result;
  }
  // set up limit switches
  result = mendel_sub_init( "limsw", limsw_init);
  if (result != 0) {
    return result;
  }
  // This initializes the complete analog subsystem!
  result = mendel_sub_init( "heater", heater_init);
  if (result != 0) {
    return result;
  }
  // enable i/o power
  result = mendel_sub_init( "bebopr (late)", bebopr_post_init);
  if (result != 0) {
    return result;
  }
  // This initializes the trajectory code and PRUSS
  result = mendel_sub_init( "gcode_process", gcode_process_init);
  if (result != 0) {
    return result;
  }
  return 0;
}

int mendel_thread_create( const char* name, pthread_t* restrict thread, const pthread_attr_t* restrict attr,
			  void* (*worker_thread)( void*), void* restrict arg)
{
  fprintf( stderr, "=== Creating %s_thread...", name);
  int result = pthread_create( thread, attr, worker_thread, arg);
  if (result == 0) {
    fprintf( stderr, "done ===\n");
    usleep( 1000); //    sched_yield();
  } else {
    fprintf( stderr, "failed with error=%d ===\n", result);
  }
  return result;
}

int mendel_sub_init( const char* name, int (*subsys)( void))
{
  fprintf( stderr, "Starting '%s' init ...\n", name);
  int result = subsys();
  if (result != 0) {
    fprintf( stderr, "... '%s' init failed with code %d\n", name, result);
  } else {
    fprintf( stderr, "... '%s' init was successfull\n", name);
  }
  return result;
}

void mendel_exit( void)
{
  fprintf( stderr, "mendel_exit called, waiting for output buffers to be flushed\n");
  usleep( 2 * 1000000);
}

/// this is where it all starts, and ends
///
/// just run init() that starts all threads, then run an endless loop where we pass characters from the serial RX buffer to gcode_parse_char()
// FIXME: This can now also be programmed as a (blocking) thread?
// FIXME: Implement proper program termination and un-init functions.
int main ( int argc, const char* argv[])
{
  extern const char* mendel_date;
  extern const int   mendel_version;
  extern const int   mendel_build;
  fprintf( stderr, "Starting '%s' version %d.%d.%d (%s).\n",
	  argv[ 0], mendel_version, FW_VERSION, mendel_build, mendel_date);
  if (debug_flags) {
    fprintf( stderr, "Starting with debug flags set to 0x%04x (%u)\n",
	    debug_flags, debug_flags);
  }
  if (init() != 0) {
    fprintf( stderr, "Initialization failed, terminating.\n");
    exit( EXIT_FAILURE);
  }

  atexit( mendel_exit);

  // say hi to host
  printf( "start\nok\n");
  fprintf( stderr, "Starting main loop...\n");

  for (;;) {
    char s[ 100];

    if (fgets( s, sizeof( s), stdin) == NULL) {
      fprintf( stderr, "main loop - EOF on input, terminating.\n");
      exit( EXIT_SUCCESS);
    } else {
      char* p = s;
      while (*p) {
        gcode_parse_char( *p++);
      }
    }
  }
}

// -*- indent-tabs-mode: nil; tab-width: 4; c-basic-offset: 4; -*-
// ex:ts=4:sw=4:sts=4:et
