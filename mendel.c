/** \file
	\brief Main file - this is where it all starts, and ends
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>

#include "config.h"
#include "serial.h"
#include "heater.h"
#include "bebopr.h"
#include "mendel.h"
#include "gcode_process.h"
#include "gcode_parse.h"
#include "limit_switches.h"
#include "pruss.h"


static int arm_init( void)
{
  if (geteuid() != 0) {
    // Only root can set scheduling parameters, not running as root
    // this may create all kinds of problems later on, so bail out!
    fprintf( stderr, "FATAL ERROR - This program can only be run as root!\n");
    exit( EXIT_FAILURE);
  }

  int policy = SCHED_OTHER;
  int prio_max = sched_get_priority_max( policy);
  int prio_min = sched_get_priority_min( policy);

  struct sched_param sp = { 
    .sched_priority = (prio_min + prio_max) / 2
  };
  // Set realtime process scheduling using the Round Robin scheduler
  // with absolute priority halfway between min and max.
  if (sched_setscheduler( 0, policy, &sp) < 0) {
  }
  fprintf( stderr, "Scheduler set to %d, priority to %d\n", policy, sp.sched_priority);

  struct timespec clock_resolution;
  clock_getres( CLOCK_MONOTONIC, &clock_resolution);
  if (clock_resolution.tv_sec == 0) {
    printf( "Clock resolution = %ld ns.\n", clock_resolution.tv_nsec);
  } else {
    printf( "Clock resolution = %ld.%09ld s.\n", clock_resolution.tv_sec, clock_resolution.tv_nsec);
  }
  return 0;
}

/// Startup code, run when we come out of reset
void init(void) {

  // configure
  mendel_sub_init( "bebopr", bebopr_pre_init);

  // set up arm & linux specific stuff
  mendel_sub_init( "arm", arm_init);

  // set up limit switches
  mendel_sub_init( "limsw", limsw_init);

  // set up serial communication
  mendel_sub_init( "serial", serial_init);

  // This initializes the complete analog subsystem!
  mendel_sub_init( "heater", heater_init);

  // This initializes the trajectory code and PRUSS
  mendel_sub_init( "gcode_process", gcode_process_init);

	// set up dda

  // say hi to host
  serial_writestr_P( "start\nok\n");
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
  fprintf( stderr, "Starting %s_init() ...\n", name);
  int result = subsys();
  if (result != 0) {
    fprintf( stderr, "... %s_init() failed with code %d\n", name, result);
  } else {
    fprintf( stderr, "... %s_init() was successfull\n", name);
  }
  return result;
}

/// this is where it all starts, and ends
///
/// just run init() that starts all threads, then run an endless loop where we pass characters from the serial RX buffer to gcode_parse_char()
// FIXME: This can now also be programmed as a (blocking) thread?
// FIXME: Implement proper program termination and un-init functions.
int main (void)
{
  int block = 0;
  init();
  fprintf( stderr, "Starting main loop...\n");

  for (;;) {
    if (pruss_queue_full()) {
      if (block) {
        usleep( 1000);
      } else {
        fprintf( stderr, "<queue_full>");
        block = 1;
      }
    } else {
      block = 0;
      if (serial_rxchars() > 0) {
        gcode_parse_char( serial_popchar());
      } else {
	sched_yield();
      }
    }
  }
}
// -*- indent-tabs-mode: nil; tab-width: 4; c-basic-offset: 4; -*-
// ex:ts=4:sw=4:sts=4:et
