/** \file
	\brief Main file - this is where it all starts, and ends
*/

/** \mainpage Teacup Reprap Firmware
	\section intro_sec Introduction
		Teacup Reprap Firmware (originally named FiveD on Arduino) is a firmware package for numerous reprap electronics sets.

		Please see README for a full introduction and long-winded waffle about this project
	\section install_sec	Installation
		\subsection step1 Step 1: Download
			\code git clone git://github.com/triffid/Teacup_Firmware \endcode
		\subsection step2 Step 2: configure
			\code cp config.[yourboardhere].h config.h \endcode
			Edit config.h to suit your machone
			Edit Makefile to select the correct chip and programming settings
		\subsection step3 Step 3: Compile
			\code make \endcode
			\code make program \endcode
		\subsection step4 Step 4: Test!
			\code ./func.sh mendel_reset
			./func.sh mendel_talk
			M115
			ctrl+d \endcode
*/

#include	<stdio.h>
#include	<stdlib.h>
#include	<unistd.h>
#include	<time.h>
#include	<sched.h>
#include 	<pthread.h>

#include	"config.h"

#include	"serial.h"
#include	"dda_queue.h"
#include	"dda.h"
#include	"gcode_parse.h"
#include	"temp.h"
#include	"sermsg.h"
#include	"debug.h"
#include	"sersendf.h"
#include	"heater.h"
#include	"analog.h"
#include	"pinio.h"
#include	"platform.h"
#include	"clock.h"
#include	"bebopr.h"
#include	"mendel.h"
#include	"gcode_process.h"
#include	"limit_switches.h"

/// initialise all I/O - set pins as input or output, turn off unused subsystems, etc
void io_init(void) {
}

void arm_init( void)
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
  gcode_process_init();
}

/// Startup code, run when we come out of reset
void init(void) {

  bebopr_pre_init();

  arm_init();
  fprintf( stderr, "<arm-init done>");

	// set up watchdog
	wd_init();
	fprintf( stderr, "<wd-init done>");
	// set up limit switches
	mendel_sub_init( "limsw", limsw_init);

	// set up serial communication
	mendel_sub_init( "serial", serial_init);

	// set up inputs and outputs
	io_init();
	fprintf( stderr, "<io-init done>");

	// set up timers
//	timer_init();
//	fprintf( stderr, "<timer-init done>");

	// set up dda
	dda_init();
	fprintf( stderr, "<dda-init done>");

	// enable interrupts
	sei();
	fprintf( stderr, "<sei done>");

	// reset watchdog
	wd_reset();
	fprintf( stderr, "<wd_reset done>");

	mendel_sub_init( "heater", heater_init);

	// say hi to host
	serial_writestr_P(PSTR("start\nok\n"));
}

int mendel_thread_create( const char* name, pthread_t* restrict thread, const pthread_attr_t* restrict attr,
			  void* (*worker_thread)( void*), void* restrict arg)
{
  fprintf( stderr, "=== Creating %s_thread...", name);
  int result = pthread_create( thread, attr, worker_thread, arg);
  if (result == 0) {
    fprintf( stderr, "done ===\n");
    usleep( 1000);
    //    sched_yield();
  } else {
    fprintf( stderr, "failed with error=%d ===\n", result);
  }
  return result;
}

int mendel_sub_init( const char* name, int (*subsys)( void))
{
  int result = subsys();
  if (result != 0) {
    fprintf( stderr, "%s_init() failed with code %d\n", name, result);
  } else {
    fprintf( stderr, "%s_init() was successfull\n", name);
  }
  return result;
}


/// this is where it all starts, and ends
///
/// just run init(), then run an endless loop where we pass characters from the serial RX buffer to gcode_parse_char() and check the clocks
int main (void)
{
  fprintf( stderr, "<pre-init>");
	init();
  fprintf( stderr, "<post-init>");

	// main loop
	for (;;) {
		if (dda_queue_full()) {
			fprintf( stderr, "<queue_full>");
			// if queue is full, no point in reading chars- host will just have to wait
		} else if (serial_rxchars() > 0) {
			uint8_t c = serial_popchar();
			gcode_parse_char( c);
		}
		usleep( 1000);
		sched_yield();
	}
}
// -*- indent-tabs-mode: nil; tab-width: 4; c-basic-offset: 4; -*-
// ex:ts=4:sw=4:sts=4:et
