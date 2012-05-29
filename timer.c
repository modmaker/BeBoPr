#include	"timer.h"

/** \file
	\brief Timer management - step pulse clock and system clock

	Teacup uses timer1 to generate both step pulse clock and system clock.

	We achieve this by using the output compare registers to generate the two clocks while the timer free-runs.

	Teacup has tried numerous timer management methods, and this is the best so far.
*/

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>

#include	"platform.h"
#include	"config.h"

#ifdef	HOST
#include	"dda_queue.h"
#endif

#include	"memory_barrier.h"

/// how often we overflow and update our clock; set to nanoseconds on ARM
#define		TICK_TIME			2UL MS
/// convert back to ms from cpu ticks so our system clock runs properly if you change TICK_TIME
#define		TICK_TIME_MS	(TICK_TIME / (1000 * 1000))

/// time until next step, as output compare register is too small for long step times
uint32_t	next_step_time;

/// every time our clock fires, we increment this so we know when 10ms has elapsed
uint8_t						clock_counter_10ms = 0;
/// keep track of when 250ms has elapsed
uint8_t						clock_counter_250ms = 0;
/// keep track of when 1s has elapsed
uint8_t						clock_counter_1s = 0;

/// flags to tell main loop when above have elapsed
volatile uint8_t	clock_flag_10ms = 0;
volatile uint8_t	clock_flag_250ms = 0;
volatile uint8_t	clock_flag_1s = 0;

volatile uint8_t	timer1_compa_deferred_enable = 0;

//==================  A R M / L I N U X   P L A T F O R M  ==================

static pthread_t worker;
static const struct timespec sleep_cycletime = {
	.tv_sec  = (TICK_TIME) / 1000000000UL,
	.tv_nsec = (TICK_TIME) % 1000000000UL
};

void* timer_thread( void* arg)
{
	struct timespec sleep_request;
	struct timespec sleep_remaining;
	printf( "timer_thread initializing...\n");
	fprintf( stderr, "timer_thread initializing...\n");
	sleep_request = sleep_cycletime;
	for (;;) {
		int result = clock_nanosleep( CLOCK_MONOTONIC, 0, &sleep_request, &sleep_remaining);
		if (result == 0) {
			// update stuff
			/*
			  clock stuff
			*/
			clock_counter_10ms += TICK_TIME_MS;
			if (clock_counter_10ms >= 10) {
				clock_counter_10ms -= 10;
				clock_flag_10ms = 1;
				
				clock_counter_250ms += 1;
				if (clock_counter_250ms >= 25) {
					clock_counter_250ms -= 25;
					clock_flag_250ms = 1;
					
					clock_counter_1s += 1;
					if (clock_counter_1s >= 4) {
						clock_counter_1s -= 4;
						clock_flag_1s = 1;
					}
				}
			}
			sleep_request = sleep_cycletime;
		} else if (result == EINTR) {
			// resume sleep with remaining time
			sleep_request = sleep_remaining;
		} else {
			fprintf( stderr, "unexpected result from clock_nanosleep: %d\n", result);
			pthread_exit( NULL);
		}
	}
}



/// initialise timer and enable system clock interrupt.
/// step interrupt is enabled later when we start using it
void timer_init()
{
	printf( "Timer cycletime = %ld.%09ld s.\n", sleep_cycletime.tv_sec, sleep_cycletime.tv_nsec);
	printf( "TICK_TIME_MS = %lu.\n", TICK_TIME_MS);
	fprintf( stderr, "Creating timer thread...");
	int result = pthread_create( &worker, NULL, &timer_thread, NULL);
	if (result == 0) {
		fprintf( stderr, "ok, ");
	} else {
		fprintf( stderr, "failed with error=%d\n", result);
		exit( EXIT_FAILURE);
	}
	fprintf( stderr, "waiting for response...");
	do {} while (clock_flag_10ms == 0);
	clock_flag_10ms = 0;
	fprintf( stderr, "10ms...");
	do {} while (clock_flag_250ms == 0);
	clock_flag_250ms = 0;
	fprintf( stderr, "250ms...");
	do {} while (clock_flag_1s == 0);
	clock_flag_1s = 0;
	fprintf( stderr, "1s...");
	fprintf( stderr, "done\n");
}

#ifdef	HOST
/// specify how long until the step timer should fire
void setTimer(uint32_t delay)
{
  if (delay > 0) {
    // arm timer
  } else {
    // disable timer
  }
}

/// stop timers - emergency stop
void timer_stop() {
}
#endif /* ifdef HOST */
