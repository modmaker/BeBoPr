/** \file
	\brief Serial subsystem

	Teacup's serial subsystem is a powerful, thoroughly tested and highly modular serial management system.

	It uses ringbuffers for both transmit and receive, and intelligently decides whether to wait or drop transmitted characters if the buffer is full.

	It also supports XON/XOFF flow control of the receive buffer, to help avoid overruns.
*/

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>

#include "serial.h"
#include "mendel.h"

//==================  A R M / L I N U X   P L A T F O R M  ==================

static pthread_t worker;

static char input_buffer[ 100];

// We're using only an in pointer and an out pointer (no count). This way
// there can't be an inconsistent state and we don't need to lock.
// (As long as we're programming correctly...)
// Note that this costs us one buffer space. I.e. we can't fill the buffer
// completely but have to leave one empty space or we can't differentiate
// between a full and an empty buffer.
// TODO: another concern is the atomic update of the in_ptr and out_ptr variables!
static volatile unsigned int in_ptr;	// only modified by writer
static volatile unsigned int out_ptr;	// only modified by reader

#define debug 0

void* serial_input( void* arg)
{
  // read input and put it in a circular buffer
  unsigned int pout;
  unsigned int pin;
  //  int count;
  //  serial_writestr( "Serial thread - initializing\n");
  fprintf( stderr, "Serial thread - initializing\n");
  for (;;) {
    pout = out_ptr;
    pin  = in_ptr;
    if ((pout - pin) >= 0 || (pout + sizeof( input_buffer) - pin) > 1) {
      int c = getchar();
      if (c != EOF) {
	fprintf( stderr, "Serial thread - putting char 0x%02x at in_ptr = %d, out_ptr = %d\n", c, pin, pout);
	input_buffer[ pin] = (char) c;
	if (++pin < sizeof( input_buffer)) {
	  in_ptr = pin;
	} else {
	  in_ptr = 0;
	}
      }
//      if (debug) {
//      fprintf( stderr, "Serial thread - putting char 0x%02x at in_ptr = %d, out_ptr = %d\n", c, pin, pout);
//      }
    }
    sched_yield();
  }
  return (void*)-1;
}

// TODO: Think about implementation of network link / socket (ssh)...

// initialise serial subsystem
int serial_init( void)
{
  // open serial port and set communication parameters (baud,parity,stop,flow-control,etc.)
  // NOPE: we're just using stdio...
  in_ptr = 0; //sizeof( input_buffer);
  out_ptr = 0;

  //  fcntl( 0, F_SETFL, fcntl( 0, F_GETFL) | O_NONBLOCK);
  int result = mendel_thread_create( "serial", &worker, NULL, &serial_input, NULL);
  return result;
}

// return number of characters in the receive buffer, and number of spaces in the send buffer
uint32_t serial_rxchars( void)
{
  // copy vars to prevent changes while using them
  // so we don't need to lock.
  uint32_t pin  = in_ptr;
  uint32_t pout = out_ptr;
  if (pout > pin) {
    pin += sizeof( input_buffer);
  }
#if 0
  if (pin > pout) {
    printf( "serial_rxchars returns count %d\n", pin - pout);
  }
#endif
  return pin - pout;
}

// read one character
uint8_t serial_popchar( void)
{
  int pin  = in_ptr;
  int pout = out_ptr;
  // If both pointers are the same, the buffer is empty.
  while (pin == pout) {
    // wait
    if (sched_yield() == -1) {
      perror( "Reader: cannot yield the processor");
    }
    pin  = in_ptr;
    pout = out_ptr;
  }
  uint8_t c = input_buffer[ pout];
  // Don't use out_ptr for this calculation as it may
  // hold an invalid value (briefly).
  pout += 1;
  if (pout >= sizeof( input_buffer)) {
    pout = 0;
  }
  out_ptr = pout;
  if (debug) fprintf( stderr, "Reader thread - exit with in_ptr = %d, out_ptr = %d\n", in_ptr, out_ptr);
  return c;
}
