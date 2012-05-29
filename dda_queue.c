#include	"dda_queue.h"

/** \file
	\brief DDA Queue - manage the move queue
*/

#include	<string.h>
#include	<sched.h>
#include	<stdio.h>

#include	"config.h"
#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"temp.h"
#include	"sersendf.h"
#include	"clock.h"
#include	"memory_barrier.h"
#include	"pruss.h"

/// movebuffer head pointer. Points to the last move in the queue.
/// this variable is used both in and out of interrupts, but is
/// only written outside of interrupts.
static unsigned	mb_head = 0;

/// movebuffer tail pointer. Points to the currently executing move
/// this variable is read/written both in and out of interrupts.
static unsigned	mb_tail = 0;

/// move buffer.
/// holds move queue
/// contents are read/written both in and out of interrupts, but
/// once writing starts in interrupts on a specific slot, the
/// slot will only be modified in interrupts until the slot is
/// is no longer live.
/// The size does not need to be a power of 2 anymore!
#if ARCH == arm
DDA movebuffer[ MOVEBUFFER_SIZE];
#else
DDA movebuffer[MOVEBUFFER_SIZE] __attribute__ ((__section__ (".bss")));
#endif


#if ARCH == arm
unsigned dda_queue_get_mb_head( void) { return mb_head; }
unsigned dda_queue_get_mb_tail( void) { return mb_tail; }
#endif

/// check if the queue is completely full
#if ARCH == arm
int dda_queue_full() {
	int result;
	if (mb_tail > mb_head) {
		result = ((mb_tail - mb_head - 1) == 0) ? 1 : 0;
	} else {
		result = ((mb_tail + MOVEBUFFER_SIZE - mb_head - 1) == 0) ? 1 : 0;
	}
	return result;
#else
uint8_t queue_full() {
	MEMORY_BARRIER();
	if (mb_tail > mb_head) {
		return ((mb_tail - mb_head - 1) == 0) ? 255 : 0;
	} else {
		return ((mb_tail + MOVEBUFFER_SIZE - mb_head - 1) == 0) ? 255 : 0;
	}
#endif
}

/// check if the queue is completely empty
#if ARCH == arm
int dda_queue_empty() {
	cli();
	uint8_t result = ((mb_tail == mb_head) && (movebuffer[ mb_tail].live == 0)) ? 1 : 0;
	sei();
	return result;
}
#else
uint8_t queue_empty() {
	uint8_t save_reg = SREG;
	cli();
	CLI_SEI_BUG_MEMORY_BARRIER();	

	uint8_t result = ((mb_tail == mb_head) && (movebuffer[mb_tail].live == 0))?255:0;

	MEMORY_BARRIER();
	SREG = save_reg;
	return result;
}
#endif

// -------------------------------------------------------
// This is the one function called by the timer interrupt.
// It calls a few other functions, though.
// -------------------------------------------------------
/// Take a step or go to the next move.
void dda_queue_step() {
	// do our next step
	DDA* current_movebuffer = &movebuffer[mb_tail];
	if (current_movebuffer->live) {
		if (current_movebuffer->waitfor_temp) {
			setTimer(HEATER_WAIT_TIMEOUT);
			if (temp_achieved()) {
				current_movebuffer->live = current_movebuffer->waitfor_temp = 0;
				serial_writestr_P(PSTR("Temp achieved\n"));
			}

			#if STEP_INTERRUPT_INTERRUPTIBLE
				sei();
			#endif
		}
		else {
			// NOTE: dda_step makes this interrupt interruptible after steps have been sent but before new speed is calculated.
			dda_step(current_movebuffer);
		}
	}

	// fall directly into dda_start instead of waiting for another step
	// the dda dies not directly after its last step, but when the timer fires and there's no steps to do
	if (current_movebuffer->live == 0)
		dda_queue_next_move();
}

/// add a move to the movebuffer
/// \note this function waits for space to be available if necessary, check queue_full() first if waiting is a problem
/// This is the only function that modifies mb_head and it always called from outside an interrupt.
void dda_queue_enqueue(TARGET *t) {
	// don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target
#if ARCH == arm
	fprintf( stderr, "enqueue called\n");
	while (pruss_queue_full()) {
		sched_yield();
	}
	if (t != NULL) {
	  //		pruss_execute_move( int axis, double distance, double speed, double ramp);
	}

#else
	while (dda_queue_full()) {
		delay( WAITING_DELAY);
	}
	uint8_t h = mb_head + 1;
	h &= (MOVEBUFFER_SIZE - 1);

	DDA* new_movebuffer = &(movebuffer[h]);
	
	if (t != NULL) {
		dda_create(new_movebuffer, t);
	}
	else {
		// it's a wait for temp
		new_movebuffer->waitfor_temp = 1;
		new_movebuffer->nullmove = 0;
	}

	// make certain all writes to global memory
	// are flushed before modifying mb_head.
	MEMORY_BARRIER();
	
	mb_head = h;
	
#if ARCH != arm
	uint8_t save_reg = SREG;
#endif
	cli();
#if ARCH != arm
	CLI_SEI_BUG_MEMORY_BARRIER();
#endif

	uint8_t isdead = (movebuffer[mb_tail].live == 0);
	
	MEMORY_BARRIER();
#if ARCH != arm
	SREG = save_reg;
#endif

	
	if (isdead) {
		timer1_compa_deferred_enable = 0;
		dda_queue_next_move();
		if (timer1_compa_deferred_enable) {
#if ARCH != arm
			uint8_t save_reg = SREG;
#endif
			cli();
#if ARCH != arm
			CLI_SEI_BUG_MEMORY_BARRIER();
#endif
			
#if ARCH != arm
			TIMSK1 |= MASK(OCIE1A);
#endif
			
			MEMORY_BARRIER();
#if ARCH != arm
			SREG = save_reg;
#endif
		}
	}	
#endif
}

/// go to the next move.
/// be aware that this is sometimes called from interrupt context, sometimes not.
/// Note that if it is called from outside an interrupt it must not/can not by
/// be interrupted such that it can be re-entered from within an interrupt.
/// The timer interrupt MUST be disabled on entry. This is ensured because
/// the timer was disabled at the start of the ISR or else because the current
/// move buffer was dead in the non-interrupt case (which indicates that the 
/// timer interrupt is disabled).
void dda_queue_next_move() {
	while ((dda_queue_empty() == 0) && (movebuffer[mb_tail].live == 0)) {
		// next item
		uint8_t t = mb_tail + 1;
		t &= (MOVEBUFFER_SIZE - 1);
		DDA* current_movebuffer = &movebuffer[t];
		// tail must be set before setTimer call as setTimer
		// reenables the timer interrupt, potentially exposing
		// mb_tail to the timer interrupt routine. 
		mb_tail = t;
		if (current_movebuffer->waitfor_temp) {
			#ifndef	REPRAP_HOST_COMPATIBILITY
				serial_writestr_P(PSTR("Waiting for target temp\n"));
			#endif
			current_movebuffer->live = 1;
			setTimer(HEATER_WAIT_TIMEOUT);
		}
		else {
			dda_start(current_movebuffer);
		}
	} 
	if (dda_queue_empty())
		setTimer(0);

}

/// DEBUG - print queue.
/// Qt/hs format, t is tail, h is head, s is F/full, E/empty or neither
void dda_queue_print() {
	sersendf_P(PSTR("Q%d/%d%c"), mb_tail, mb_head, (dda_queue_full()?'F':(dda_queue_empty()?'E':' ')));
}

/// dump queue for emergency stop.
/// \todo effect on startpoint/current_position is undefined!
void dda_queue_flush() {
	// Since the timer interrupt is disabled before this function
	// is called it is not strictly necessary to write the variables
	// inside an interrupt disabled block...
#if ARCH != arm
	uint8_t save_reg = SREG;
#endif
	cli();
#if ARCH != arm
	CLI_SEI_BUG_MEMORY_BARRIER();
#endif
	
	// flush queue
	mb_tail = mb_head;
	movebuffer[mb_head].live = 0;

	// disable timer
	setTimer(0);
	
	MEMORY_BARRIER();
#if ARCH != arm
	SREG = save_reg;
#endif
}

/// wait for queue to empty
void dda_queue_wait() {
	for ( ; dda_queue_empty() == 0 ; ) {
		ifclock(clock_flag_10ms) {
			clock_10ms();
		}
	}
}
