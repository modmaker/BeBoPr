#ifndef	_DDA_QUEUE
#define	_DDA_QUEUE

#include	"dda.h"
#include	"timer.h"

#define HEATER_WAIT_TIMEOUT 1000 MS

/*
	variables
*/

// this is the ringbuffer that holds the current and pending moves.
#if ARCH == arm
extern unsigned dda_queue_get_mb_head( void);
extern unsigned dda_queue_get_mb_tail( void);
#else
extern int8_t	mb_head;
extern uint8_t	mb_tail;
#endif
extern DDA movebuffer[MOVEBUFFER_SIZE];

/*
	methods
*/

// queue status methods
#if ARCH == arm
int dda_queue_full(void);
int dda_queue_empty(void);
#else
uint8_t dda_queue_full(void);
uint8_t dda_queue_empty(void);
#endif
// take one step
void dda_queue_step(void);

// add a new target to the queue
// t == NULL means add a wait for target temp to the queue
void dda_queue_enqueue(TARGET *t);

// called from step timer when current move is complete
//void dda_queue_next_move(void) __attribute__ ((hot));
void dda_queue_next_move(void);

// print queue status
void dda_queue_print(void);

// flush the queue for eg; emergency stop
void dda_queue_flush(void);

// wait for queue to empty
void dda_queue_wait(void);

#endif	/* _DDA_QUEUE */
