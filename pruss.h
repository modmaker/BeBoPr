#ifndef _PRUSS_H
#define _PRUSS_H

#include <stdint.h>
#include "algo2cmds.h"

/* Low level interface */

extern uint32_t pruss_rd32( unsigned int addr);
extern uint16_t pruss_rd16( unsigned int addr);
extern uint8_t pruss_rd8( unsigned int addr);
extern void pruss_wr32( unsigned int addr, uint32_t data);
extern void pruss_wr16( unsigned int addr, uint16_t data);
extern void pruss_wr8( unsigned int addr, uint8_t data);


/* High level interface */

extern int pruss_load_code( const char* fname, unsigned int* start_addr);
//extern int pruss_write_command_struct( int ix_in, PruCommandUnion* data);
//extern int pruss_command( PruCommandUnion* cmd);
extern int pruss_init( void);
extern void pruss_wait_for_halt( void);
extern int pruss_dump_state( void);
extern int pruss_halt_pruss( void);

extern int pruss_queue_full( void);
extern int pruss_queue_set_origin( int axis);
extern int pruss_queue_set_accel( int axis, uint32_t c0);
extern int pruss_queue_accel_more( int axis, uint32_t cmin, int32_t delta);
extern int pruss_queue_accel( int axis, uint32_t c0, uint32_t cmin, int32_t delta);
extern int pruss_queue_dwell( int axis, uint32_t cmin, int32_t delta);
extern int pruss_queue_decel( int axis, int32_t delta);
extern int pruss_queue_execute( void);
extern int pruss_queue_set_pulse_length( int axis, uint16_t length);
extern int pruss_queue_config_axis( int axis, uint32_t ssi, uint16_t sst, uint16_t ssn, int reverse);

extern void pruss_dump_position( int axis);

#endif
