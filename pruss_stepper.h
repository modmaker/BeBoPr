#ifndef _PRUSS_STEPPER_H
#define _PRUSS_STEPPER_H

#include <stdint.h>
#include "algo2cmds.h"

extern int pruss_stepper_init( void);
extern int pruss_stepper_dump_state( void);
extern int pruss_queue_full( void);
extern int pruss_queue_empty( void);
extern int pruss_queue_set_origin( int axis);
extern int pruss_queue_adjust_origin( int axis);
extern int pruss_queue_set_accel( int axis, uint32_t c0);
extern int pruss_queue_accel_more( int axis, uint32_t cmin, int32_t delta);
extern int pruss_queue_accel( int axis, uint32_t c0, uint32_t cmin, int32_t delta);
extern int pruss_queue_dwell( int axis, uint32_t cmin, int32_t delta);
extern int pruss_queue_decel( int axis, int32_t delta);
extern int pruss_queue_execute( void);
extern int pruss_queue_set_pulse_length( int axis, uint16_t length);
extern int pruss_queue_set_idle_timeout( uint8_t period);
extern int pruss_queue_config_axis( int axis, uint32_t ssi, uint16_t sst, uint16_t ssn, int reverse);
extern int pruss_queue_set_enable( int on);
extern int pruss_dump_position( int axis);

#endif
