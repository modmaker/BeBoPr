#ifndef _PRUSS_STEPPER_H
#define _PRUSS_STEPPER_H

#include <stdint.h>
#include "algo2cmds.h"

extern int pruss_stepper_init( void);
extern int pruss_stepper_dump_state( void);
extern int pruss_queue_full( void);
extern int pruss_queue_empty( void);
extern int pruss_queue_set_position( int axis, int32_t pos);
extern int pruss_queue_set_origin( int axis);
extern int pruss_queue_adjust_origin( int axis, int32_t delta);
extern int pruss_queue_adjust_for_ramp( int axis, int32_t delta);
extern int pruss_queue_accel( int axis, uint32_t n0, uint32_t c0, uint32_t cmin, int32_t delta);
extern int pruss_queue_dwell( int axis, uint32_t cmin, int32_t delta);
extern int pruss_queue_decel( int axis, uint32_t nmin, uint32_t cmin, int32_t delta);
extern int pruss_queue_execute( void);
extern int pruss_queue_exec_limited( uint8_t invert, uint8_t mask);
extern int pruss_queue_set_pulse_length( int axis, uint16_t length);
extern int pruss_queue_set_idle_timeout( uint8_t period);
extern int pruss_queue_config_axis( int axis, uint32_t ssi, int reverse);
extern int pruss_queue_config_limsw( int axis, uint8_t min_gpio, uint8_t min_invert, uint8_t max_gpio, uint8_t max_invert);
extern int pruss_queue_set_enable( int on);
extern int pruss_dump_position( void);
extern int pruss_stepper_busy( void);
extern int pruss_stepper_halted( void);
extern int pruss_get_positions( int axis, int32_t* virtPosI, int32_t* requestedPos);
extern int pruss_wait_for_completion( void);
extern void pruss_queue_exit( void);
extern void pruss_stepper_resume( void);
extern void pruss_stepper_single_step( void);

#endif
