#ifndef _BEBOPR_H
#define _BEBOPR_H

#include "config.h"

// Define the frequency of the PRUSS clock
#define TIMER_CLOCK	200.0E6

// Identification values for axes
typedef enum {
  x_axis, y_axis, z_axis, e_axis
} axis_e;

// Early init that pushes configuration to subsystems
extern int bebopr_pre_init( void);

// determines stepper driver control
extern int use_pololu_drivers( void);

// Configuration

// these all return either 0 for false or 1 for true
extern int config_axis_has_min_limit_switch( axis_e axis);
extern int config_axis_has_max_limit_switch( axis_e axis);
extern int config_min_limit_switch_is_active_low( axis_e axis);
extern int config_max_limit_switch_is_active_low( axis_e axis);
extern int config_reverse_axis( axis_e axis);

// these all return a hardware dimenstion
extern double config_axis_get_min_pos( axis_e axis);
extern double config_axis_get_max_pos( axis_e axis);
extern double config_get_max_feed( axis_e axis);
extern double config_get_max_accel( axis_e axis);
extern double config_get_step_size( axis_e axis);

#endif
