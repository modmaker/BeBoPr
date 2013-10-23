#ifndef _BEBOPR_H
#define _BEBOPR_H

/* Define the file that contains the (new) stepper code for the PRUSS */
#define UCODE_FILE "stepper.bin"

// Identification values for axes
typedef enum {
  x_axis, y_axis, z_axis, e_axis
} axis_e;

static inline char axisName( axis_e axis)
{
  char names[] = { 'X', 'Y', 'Z', 'E', };
  return names[ axis];
}

// Early init that pushes configuration to subsystems
extern int bebopr_pre_init( void);

// Late init that enables I/O power
extern int bebopr_post_init( void);

// Ultimate exit, disable I/O power
extern void bebopr_exit( void);

// Configuration

extern int config_e_axis_is_always_relative( void);
extern char config_keep_alive_char( void);

// determines stepper driver control
extern int config_use_pololu_drivers( void);

// these all return either 0 for false or 1 for true
extern int config_axis_has_min_limit_switch( axis_e axis);
extern int config_axis_has_max_limit_switch( axis_e axis);
extern int config_min_limit_switch_is_active_low( axis_e axis);
extern int config_max_limit_switch_is_active_low( axis_e axis);
extern int config_reverse_axis( axis_e axis);

// these all return a hardware dimension
extern int config_min_soft_limit( axis_e axis, double* pos);
extern int config_max_soft_limit( axis_e axis, double* pos);
extern int config_min_switch_pos( axis_e axis, double* pos);
extern int config_max_switch_pos( axis_e axis, double* pos);
extern unsigned int config_get_micro_step_factor( axis_e axis);
extern unsigned int config_get_active_current( axis_e axis);
extern unsigned int config_get_idle_current( axis_e axis);
extern double config_get_step_size( axis_e axis);

// these all return physical limitations
extern double config_get_max_feed( axis_e axis);
extern double config_get_max_accel( axis_e axis);

// these return preferred settings
extern double config_get_home_max_feed( axis_e axis);
extern double config_get_home_release_feed( axis_e axis);

// recalibrate reference sensor position
extern int config_set_cal_pos( axis_e axis, double pos);
// set absolute or relative E-axis mode
extern int config_set_e_axis_mode( int relative);

// workaround for defines from pinio until removed from code
/* the axis enable signals are handled in the PRUSS code! */
#define	x_enable()	do { /* void */ } while (0)
#define	y_enable()	do { /* void */ } while (0)
#define	z_enable()	do { /* void */ } while (0)
#define	e_enable()	do { /* void */ } while (0)
#define	x_disable()	do { /* void */ } while (0)
#define	y_disable()	do { /* void */ } while (0)
#define	z_disable()	do { /* void */ } while (0)
#define	e_disable()	do { /* void */ } while (0)
#define	power_on()	do { /* void */ } while (0)
#define	power_off()	do { /* void */ } while (0)

// return the kernel type running
typedef enum {
  e_kernel_unknown, e_kernel_other, e_kernel_3_2, e_kernel_3_8
} kernel_type;
extern kernel_type get_kernel_type( void);

#endif
