
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "analog.h"
#include "temp.h"
#include "beaglebone.h"
#include "thermistor.h"
#include "bebopr.h"
#include "heater.h"
#include "pwm.h"
#include "traject.h"
#include "eeprom.h"
#include "gpio.h"

/*
 * Here one defines where the kernel puts the analog inputs,
 * this happens to change from kernel to kernel :-(
 */
//#define AIN_PATH_PREFIX "/sys/devices/platform/tsc/"		/* kernel 3.2.0 */
#define AIN_PATH_PREFIX "/sys/devices/platform/omap/tsc/"	/* kernel 3.2.16 */

#define PWM_PATH_PREFIX "/sys/class/pwm/"

/*
 * Note, for the ease of implementation, the string addresses are used.
 * This means one cannot use identical strings, but must use pointers
 * to the one and only string!
 */
#define GENERATE_TAG( name) static const char name[] = #name
GENERATE_TAG( bed_thermistor);
GENERATE_TAG( extruder_thermistor);
GENERATE_TAG( spare_ain);
GENERATE_TAG( temp_extruder);
GENERATE_TAG( temp_bed);
GENERATE_TAG( heater_extruder);
GENERATE_TAG( heater_bed);
GENERATE_TAG( pwm_extruder);
GENERATE_TAG( pwm_bed);
GENERATE_TAG( pwm_fan);

static const analog_config_record analog_config_data[] = {
  {
    .tag                = bed_thermistor,
    .device_path	= AIN_PATH_PREFIX "ain2",	// BEBOPR_R2_J6 - THRM0 (hardware ain1)
    .filter_length	= 50,
  },
  {
    .tag                = spare_ain,
    .device_path	= AIN_PATH_PREFIX "ain4",	// BEBOPR_R2_J7 - THRM1 (hardware ain3)
    .filter_length	= 10,
  },
  {
    .tag                = extruder_thermistor,
    .device_path	= AIN_PATH_PREFIX "ain6",	// BEBOPR_R2_J8 - THRM2 (hardware ain5)
    .filter_length	= 50,
  },
};

static const temp_config_record temp_config_data[] = {
  {
    .tag                = temp_extruder,
    .source		= extruder_thermistor,
    .in_range_time	= 15000,
    .conversion		= bone_epcos_b5760g104f,
  },
  {
    .tag                = temp_bed,
    .source		= bed_thermistor,
    .in_range_time	= 15000,
    .conversion		= bone_bed_thermistor_330k,
  },
};

static const pwm_config_record pwm_config_data[] = {
  {
    .tag		= pwm_extruder,
    .device_path	= PWM_PATH_PREFIX "ehrpwm.2:0",	// BEBOPR_R2_J3 - PWM1
    .frequency		= 10,
  },
  {
    .tag		= pwm_fan,
    .device_path	= PWM_PATH_PREFIX "ehrpwm.2:1",	// BEBOPR_R2_J2 - PWM0
    .frequency		= 0,         // frequency is determined by ehrpwm.2:0 !
  },
  {
    .tag		= pwm_bed,
    .device_path	= PWM_PATH_PREFIX "ehrpwm.1:0",	// BEBOPR_R2_J4 - PWM2
    .frequency		= 10,
  },
};

static const heater_config_record heater_config_data[] = {
  {
    .tag		= heater_extruder,
    .analog_input	= temp_extruder,
    .analog_output	= pwm_extruder,
    .pid =
    {
	    .K = 0.0,
	    .P = 15.0,
	    .I = 10.0,
	    .D = 0.0,
	    .I_limit = 0.7,
    },
  },
  {
    .tag		= heater_bed,
    .analog_input	= temp_bed,
    .analog_output	= pwm_bed,
    .pid =
    {
	    .K = 0.0,
	    .P = 10.0,
	    .I = 0.0,
	    .D = 0.0,
	    .I_limit = 0.0,
    },
  },
};

static int use_pololu_drivers = 1;

int bebopr_pre_init( void)
{
  int result = -1;

  result = analog_config( analog_config_data, NR_ITEMS( analog_config_data));
  if (result < 0) {
    fprintf( stderr, "analog_config failed!\n");
    goto done;
  }
  result = temp_config( temp_config_data, NR_ITEMS( temp_config_data));
  if (result < 0) {
    fprintf( stderr, "temp_config failed!\n");
    goto done;
  }
  result = pwm_config( pwm_config_data, NR_ITEMS( pwm_config_data));
  if (result < 0) {
    fprintf( stderr, "pwm_config failed!\n");
    goto done;
  }
  result = heater_config( heater_config_data, NR_ITEMS( heater_config_data));
  if (result < 0) {
    fprintf( stderr, "heater_config failed!\n");
    goto done;
  }
  result = get_step_io_config( EEPROM_PATH);
  // Only differentiate between Pololu and TB6560, default to Pololu
  if (result == TB6560_DRIVERS) {
    use_pololu_drivers = 0;
  }
  fprintf( stderr, "Using stepper driver configuration: '%s'\n", (use_pololu_drivers) ? "Pololu" : "TB6560");
  result = 0;
 done:
  return result;
}

// Limit switches present in the system.
// only return 0 or 1
int config_axis_has_min_limit_switch( axis_e axis)
{
  switch (axis) {
  case x_axis:	return 1;
  case y_axis:	return 1;
  case z_axis:	return 1;
  default:      return 0;
  }
}

int config_axis_has_max_limit_switch( axis_e axis)
{
  switch (axis) {
  case x_axis:	return 0;
  case y_axis:	return 0;
  case z_axis:	return 1;
  default:      return 0;
  }
}

// Limit switch polarity, return either 0 or 1. Note that the inputs are being
// inverted: led on = reads a 1, led off = reads a 0. If the LED turns off when
// activating a switch, that switch should be set to active low and vice versa.
int config_min_limit_switch_is_active_low( axis_e axis)
{
  switch (axis) {
  case x_axis:	return 1;
  case y_axis:	return 1;
  case z_axis:	return 0;
  default:      return 0;
  }
}

int config_max_limit_switch_is_active_low( axis_e axis)
{
  switch (axis) {
  case x_axis:	return 0;
  case y_axis:	return 0;
  case z_axis:	return 1;
  default:      return 0;
  }
}

int config_use_pololu_drivers( void)
{
  return use_pololu_drivers;
}

/*
 *  Specify step size for each axis in [m]
 */
double config_get_step_size( axis_e axis)
{
  switch (axis) {
  case x_axis:	return 6.25E-6;
  case y_axis:	return 6.25E-6;
  case z_axis:	return 0.390125E-6;
  case e_axis:	return 1.978E-6;
  default:	return 0.0;
  }
}

/*
 *  Specify maximum allowed feed for each axis in [mm/min]
 */
double config_get_max_feed( axis_e axis)
{
  switch (axis) {
  case x_axis:	return 22500.0;	// 0.00625 mm/step @ 60 kHz
  case y_axis:	return 16000.0;	// 0.00625 mm/step @ 53 kHz
  case z_axis:	return   300.0; // 0.00039 mm/step @ 13 kHz
  case e_axis:	return  3000.0; // 0.00198 mm/step @ 25 kHz
  default:	return 0.0;
  }
}

/*
 *  Specify maximum acceleration for each axis in [m/s^2]
 */
double config_get_max_accel( axis_e axis)
{
  switch (axis) {
  case x_axis:	return 1.0;
  case y_axis:	return 1.0;
  case z_axis:	return 1.0;
  case e_axis:	return 1.0;
  default:	return 0.0;
  }
}

/*
 *  Specifiy the axes that need a reversed stepper direction signal
 */
int config_reverse_axis( axis_e axis)
{
  switch (axis) {
  case x_axis:  return 1;
  case y_axis:	return 0;
  case z_axis:	return 0;
  case e_axis:	return 1;
  default:	return 0;
  }
}

/*
 *  Specify the values that will be used as soft limits.
 *  During normal operation the machine will not move outside
 *  the space defined by the soft limits.
 *  Behaviour depends on implementation: Faulting or clipping
 *  are two of the options.
 *  Return true if a position is defined, false otherwise.
 */
int config_min_soft_limit( axis_e axis, double* pos)
{
  switch (axis) {
  case x_axis:	*pos = 0.0; return 1;
  case y_axis:	*pos = 0.0; return 1;
  case z_axis:	*pos = 0.0; return 1;
  default:	return 0;
  }
}

int config_max_soft_limit( axis_e axis, double* pos)
{
  switch (axis) {
  case x_axis:	*pos = 215.0; return 1;
  case y_axis:	*pos = 200.0; return 1;
  case z_axis:	*pos =  80.0; return 1;
  default:	return 0;
  }
}

/*
 *  Specify the positions of the limit switches or homing sensors.
 *  Any switch can act in either one of the following modes:
 *    as limit switch, to signal a (global) end position
 *  or
 *    as calibration position, to define an exact known position.
 *  
 *  Return true if a calibration position is defined by a switch,
 *  false otherwise. NOTE: For a consistent coordinate space, exactly
 *  one switch on each axis should be defined as calibration switch!
 */
int config_min_switch_pos( axis_e axis, double* pos)
{
  switch (axis) {
  case x_axis:	*pos = x_cal_pos; return 1;
  case y_axis:	*pos = y_cal_pos; return 1;
  case z_axis:	*pos = z_cal_pos; return 1;
//case z_axis:	return 0;
  default:	return 0;
  }
}

int config_max_switch_pos( axis_e axis, double* pos)
{
  switch (axis) {
  case x_axis:	return 0;
  case y_axis:	return 0;
  case z_axis:	return 0;
//case z_axis: *pos = z_cal_pos; return 1;
  default:	return 0;
  }
}

/*
 *  Specify the feed used during homing operations
 *  to release the home switch.
 */
double config_get_home_release_feed( axis_e axis)
{
  switch (axis) {
  case x_axis:	return  150.0;
  case y_axis:	return  150.0;
  case z_axis:	return  150.0;
  default:	return    0.0;
  }
}

/*
 *  Specify the maximum feed that may be used during homing
 *  operations when moving towards the home switch.
 */
double config_get_home_max_feed( axis_e axis)
{
  switch (axis) {
  case x_axis:	return 3000.0;
  case y_axis:	return 3000.0;
  case z_axis:	return  450.0;
  default:	return    0.0;
  }
}

/*
 *  Specify is the E axis is being fed relative coordinates only
 */
int config_e_axis_is_always_relative( void)
{
  return 1;
}

/*
 *  Specify the character code that should be used for keep-alive messages.
 *  This character should not disturb the communication.
 *  Pronterface seems to accept most characters, but only a newline
 *  does not disturb the program.
 *  Repsnapper also accepts the newline without causing problems.
 */
char config_keep_alive_char( void)
{
  return '\n';
}


/*
 *  Late initialization enables I/O power.
 */
int bebopr_post_init( void)
{
  int result = -1;

  /*
   *  IO_PWR_ON  = R9 / GPIO1[6] / gpio38 /  gpmc_ad6
   *  !IO_PWR_ON = R8 / GPIO1[2] / gpio34 /  gpmc_ad2
   */
  gpio_write_int_value_to_file( "export", 38);
  gpio_write_value_to_pin_file( 38, "direction", "out");
  gpio_write_value_to_pin_file( 38, "value", "1");

  gpio_write_int_value_to_file( "export", 34);
  gpio_write_value_to_pin_file( 34, "direction", "out");
  gpio_write_value_to_pin_file( 34, "value", "0");

  fprintf( stderr, "Turned BEBOPR I/O power on\n");
  result = 0;

  return result;
}
