
#include <stdio.h>

#include "analog.h"
#include "temp.h"
#include "beaglebone.h"
#include "thermistor.h"
#include "bebopr.h"
#include "heater.h"
#include "pwm.h"

/*
 * Here one defines where the kernel puts the analog inputs,
 * this happens to change from kernel to kernel :-(
 */
//#define AIN_PATH_PREFIX "/sys/devices/platform/tsc/"		/* kernel 3.2.0 */
#define AIN_PATH_PREFIX "/sys/devices/platform/omap/tsc/"	/* kernel 3.2.16 */

static const analog_config_struct analog_config_data[] = {
  {  // e_analog_1
    .device_path	= AIN_PATH_PREFIX "ain2",	// BEBOPR_R2_J6 - THRM0 (hardware ain1)
    .filter_length	= 0,
  },
  {  // e_analog_2
    .device_path	= AIN_PATH_PREFIX "ain4",	// BEBOPR_R2_J7 - THRM1 (hardware ain3)
    .filter_length	= 10,
  },
  {  // e_analog_3
    .device_path	= AIN_PATH_PREFIX "ain6",	// BEBOPR_R2_J8 - THRM2 (hardware ain5)
    .filter_length	= 10,
  },
};

static const temp_config_struct temp_config_data[] = {
  {
    .sensor		= e_temp_extruder,
    .channel		= 2,	// index of THRM2 entry in analog_config_data
    .in_range_time	= 5000,
    .conversion		= bone_epcos_b5760g104f,
  },
  {
    .sensor		= e_temp_bed,
    .channel		= 0,	// index of THRM0 entry in analog_config_data
    .in_range_time	= 5000,
    .conversion		= bone_thermistor_100k,
  },
};

static const heater_config_struct heater_config_data[] = {
  {
    .heater		= e_heater_extruder,
    .sensor		= e_temp_extruder,
    .pwm_output		= e_pwm_output_2
  },
  {
    .heater		= e_heater_bed,
    .sensor		= e_temp_bed,
    .pwm_output		= e_pwm_output_1
  },
};


int bebopr_pre_init( void)
{
  int result = -1;
  fprintf( stderr, "<bebopr_pre_init>");

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
  result = heater_config( heater_config_data, NR_ITEMS( heater_config_data));
  if (result < 0) {
    fprintf( stderr, "heater_config failed!\n");
    goto done;
  }
 done:
  return result;
}

// Limit switches present in the system.
// only return 0 or 1
int limsw_x_has_max( void) { return 0; }
int limsw_x_has_min( void) { return 1; }
int limsw_y_has_max( void) { return 0; }
int limsw_y_has_min( void) { return 1; }
int limsw_z_has_max( void) { return 1; }
int limsw_z_has_min( void) { return 1; }

// Limit switch polarity, only return 0 or 1
// input has inverter! led on = 1, led off = 0
// If the LED turns off activating the switch,
// the switch is active low and vice versa.
int limsw_x_max_is_active_low( void) { return 0; }
int limsw_x_min_is_active_low( void) { return 1; }
int limsw_y_max_is_active_low( void) { return 0; }
int limsw_y_min_is_active_low( void) { return 1; }
int limsw_z_max_is_active_low( void) { return 1; }
int limsw_z_min_is_active_low( void) { return 0; }

int use_pololu_drivers( void) { return 0; }
