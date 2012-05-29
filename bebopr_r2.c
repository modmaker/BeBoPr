
#include <stdio.h>

#include "analog.h"
#include "temp.h"
#include "beaglebone.h"
#include "thermistor.h"
#include "bebopr.h"
#include "heater.h"
#include "pwm.h"


static const temp_config_struct temp_config_data[] = {
  {
    .sensor		= e_temp_extruder,
    .channel		= e_analog_3,
    .in_range_time	= 5000,
    .conversion		= bone_thermistor_100k
  },
  {
    .sensor		= e_temp_bed,
    .channel		= e_analog_1,
    .in_range_time	= 5000,
    .conversion		= bone_thermistor_100k
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
