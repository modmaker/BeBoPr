/** \file
	\brief Manage temperature sensors

	\note \b ALL temperatures are stored as 14.2 fixed point in teacup, so we have a range of 0 - 16383.75 celsius and a precision of 0.25 celsius. That includes the ThermistorTable, which is why you can't copy and paste one from other firmwares which don't do this.
*/


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "temp.h"
#include "analog.h"
#include "beaglebone.h"
#include "debug.h"

/*
 * Temperature sensor interface for BeBoPr rev0.
 *
 * Three of the analog inputs on the AM3359 are used for temperature measurements.
 * (See the analog.c code)
 * The raw data is converted to and from temperatures in this file, using a table
 * with reference values and linear interpolation.
 *
 * Optionally, an I2C connected device (ADC / TC interface) can be used (TBI).
 */


static const int out_of_range_time = 15;

static struct {
  double value;
  double setpoint;
  double range_low;
  double range_high;
  int out_of_range;
  temp_conversion_f* conversion;
} temp_sensors[ e_temp_num_sensors];

/*
 * Callback, called from adc processing thread in analog.c
 */
static int temp_update( update_channel_t channel, int analog_value)
{
  if (channel >= 0 && channel <= e_temp_num_sensors) {
    double celsius;
    int result;
    temp_conversion_f* convert = temp_sensors[ channel].conversion;
    if (convert != NULL) {
      result = convert( analog_value, &celsius);
    } else {
      celsius = (double) analog_value;
      result = 0;
    }
    if (debug_flags & DEBUG_TEMP) {
      fprintf( stderr, "temp_update called for channel %d with value %d => celsius %1.1lf\n",
	       channel, analog_value, celsius);
    }
    if (result == 0) {
      temp_sensors[ channel].value = celsius;
    }
    if (result == 0 &&
	temp_sensors[ channel].range_low <= celsius &&
	celsius <= temp_sensors[ channel].range_high) {
      if (temp_sensors[ channel].out_of_range > 0) {
	--temp_sensors[ channel].out_of_range;
      }	
    } else {
      temp_sensors[ channel].out_of_range = out_of_range_time;
    }
  }
  return -1;
}

/*
 * Configuration settings are stored seperately (bebopr_rx.c) and
 * a configuration call is used to communicate these with this
 * code.
 */
static const temp_config_struct* temp_config_data = NULL;
static int temp_config_items = 0;

int temp_config( const temp_config_struct* config_data, int nr_config_items)
{
  temp_config_data  = config_data;
  temp_config_items = nr_config_items;
  return 0;
}

/*
 * Initialization and configuration
 */
int temp_init( void)
{
  if (temp_config_data != NULL) {
    analog_init();
    for (int i = 0 ; i < temp_config_items ; ++i) {
      unsigned int		analog_channel	= temp_config_data[ i].channel;
      temp_sensor_e		sensor		= temp_config_data[ i].sensor;
      temp_conversion_f*	conversion	= temp_config_data[ i].conversion;
      temp_sensors[ sensor].out_of_range 	= temp_config_data[ i].in_range_time;
      temp_sensors[ sensor].conversion 		= conversion;
      analog_set_update_callback( analog_channel, temp_update, (update_channel_t)sensor);
    }
    return 0;
  }
  fprintf( stderr, "temp_init: no configuration data!\n");
  return -1;
}

int temp_set_setpoint( temp_sensor_e channel, double setpoint, double delta_low, double delta_high)
{
  int result = -1;
  if (channel >= 0 && channel <= e_temp_num_sensors) {
    temp_sensors[ channel].setpoint   = setpoint;
    temp_sensors[ channel].range_low  = setpoint - delta_low;
    temp_sensors[ channel].range_high = setpoint + delta_high;
    result = 0;
  }
  return result;
}

int temp_get_celsius( temp_sensor_e channel, double* celsius)
{
  int result = -1;
  if (channel >= 0 && channel < e_temp_num_sensors && celsius != NULL) {
    *celsius = temp_sensors[ channel].value;
    result = 0;
  }
  return result;
}

/// report whether all temp sensors are reading their target temperatures
/// used for M109 and friends
int temp_achieved( void)
{
  temp_sensor_e i;

  for (i = 0 ; i < e_temp_num_sensors ; ++i) {
    if (temp_sensors[ i].out_of_range > 0) {
      return 0;
    }
  }
  return 1;
}

/*
 * Next two functions for compatability with gcode_process.c / clock.c
 */
void temp_tick( void)
{
  /* NOP */
}

int temp_all_zero( void)
{
  temp_sensor_e i;

  for (i = 0 ; i < e_temp_num_sensors ; ++i) {
    if (temp_sensors[ i].setpoint != 0.0) {
      return 0;
    }
  }
  return 1;
}

