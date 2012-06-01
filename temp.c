/** \file
	\brief Manage temperature sensors

	\note \b ALL temperatures are stored as 14.2 fixed point in teacup, so we have a range of 0 - 16383.75 celsius and a precision of 0.25 celsius. That includes the ThermistorTable, which is why you can't copy and paste one from other firmwares which don't do this.
*/


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "temp.h"
#include "analog.h"
#include "beaglebone.h"
#include "debug.h"
#include "mendel.h"

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

struct temp_channel {
  channel_tag		id;
  channel_tag		source;
  temp_conversion_f* 	conversion;
  unsigned int 		out_of_range;
  double 		value;
  double 		setpoint;
  double 		range_low;
  double 		range_high;
};

static struct temp_channel* temp_channels;
static unsigned int num_temp_channels;

static int temp_index_lookup( channel_tag temp_channel)
{
  for (int ix = 0 ; ix < num_temp_channels ; ++ix) {
    if (temp_channels[ ix].id == temp_channel) {
      return ix;
    }
  }
  if (debug_flags & DEBUG_TEMP) {
    fprintf( stderr, "temp_index_lookup failed for '%s'\n", tag_name( temp_channel));
  }
  return -1;
}

/*
 * Callback, called from adc processing thread in analog.c
 */
static int temp_update( channel_tag temp_channel, int analog_value)
{
  int ix = temp_index_lookup( temp_channel);
  if (ix >= 0) {
    double celsius;
    int result;
    temp_conversion_f* convert = temp_channels[ ix].conversion;
    if (convert != NULL) {
      result = convert( analog_value, &celsius);
    } else {
      celsius = (double) analog_value;
      result = 0;
    }
    if (debug_flags & DEBUG_TEMP) {
      fprintf( stderr, "temp_update was called for '%s' with value %d => celsius %1.1lf\n",
	      tag_name( temp_channel), analog_value, celsius);
    }
    if (result == 0) {
      temp_channels[ ix].value = celsius;
    }
    if (result == 0 &&
	temp_channels[ ix].range_low <= celsius &&
	celsius <= temp_channels[ ix].range_high) {
      if (temp_channels[ ix].out_of_range > 0) {
	--temp_channels[ ix].out_of_range;
      }	
    } else {
      temp_channels[ ix].out_of_range = out_of_range_time;
    }
  }
  return -1;
}

/*
 * Configuration settings are stored seperately (bebopr_rx.c) and
 * a configuration call is used to communicate these with this
 * code.
 */
static temp_config_record* temp_config_data = NULL;
static int temp_config_items = 0;

int temp_config( temp_config_record* config_data, int nr_config_items)
{
  temp_config_data  = config_data;
  temp_config_items = nr_config_items;
  num_temp_channels = 0;
  temp_channels     = calloc( nr_config_items, sizeof( struct temp_channel));
  return 0;
}

/*
 * Initialization and configuration
 */
int temp_init( void)
{
  if (temp_config_data != NULL) {
    mendel_sub_init( "analog", analog_init);
    for (int ix = 0 ; ix < temp_config_items ; ++ix) {
      temp_config_record* ps 	= &temp_config_data[ ix];
      struct temp_channel* pd	= &temp_channels[ ix];
      pd->id			= ps->tag;
      pd->source		= ps->source;
      pd->conversion 		= ps->conversion;
      pd->out_of_range 		= ps->in_range_time;
      if (analog_set_update_callback( ps->source, temp_update, ps->tag) < 0) {
        fprintf( stderr, "temp_init: could not connect callback for '%s' to source '%s'\n", ps->tag, ps->source);
      }
      ++num_temp_channels;
    }
    return 0;
  }
  fprintf( stderr, "temp_init: no configuration data!\n");
  return -1;
}

int temp_set_setpoint( channel_tag temp_channel, double setpoint, double delta_low, double delta_high)
{
  int ix = temp_index_lookup( temp_channel);
  if (ix >= 0) {
    temp_channels[ ix].setpoint   = setpoint;
    temp_channels[ ix].range_low  = setpoint - delta_low;
    temp_channels[ ix].range_high = setpoint + delta_high;
    return 0;
  }
  return -1;
}

int temp_get_celsius( channel_tag temp_channel, double* pcelsius)
{
  if (pcelsius != NULL) {
    int ix = temp_index_lookup( temp_channel);
    if (ix >= 0) {
      *pcelsius = temp_channels[ ix].value;
      return 0;
    }
  }
  return -1;
}

/// report whether all temp sensors are reading their target temperatures
/// used for M109 and friends
int temp_achieved( void)
{
  for (int ix = 0 ; ix < num_temp_channels ; ++ix) {
    if (temp_channels[ ix].out_of_range > 0) {
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
  for (int ix = 0 ; ix < num_temp_channels ; ++ix) {
    if (temp_channels[ ix].setpoint != 0.0) {
      return 0;
    }
  }
  return 1;
}

channel_tag temp_lookup_by_name( const char* name)
{
  for (int ix = 0 ; ix < num_temp_channels ; ++ix) {
    channel_tag tag = temp_channels[ ix].id;
    if (strcmp( tag_name( tag), name) == 0) {
      return tag;
    }
  }
  return NULL;
}

