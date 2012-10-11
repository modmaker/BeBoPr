#ifndef	_TEMP_H
#define	_TEMP_H


#include "beaglebone.h"

/*
 *  This (temp.[ch]) code links an analog input channel and a conversion function to a temperature sensor.
 *  At least, a sensor for the extruder needs to be defined. A sensor for a heated bed is optional.
 *  The sensors will be used by the heater code and might be used by other temperature sensing/logging code.
 */

typedef int (temp_conversion_f) (int in, double* pout);

typedef const struct {
  channel_tag		tag;
  channel_tag		source;
  unsigned int		in_range_time;	// ms
  temp_conversion_f*	conversion;
} temp_config_record;

extern channel_tag temp_lookup_by_name( const char* name);

extern int temp_config( temp_config_record* config_data, int nr_config_items);
extern int temp_init( void);
//extern channel_tag temp_lookup_by_name( const char* id);
extern int temp_get_celsius( channel_tag channel, double* pcelsius);
extern int temp_achieved( channel_tag temp_channel);
extern int temp_set_setpoint( channel_tag channel, double setpoint, double delta_low, double delta_high);
extern int temp_get_setpoint( channel_tag channel, double* psetpoint);

#endif

