#ifndef	_TEMP_H
#define	_TEMP_H


#include "analog.h"

/*
 *  This (temp.[ch]) code links an analog input channel and a conversion function to a temperature sensor.
 *  At least, a sensor for the extruder needs to be defined. A sensor for a heated bed is optional.
 *  The sensors will be used by the heater code and might be used by other temperature sensing/logging code.
 */
typedef enum {
  e_temp_extruder,
  e_temp_bed,
  e_temp_num_sensors
} temp_sensor_e;

typedef int (temp_conversion_f) (int in, double* out);

typedef struct {
  unsigned int		channel;
  temp_sensor_e		sensor;
  int			in_range_time;	// ms
  temp_conversion_f*	conversion;
} temp_config_struct;

extern int temp_config( const temp_config_struct* config_data, int nr_config_items);
extern int temp_init( void);
extern int temp_get_celsius( temp_sensor_e channel, double* celsius);
extern int temp_achieved( void);
extern int temp_set_setpoint( temp_sensor_e channel, double setpoint, double delta_low, double delta_high);
extern int temp_get_setpoint( temp_sensor_e channel, double* setpoint);
// TODO: remove the following two from all code:
extern void temp_tick( void);
extern int temp_all_zero( void);

#endif

