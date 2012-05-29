#ifndef	_TEMP_H
#define	_TEMP_H

#if 0

#include <stdint.h>

//#include "config.h"


#define TEMP_NR_SENSORS	2

#define TEMP_EXTRUDER	0
#define TEMP_BED	1


/*
NOTES

no point in specifying a port- all the different temp sensors we have must be on a particular port. The MAX6675 must be on the SPI, and the thermistor and AD595 must be on an analog port.

we still need to specify which analog pins we use in machine.h for the analog sensors however, otherwise the analog subsystem won't read them.
*/

#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) TEMP_SENSOR_ ## name,
typedef enum {
	#include "config.h"
	NUM_TEMP_SENSORS,
	TEMP_SENSOR_none
} temp_sensor_t;
#undef DEFINE_TEMP_SENSOR

typedef enum {
	TT_THERMISTOR,
	TT_MAX6675,
	TT_AD595,
	TT_PT100,
	TT_INTERCOM,
	TT_DUMMY,
} temp_type_t;

#define	temp_tick temp_sensor_tick

void temp_init(void);

void temp_sensor_tick(void);

uint8_t	temp_achieved(void);

void temp_set(temp_sensor_t index, uint16_t temperature);
uint16_t temp_get(temp_sensor_t index);

uint8_t temp_all_zero(void);

void temp_print(temp_sensor_t index);

#endif

#include "analog.h"

/*
 *  This (temp.[ch]) code links an analog input channel and a conversion function to a temperature sensor.
 *  At least, a sensor for the extruder needs to be defined. A sensor for a heated be is optional.
 *  The sensors will be used by the heater code and might be used by other temperature sensing/logging code.
 */
typedef enum {
  e_temp_extruder,
  e_temp_bed,
  e_temp_num_sensors
} temp_sensor_e;

typedef int (temp_conversion_f) (int in, double* out);

typedef struct {
  analog_channel_e	channel;
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

