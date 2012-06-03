#ifndef	_HEATER_H
#define	_HEATER_H


#include "temp.h"
#include "pwm.h"
#include "beaglebone.h"


typedef struct {
  double	P;
  double	I;
  double	D;
  double	I_limit;
  double	FF_factor;
  double	FF_offset;
} pid_settings;

typedef const struct {
  channel_tag		tag;
  channel_tag		analog_input;
  channel_tag		analog_output;
  pid_settings		pid;
  double		setpoint;
} heater_config_record;


extern channel_tag heater_lookup_by_name( const char* name);

extern int heater_config( heater_config_record* config_data, int nr_config_items);
extern int heater_init( void);
extern int heater_save_settings( void);
extern int heater_load_settings( void);

extern int heater_set_pid_values( channel_tag heater, const pid_settings* pid_settings);
extern int heater_get_pid_values( channel_tag heater, pid_settings* pid_settings);
extern int heater_set_setpoint( channel_tag heater, double setpoint);
extern int heater_get_setpoint( channel_tag heater, double* setpoint);
extern int heater_enable( channel_tag heater, int state);
extern int heater_set_raw_pwm( channel_tag heater, double percentage);
extern int heater_get_celsius( channel_tag heater_channel, double* pcelsius);
extern int heater_temp_reached( channel_tag heater);

extern channel_tag heater_lookup_by_name( const char* name);

#if 0
void heater_set(heater_t index, uint8_t value);
void heater_tick(heater_t h, temp_sensor_t t, uint16_t current_temp, uint16_t target_temp);

uint8_t heaters_all_off(void);

void pid_set_p(heater_t index, int32_t p);
void pid_set_i(heater_t index, int32_t i);
void pid_set_d(heater_t index, int32_t d);
void pid_set_i_limit(heater_t index, int32_t i_limit);

void heater_print(uint16_t i);
#endif

#endif	/* _HEATER_H */
