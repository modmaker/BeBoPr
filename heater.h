#ifndef	_HEATER_H
#define	_HEATER_H

#if 0

#include "config.h"
#include	<stdint.h>
#include "temp.h"

#define	enable_heater()		heater_set(0, 64)
#define	disable_heater()	heater_set(0, 0)

#undef DEFINE_HEATER
#define DEFINE_HEATER(name, pin) HEATER_ ## name,
typedef enum
{
	#include "config.h"
	NUM_HEATERS,
	HEATER_noheater
} heater_t;
#undef DEFINE_HEATER

#define NUM_HEATERS	2
#define HEATER_EXTRUDER	1
#define HEATER_BED	0

#endif

#include "temp.h"
#include "pwm.h"

typedef struct {
  double	k;
  double	p;
  double	i;
  double	d;
  double	i_limit;
} pid_struct;

typedef enum {
  e_heater_extruder,
  e_heater_bed,
  e_heater_num_outputs
} heater_e;

typedef struct {
  heater_e		heater;
  temp_sensor_e		sensor;
  pwm_output_e		pwm_output;
} heater_config_struct;


extern int heater_config( const heater_config_struct* config_data, int nr_config_items);
extern int heater_init( void);
extern int heater_save_settings( void);
extern int heater_load_settings( void);

extern int heater_set_pid_values( int heater, const pid_struct* pid_settings);
extern int heater_get_pid_values( int heater, pid_struct* pid_settings);
extern int heater_set_setpoint( int heater, double setpoint);
extern int heater_get_setpoint( int heater, double* setpoint);

extern int heater_enable( heater_e heater, int state);
extern int heater_set_raw_pwm( int heater, double percentage);

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
