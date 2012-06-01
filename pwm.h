#ifndef _PWM_H
#define _PWM_H

#include "beaglebone.h"

typedef const struct {
  channel_tag		tag;
  const char*		device_path;
  unsigned int		frequency;
} pwm_config_record;

extern int pwm_init( void);
extern int pwm_config( pwm_config_record* pconfig_data, int nr_config_items);

extern int pwm_set_output( channel_tag pwm_channel, unsigned int percentage);

#endif
