#ifndef _PWM_H
#define _PWM_H

#include "beaglebone.h"

extern int pwm_init( void);
extern int pwm_set_output( channel_tag ch, unsigned int percentage);

#endif
