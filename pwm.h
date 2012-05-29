#ifndef _PWM_H
#define _PWM_H

typedef enum {
  e_pwm_output_1,
  e_pwm_output_2,
  e_pwm_output_3,
  e_pwm_num_outputs
} pwm_output_e;

extern int pwm_init( void);
extern int pwm_set_output( double percentage);

#endif
