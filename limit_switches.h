#ifndef _LIMIT_SWITCHES_H
#define _LIMIT_SWITCHES_H

enum {
  e_x_min,
  e_x_max,
  e_y_min,
  e_y_max,
  e_z_min,
  e_z_max,
  e_num_limit_switches
} limit_switches_e;

extern int limsw_init( void);

// return 1 if switch is activated, 0 otherwise
extern int limsw_x_max( void);
extern int limsw_x_min( void);
extern int limsw_y_max( void);
extern int limsw_y_min( void);
extern int limsw_z_max( void);
extern int limsw_z_min( void);



#endif
