#ifndef _BEBOPR_H
#define _BEBOPR_H

extern int bebopr_pre_init( void);

// all return either 0 or 1
extern int limsw_x_has_max( void);
extern int limsw_x_has_min( void);
extern int limsw_y_has_max( void);
extern int limsw_y_has_min( void);
extern int limsw_z_has_max( void);
extern int limsw_z_has_min( void);

// all return either 0 or 1
extern int limsw_x_max_is_active_low( void);
extern int limsw_x_min_is_active_low( void);
extern int limsw_y_max_is_active_low( void);
extern int limsw_y_min_is_active_low( void);
extern int limsw_z_max_is_active_low( void);
extern int limsw_z_min_is_active_low( void);

// determines stepper driver control
extern int use_pololu_drivers( void);

#endif
