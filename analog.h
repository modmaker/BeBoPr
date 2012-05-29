#ifndef	_ANALOG_H
#define	_ANALOG_H

typedef enum {
  e_analog_1,
  e_analog_2,
  e_analog_3,
  e_analog_num_channels	// keep last !
} analog_channel_e;

typedef int update_channel_t;
typedef int (update_callback)( update_channel_t, int);

extern int analog_init( void);
extern int analog_set_update_callback( analog_channel_e, update_callback*, update_channel_t);

#endif
