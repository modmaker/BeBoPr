#ifndef	_ANALOG_H
#define	_ANALOG_H

typedef struct {
  const char*		device_path;
  unsigned int		filter_length;
} analog_config_struct;

typedef int update_channel_t;
typedef int (update_callback)( update_channel_t, int);

extern int analog_init( void);
extern int analog_set_update_callback( unsigned int analog_channel, update_callback* update, update_channel_t temp_channel);
extern int analog_config( const analog_config_struct* config_data, int nr_config_items);
// Do not use this, for debugging only!
extern int analog_get_raw_value( unsigned int analog_channel, int* value);

#endif
