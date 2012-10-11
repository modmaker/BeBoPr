#ifndef	_ANALOG_H
#define	_ANALOG_H

#include "beaglebone.h"

#define ANALOG_CYCLE_TIME         20000 /* usecs, sensor readout cycle */
#define ANALOG_UPDATE_CYCLE_TIME 200000 /* usecs, update interval callbacks */

typedef const struct {
  channel_tag		tag;
  const char*		device_path;
  unsigned int		filter_length;
} analog_config_record;

typedef int (update_callback)( channel_tag channel, int new_value);

extern int analog_init( void);
extern int analog_set_update_callback( channel_tag analog_channel, update_callback* pupdate, channel_tag update_channel);
extern int analog_config( analog_config_record* pconfig_data, int nr_config_items);
// Do not use this, for debugging only!
extern int analog_get_raw_value( channel_tag analog_channel, int* pvalue);

#endif
