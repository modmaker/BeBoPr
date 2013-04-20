#ifndef	_GCODE_PROCESS_H
#define	_GCODE_PROCESS_H

#include <stdint.h>

#include "bebopr.h"
#include "gcode_parse.h"

// when we have a whole line, feed it to this
extern void process_gcode_command( GCODE_COMMAND* next_target);
extern void gcode_trace_move( void);
extern void gcode_set_axis_pos( axis_e axis, uint32_t pos);
extern int gcode_process_init( void);

#endif	/* _GCODE_PROCESS_H */
