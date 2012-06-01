#ifndef	_GCODE_PROCESS_H
#define	_GCODE_PROCESS_H

#include	"gcode_parse.h"

// the current tool
extern uint8_t tool;
// the tool to be changed when we get an M6
extern uint8_t next_tool;

// when we have a whole line, feed it to this
extern void process_gcode_command( void);
extern void gcode_trace_move( void);
extern void gcode_set_pos( char c, uint32_t pos);
extern int gcode_process_init( void);

#endif	/* _GCODE_PROCESS_H */
