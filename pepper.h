#ifndef _PEPPER_H
#define _PEPPER_H

#ifdef PEPPER

#include <stdint.h>

extern void pepper_send_byte( uint8_t data);
extern int pepper_init( void);

#endif

#endif
