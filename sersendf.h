#ifndef	_SERSENDF_H
#define	_SERSENDF_H

#include <stdarg.h>

#if ARCH == arm
#if defined( PGM_P)
#undef PGM_P
#endif
#define PGM_P char*
#define PSTR( s) s
#else
#include	<avr/pgmspace.h>
#endif

//void sersendf_P(PGM_P format, ...)	__attribute__ ((format (printf, 1, 2)));

#if ARCH != arm

//#define sersendf_P sersendf
//#define PGM_P

void sersendf( const char *format, ...);
void sersendf_P( const char *format, ...);

#else

void sersendf(char *format, ...)		__attribute__ ((format (printf, 1, 2)));
void sersendf_P(PGM_P format, ...)	__attribute__ ((format (printf, 1, 2)));

#endif

#endif	/* _SERSENDF_H */
