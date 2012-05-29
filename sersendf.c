#include	"sersendf.h"

/** \file sersendf.c
	\brief Simplified printf implementation
*/

#include	<stdarg.h>

#include	"serial.h"
#include	"sermsg.h"

/** \brief Simplified printf
	\param format pointer to output format specifier string stored in FLASH.
	\param ... output data

	Implements only a tiny subset of printf's format specifiers :-

	%[ls][udcx%]

	l - following data is (32 bits)\n
	s - following data is short (8 bits)\n
	none - following data is 16 bits.

	u - unsigned int\n
	d - signed int\n
	c - character\n
	x - hex\n
	% - send a literal % character

	Example:

	\code sersendf_P(PSTR("X:%ld Y:%ld temp:%u.%d flags:%sx Q%su/%su%c\n"), target.X, target.Y, current_temp >> 2, (current_temp & 3) * 25, dda.allflags, mb_head, mb_tail, (queue_full()?'F':(queue_empty()?'E':' '))) \endcode
*/
void sersendf_P(PGM_P format, ...) {
	va_list args;
	va_start(args, format);

	uint16_t i = 0;
	uint8_t c = 1, j = 0;
	while ((c = pgm_read_byte(&format[i++]))) {
		if (j) {
			switch(c) {
				case 's':
					j = 1;
					break;
				case 'l':
					if (j == 2) {
						j = 4;
					} else if (j == 4) {
						j = 8;
					}
					break;
				case 'u':
					if (j == 8)
						serwrite_uint64(va_arg(args, uint64_t));
					else if (j == 4)
						serwrite_uint32(va_arg(args, uint32_t));
					else
						serwrite_uint16( (uint16_t)va_arg( args, uint32_t));
					j = 0;
					break;
				case 'd':
					if (j == 8)
						serwrite_int64(va_arg(args, int64_t));
					else if (j == 4)
						serwrite_int32(va_arg(args, int32_t));
					else
						serwrite_int16( (int16_t)va_arg( args, int32_t));
					j = 0;
					break;
				case 'c':
					serial_writechar( (uint16_t)va_arg( args, uint32_t));
					j = 0;
					break;
				case 'x':
					serial_writestr( "0x");
					if (j == 8)
						serwrite_hex64(va_arg(args, uint64_t));
					else if (j == 4)
						serwrite_hex32(va_arg(args, uint32_t));
					else if (j == 1)
						serwrite_hex8( (uint16_t)va_arg( args, uint32_t));
					else
						serwrite_hex16( (uint16_t)va_arg( args, uint32_t));
					j = 0;
					break;
/*				case 'p':
					serwrite_hex16(va_arg(args, uint16_t));*/
				case 'q':
					serwrite_int32_vf(va_arg(args, int32_t), 3);
					j = 0;
					break;
				default:
					serial_writechar(c);
					j = 0;
					break;
			}
		}
		else {
			if (c == '%') {
				j = 2;
			}
			else {
				serial_writechar(c);
			}
		}
	}
	va_end(args);
}
