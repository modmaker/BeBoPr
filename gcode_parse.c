/** \file
	\brief Parse received G-Codes
*/


#include <stdio.h>
#include <string.h>

#include "gcode_parse.h"
#include "debug.h"
#include "gcode_process.h"


// stubs that replace code from sersendf and sermsg:
#define serwrite_uint8( x)  printf( "%u", x)
#define serwrite_uint16( x) printf( "%u", x)
#define serwrite_uint32( x) printf( "%u", x)
#define serwrite_int32( x)  printf( "%d", x)
#define serial_writechar( c) printf( "%c", c)
#define serial_writestr_P( s) printf( "%s", s)


/*
	Convert input values (mm/inch) into nanometers for all positions.
	This allows a build volume of +/- 2^31 nm -> +/- 2147 mm.

*/


#define	NM_PER_MM	1.0E6
#define	MM_PER_MM	1.0E0
#define	NM_PER_INCH	25.4E6
#define	MM_PER_INCH	25.4E0

/// crude crc macro
#define crc(a, b)		(a ^ b)

/// crude floating point data storage
decfloat read_digit;

/// this is where we store all the data for the current command before we work out what to do with it
static GCODE_COMMAND next_target;


/*
	decfloat_to_int() is the weakest subject to variable overflow. For evaluation, we assume a build room of +-1000 mm and NM_PER_MM_x between 1.000 and 4096. Accordingly for metric units:

		df->mantissa:  +-0..2147483147 (31 bits - 500 for rounding)
		df->exponent:  0, 2, 3 or 4    ( 7 bits)
		multiplicand / denominator:  20..4194303 / 1000 (22 bit - 10 bit) or
		                              0..4095 / 1       (12 bit -  0 bit)

	imperial units:

		df->mantissa:  +-0..32267      (15 bit - 500 for rounding)
		df->exponent:  0, 2, 3 or 4    (10 bit)
		multiplicand:  1..105000       (17 bit)
		denominator:   1 or 10         ( 4 bit)
*/
// accordingly:
#define DECFLOAT_EXP_MAX              7
#define DECFLOAT_MANT_MM_MAX 2147483647
#define DECFLOAT_MANT_IN_MAX   67108863


/// convert a floating point input value into an integer with appropriate scaling.
/// \param *df pointer to floating point structure that holds fp value to convert
/// \param multiplicand multiply by this amount during conversion to integer
/// \param divide_by_1000 divide by 1000 during conversion to integer
///
/// lots of work has been done in exploring this function's limitations in terms of overflow and rounding
/// this work may not be finished

/// Convert parsed floating point value (in mm or inch) to an integer in nm.

static int32_t decfloat_to_int( decfloat *df, double multiplicand)
{
	double		r = df->mantissa;
	uint8_t		e = df->exponent;

	r *= multiplicand;

	// e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen
	// a decimal point with one digit so it's too high by one if not zero

	if (e--) {
		while (e--) {
			r /= 10.0;
		}
	}

	return (int32_t) ((df->sign) ? -r : r);
}


static char gcode_text[ 200];
static int gcode_text_index = 0;

/// Character Received - add it to our command
/// \param c the next character to process
void gcode_parse_char(uint8_t c) {
	/// newline state variable
	/// used to compact any sequence of CR/LF characters to only one
	static uint8_t newline = 0;
	/// current or previous gcode word
	/// for working out what to do with data just received
	static uint8_t last_field = 0;

	if (gcode_text_index < sizeof( gcode_text) - 1) {
		if (c == '\t') {
			gcode_text[ gcode_text_index++] = ' ';
		} else if (c >= ' ') {
			gcode_text[ gcode_text_index++] = c;
		}
	}

	// uppercase
	if (c >= 'a' && c <= 'z')
		c &= ~32;

	// process previous field
	if (last_field) {
		// check if we're seeing a new field or end of line
		// any character will start a new field, even invalid/unknown ones
		if ((c >= 'A' && c <= 'Z') || c == '*' || (c == 10) || (c == 13)) {
			switch (last_field) {
				case 'G':
					next_target.G = read_digit.mantissa;
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint8(next_target.G);
					break;
				case 'M':
					next_target.M = read_digit.mantissa;
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint8(next_target.M);
					break;
				case 'X':
					if (next_target.option_inches)
						next_target.target.X = decfloat_to_int( &read_digit, NM_PER_INCH);
					else
						next_target.target.X = decfloat_to_int( &read_digit, NM_PER_MM);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_int32(next_target.target.X);
					break;
				case 'Y':
					if (next_target.option_inches)
						next_target.target.Y = decfloat_to_int( &read_digit, NM_PER_INCH);
					else
						next_target.target.Y = decfloat_to_int( &read_digit, NM_PER_MM);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_int32(next_target.target.Y);
					break;
				case 'Z':
					if (next_target.option_inches)
						next_target.target.Z = decfloat_to_int( &read_digit, NM_PER_INCH);
					else
						next_target.target.Z = decfloat_to_int( &read_digit, NM_PER_MM);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_int32(next_target.target.Z);
					break;
				case 'E':
					if (next_target.option_inches)
						next_target.target.E = decfloat_to_int( &read_digit, NM_PER_INCH);
					else
						next_target.target.E = decfloat_to_int( &read_digit, NM_PER_MM);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint32(next_target.target.E);
					break;
				case 'F':
					// just use raw integer, we need move distance and n_steps to convert it
					// to a useful value, so wait until we have those to convert it
					if (next_target.option_inches)
						next_target.target.F = decfloat_to_int(&read_digit, MM_PER_INCH);
					else
						next_target.target.F = decfloat_to_int(&read_digit, MM_PER_MM);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint32(next_target.target.F);
					break;
				case 'S':
					if (next_target.seen_M && (next_target.M == 220 || next_target.M == 221)) {
						// if this is a scaling factor, scale 1.0 to 1000
						next_target.S = decfloat_to_int( &read_digit, 1000.0);
					} else if (next_target.seen_M && (next_target.M == 113)) {
						// if this is PWM output, scale 1.0 to 100(%)
						next_target.S = decfloat_to_int( &read_digit, 100.0);
					} else {
						// if this is temperature, PID setting or anything else, scale 1:1
						next_target.S = decfloat_to_int( &read_digit, 1.0);
					}
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint16(next_target.S);
					break;
				case 'P':
					next_target.P = decfloat_to_int(&read_digit, 1.0);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint16(next_target.P);
					break;
				case 'T':
					next_target.T = read_digit.mantissa;
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint8(next_target.T);
					break;
				case 'N':
					next_target.N = decfloat_to_int(&read_digit, 1.0);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint32(next_target.N);
					break;
				case '*':
					next_target.checksum_read = decfloat_to_int(&read_digit, 1.0);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint8(next_target.checksum_read);
					break;
			}
			// reset for next field
			last_field = 0;
			read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;
		}
	}

	// skip comments
	if (next_target.seen_semi_comment == 0 && next_target.seen_parens_comment == 0) {
		// new field?
		if ((c >= 'A' && c <= 'Z') || c == '*') {
			last_field = c;
			if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
				serial_writechar(c);
		}

		// process character
		switch (c) {
			// each currently known command is either G or M, so preserve previous G/M unless a new one has appeared
			// FIXME: same for T command
			case 'G':
				next_target.seen_G = 1;
				next_target.seen_M = 0;
				next_target.M = 0;
				break;
			case 'M':
				next_target.seen_M = 1;
				next_target.seen_G = 0;
				next_target.G = 0;
				break;
			case 'X':
				next_target.seen_X = 1;
				break;
			case 'Y':
				next_target.seen_Y = 1;
				break;
			case 'Z':
				next_target.seen_Z = 1;
				break;
			case 'E':
				next_target.seen_E = 1;
				break;
			case 'F':
				next_target.seen_F = 1;
				break;
			case 'S':
				next_target.seen_S = 1;
				break;
			case 'P':
				next_target.seen_P = 1;
				break;
			case 'T':
				next_target.seen_T = 1;
				break;
			case 'N':
				next_target.seen_N = 1;
				break;
			case '*':
				next_target.seen_checksum = 1;
				break;

			// comments
			case ';':
				next_target.seen_semi_comment = 1;
				break;
			case '(':
				next_target.seen_parens_comment = 1;
				break;

			// now for some numeracy
			case '-':
				read_digit.sign = 1;
				// force sign to be at start of number, so 1-2 = -2 instead of -12
				read_digit.exponent = 0;
				read_digit.mantissa = 0;
				break;
			case '.':
				if (read_digit.exponent == 0)
					read_digit.exponent = 1;
				break;
			#ifdef	DEBUG
			case ' ':
			case '\t':
			case 10:
			case 13:
				// ignore
				break;
			#endif

			default:
				// can't do ranges in switch..case, so process actual digits here.
				if (c >= '0' && c <= '9') {
					if (read_digit.exponent < DECFLOAT_EXP_MAX &&
							((next_target.option_inches == 0 &&
							read_digit.mantissa < DECFLOAT_MANT_MM_MAX) ||
							(next_target.option_inches &&
							read_digit.mantissa < DECFLOAT_MANT_IN_MAX)))
					{
						// this is simply mantissa = (mantissa * 10) + atoi(c) in different clothes
						read_digit.mantissa = (10 * read_digit.mantissa) + (c - '0');
						if (read_digit.exponent)
							read_digit.exponent++;
					}
				}
				#ifdef	DEBUG
				else {
					// invalid
					serial_writechar('?');
					serial_writechar(c);
					serial_writechar('?');
				}
				#endif
		}
	} else if ( next_target.seen_parens_comment == 1 && c == ')')
		next_target.seen_parens_comment = 0; // recognize stuff after a (comment)

	if (next_target.seen_checksum == 0)
		next_target.checksum_calculated = crc(next_target.checksum_calculated, c);

	// end of line
	if (((c == 10) || (c == 13)) && (newline == 0)) {
		if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
			serial_writechar(c);

		if (
		#ifdef	REQUIRE_LINENUMBER
			((next_target.N >= next_target.N_expected) && (next_target.seen_N == 1)) ||
			(next_target.seen_M && (next_target.M == 110))
		#else
			1
		#endif
			) {
			if (
				#ifdef	REQUIRE_CHECKSUM
				((next_target.checksum_calculated == next_target.checksum_read) && (next_target.seen_checksum == 1))
				#else
				((next_target.checksum_calculated == next_target.checksum_read) || (next_target.seen_checksum == 0))
				#endif
				) {
				// process
				serial_writestr_P( "ok ");
				gcode_text[ gcode_text_index] = '\0';
				next_target.command_text = gcode_text;
				process_gcode_command( &next_target);
				gcode_text_index = 0;
				next_target.seen_G = 0;
				next_target.seen_M = 0;
				serial_writechar('\n');

				// expect next line number
				if (next_target.seen_N == 1)
					next_target.N_expected = next_target.N + 1;
			}
			else {
				printf( "rs N%d Expected checksum %d\n", next_target.N_expected, next_target.checksum_calculated);
// 				request_resend();
			}
		}
		else {
			printf( "rs N%d Expected line number %d\n", next_target.N_expected, next_target.N_expected);
// 			request_resend();
		}

		// reset variables
		next_target.seen_X = next_target.seen_Y = next_target.seen_Z = \
			next_target.seen_E = next_target.seen_F = next_target.seen_S = \
			next_target.seen_P = next_target.seen_T = next_target.seen_N = \
			next_target.seen_M = next_target.seen_checksum = next_target.seen_semi_comment = \
			next_target.seen_parens_comment = next_target.checksum_read = \
			next_target.checksum_calculated = 0;
		// last_field and read_digit are reset above already

		// assume a G1 by default
#if 0
		// TODO: disabling this keeps comments from generating moves. TEST !
		next_target.seen_G = 1;
		next_target.G = 1;
#endif
		if (next_target.option_relative) {
			next_target.target.X = next_target.target.Y = next_target.target.Z = 0;
			if (!config_e_axis_is_always_relative()) {
				next_target.target.E = 0;
			}
		}
		if (config_e_axis_is_always_relative()) {
			next_target.target.E = 0;
		}
		newline = 1;
	} else {
		newline = 0;
	}
}

/***************************************************************************\
*                                                                           *
* Request a resend of the current line - used from various places.          *
*                                                                           *
* Relies on the global variable next_target.N being valid.                  *
*                                                                           *
\***************************************************************************/

void request_resend(void) {
	serial_writestr_P( "rs ");
	serwrite_uint8( next_target.N);
	serial_writechar( '\n');
}
