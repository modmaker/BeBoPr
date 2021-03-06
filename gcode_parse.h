#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include	<stdint.h>

// target is simply a point in space/time
typedef struct {
	int32_t						X;
	int32_t						Y;
	int32_t						Z;
	int32_t						E;
	uint32_t					F;
} TARGET;

// wether to insist on N line numbers
// if not defined, N's are completely ignored
//#define	REQUIRE_LINENUMBER

// wether to insist on a checksum
//#define	REQUIRE_CHECKSUM

/// this is a very crude decimal-based floating point structure.
/// a real floating point would at least have signed exponent.\n
/// resulting value is \f$ mantissa * 10^{-(exponent - 1)} * ((sign * 2) - 1)\f$
typedef struct {
  double	mantissa;		///< the actual digits of our floating point number
  uint8_t	exponent	:7;	///< scale mantissa by \f$10^{-exponent}\f$
  uint8_t	sign		:1;	///< positive or negative?
} decfloat;

/// this holds all the possible data from a received command
typedef struct {
  union {
    struct {
      uint8_t		seen_G			:1;
      uint8_t		seen_M			:1;
      uint8_t		seen_X			:1;
      uint8_t		seen_Y			:1;
      uint8_t		seen_Z			:1;
      uint8_t		seen_E			:1;
      uint8_t		seen_F			:1;
      uint8_t		seen_S			:1;

      uint8_t		seen_P			:1;
      uint8_t		seen_T			:1;
      uint8_t		seen_N			:1;
      uint8_t		seen_checksum		:1; ///< seen a checksum?
      uint8_t		seen_semi_comment	:1; ///< seen a semicolon?
      uint8_t		seen_parens_comment	:1; ///< seen an open parenthesis
      uint8_t		option_relative		:1; ///< relative or absolute coordinates?
      uint8_t		option_inches		:1; ///< inches or millimeters?
    };
    uint16_t		flags;
  };

  uint8_t		G;			///< G command number
  uint8_t		M;			///< M command number
  TARGET		target;			///< target position: X, Y, Z, E and F

  int16_t		S;			///< S word (various uses)
  uint16_t		P;			///< P word (various uses)

  uint8_t		T;			///< T word (tool index)

  uint32_t		N;			///< line number
  uint32_t		N_expected;		///< expected line number

  uint8_t		checksum_read;		///< checksum in gcode command
  uint8_t		checksum_calculated;	///< checksum we calculated

  char*	command_text;
} GCODE_COMMAND;

/// accept the next character and process it
void gcode_parse_char(uint8_t c);

// uses the global variable next_target.N
void request_resend(void);

#endif	/* _GCODE_PARSE_H */
