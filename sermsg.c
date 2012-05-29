#include	"sermsg.h"

/** \file sermsg.c
	\brief primitives for sending numbers over the serial link
*/

#include	"serial.h"

/** write a single hex digit
	\param v hex digit to write, higher nibble ignored
*/
void serwrite_hex4(uint8_t v) {
	v &= 0xF;
	if (v < 10)
		serial_writechar('0' + v);
	else
		serial_writechar('A' - 10 + v);
}

/** write a pair of hex digits
	\param v byte to write. One byte gives two hex digits
*/
void serwrite_hex8(uint8_t v) {
	serwrite_hex4(v >> 4);
	serwrite_hex4(v & 0x0F);
}

/** write four hex digits
	\param v word to write
*/
void serwrite_hex16(uint16_t v) {
	serwrite_hex8(v >> 8);
	serwrite_hex8(v & 0xFF);
}

/** write eight hex digits
	\param v long word to write
*/
void serwrite_hex32(uint32_t v) {
	serwrite_hex16(v >> 16);
	serwrite_hex16(v & 0xFFFF);
}

/** write sixteen hex digits
	\param v long long word to write
*/
void serwrite_hex64(uint64_t v) {
	serwrite_hex32(v >> 32);
	serwrite_hex32(v & 0xFFFFFFFF);
}

/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
//const uint32_t powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
const uint64_t powers[] = {
	1,
	10,
	100,					// 2 - 8 bits
	1000,
	10000,					// 4 - 16 bits
	100000,
	1000000,
	10000000,
	100000000,
	1000000000,				// 9 - 32 bits 
	10000000000,
	100000000000,
	1000000000000,
	10000000000000,
	100000000000000,		// 14 - 48 bits
	1000000000000000,
	10000000000000000,
	100000000000000000,
	1000000000000000000,
	10000000000000000000U,	// 19 - 64 bits
};

/** write decimal digits from a long unsigned int
	\param v number to send
*/
void serwrite_uint64( uint64_t v64) {
	int8_t e;
	uint8_t t;
	uint32_t v32;
	uint16_t v16;
	uint8_t started = 0;

	// speedup for small numbers
	if ((v64 & 0xFFFFFFFFFFFF0000ULL) == 0) {
		// 16-bits
		e = 4;
	} else if ((v64 & 0xFFFFFFFF00000000ULL) == 0) {
		// 32-bits
		e = 9;
	} else if ((v64 & 0xFFFF000000000000ULL) == 0) {
		// 48-bits
		e = 14;
	} else {
		// 64-bits
		e = 19;
	}
	while (e >= 9) {
		if (v64 >= powers[ e]) {
			started = 1;
			break;
		}
		--e;
	}
	while (e >= 9) {
		for (t = 0; v64 >= powers[ e]; ++t) {
			v64 -= powers[ e];
		}
		serial_writechar( t + '0');
		--e;
	}
	v32 = v64 & 0xFFFFFFFFUL;

	while (started == 0 && e >= 4) {
		if (v32 >= (uint32_t) (powers[ e] & 0xFFFFFFFFUL)) {
			started = 1;
			break;
		}
		--e;
	}
	while (e >= 4) {
		for (t = 0; v32 >= (uint32_t) (powers[ e] & 0xFFFFFFFFUL); ++t) {
			v32 -= (uint32_t) (powers[ e] & 0xFFFFFFFFUL);
		}
		serial_writechar( t + '0');
		--e;
	}
	v16 = v32 & 0xFFFFU;

	while (started == 0 && e > 0) {
		if (v16 >= (uint16_t) (powers[ e] & 0xFFFFU)) {
			started = 1;
			break;
		}
		--e;
	}
	while (e >= 0) {
		for (t = 0; v16 >= (uint16_t) (powers[ e] & 0xFFFFU); ++t) {
			v16 -= (uint16_t) (powers[ e] & 0xFFFFU);
		}
		serial_writechar( t + '0');
		--e;
	}
}

/** write decimal digits from a long signed int
	\param v number to send
*/
void serwrite_int64( int64_t v) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint64(v);
}

/** write decimal digits from a long unsigned int
\param v number to send
*/
void serwrite_uint32_vf(uint32_t v, uint8_t fp) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	if (e < fp)
		e = fp;

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
		serial_writechar(t + '0');
		if (e == fp)
			serial_writechar('.');
	}
	while (e--);
}

/** write decimal digits from a long signed int
\param v number to send
*/
void serwrite_int32_vf(int32_t v, uint8_t fp) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint32_vf(v, fp);
}
