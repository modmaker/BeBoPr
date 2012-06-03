#ifndef	_SERIAL_H
#define	_SERIAL_H

#include	<stdint.h>


// initialise serial subsystem
int serial_init(void);

// return number of characters in the receive buffer, and number of spaces in the send buffer
uint32_t serial_rxchars(void);
// uint8_t serial_txchars(void);

// read one character
uint8_t serial_popchar(void);
// send one character
void serial_writechar(uint8_t data);

// read/write many characters
// uint8_t serial_recvblock(uint8_t *block, int blocksize);
void serial_writeblock(void *data, int datalen);

#if ARCH == arm
void serial_writestr( const char* data);
#else
void serial_writestr(uint8_t *data);
#endif


#if ARCM == arm
#define pgm_read_word( p) (uint16_t)*(p)
#define pgm_read_byte( p) (uint8_t)*(p)
#define serial_writeblock_P serial_writeblock
#define serial_writestr_P serial_writestr
#else
// write from flash
void serial_writeblock_P(PGM_P data, int datalen);
void serial_writestr_P(PGM_P data);
#endif

#endif	/* _SERIAL_H */
