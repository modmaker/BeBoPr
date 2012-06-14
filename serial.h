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

#endif	/* _SERIAL_H */
