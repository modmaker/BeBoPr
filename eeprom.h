#ifndef _EEPROM_H
#define _EEPROM_H

#define POLOLU_DRIVERS 7
#define TB6560_DRIVERS 8

extern int get_step_io_config( const char* eeprom_path);
extern int set_step_io_config( const char* eeprom_path, uint8_t value);

#endif
