#ifndef _EEPROM_H
#define _EEPROM_H

#define POLOLU_DRIVERS 7
#define TB6560_DRIVERS 8

extern int eeprom_get_step_io_config( const char* eeprom_path);
extern int eeprom_set_step_io_config( const char* eeprom_path, uint8_t value);
extern unsigned int eeprom_get_pru_code_offset( const unsigned int pru_nr);
extern int eeprom_write_pru_code( const char* eeprom_path, unsigned int pru_nr, const char* fname);

#endif
