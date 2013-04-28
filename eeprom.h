#ifndef _EEPROM_H
#define _EEPROM_H

#define POLOLU_DRIVERS 7
#define TB6560_DRIVERS 8

extern unsigned int eeprom_get_pru_code_offset( const unsigned int pru_nr);
extern int eeprom_write_pru_code( const char* eeprom_path, unsigned int pru_nr, const char* fname);

extern int eeprom_get_flag( const char* eeprom_path, int flag);
#define eeprom_get_step_io_config( path) eeprom_get_flag( path, 0)
#define eeprom_get_printer_config( path) eeprom_get_flag( path, 1)


#endif
