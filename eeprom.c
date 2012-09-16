/*
 * compile with:
 *	ARCH=arm arm-arago-linux-gnueabi-gcc eeprom-decode.c -o eeprom-decode
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>

#include "beaglebone.h"
#include "eeprom.h"


#define EEPROM_MAGIC 0xAA5533EE

#define BIG_ENDIAN_8(  x) ( (x) & 0xFF)
#define BIG_ENDIAN_16( x) (( (BIG_ENDIAN_8(  x) <<  8) | BIG_ENDIAN_8(  x >>  8) ) & 0xFFFF)
#define BIG_ENDIAN_32( x) (( (BIG_ENDIAN_16( x) << 16) | BIG_ENDIAN_16( x >> 16) ) & 0xFFFFFFFF)


union bone_cape_io_config {
  struct {
    uint32_t	header;
    char	header_version[ 2];
    char	cape_name[ 32];
    char	cape_revision[ 4];
    char	manufacturer[ 16];
    char	part_number[ 16];
    uint16_t	pins_used;
    char	serial_number[ 12];	/* WWYY4P13nnnn */
    uint16_t	pin_config[ 74];
    uint16_t	vdd_3v3_current;
    uint16_t	vdd_5v0_current;
    uint16_t	sys_5v0_current;
    uint16_t	dc_supplied;
  };
  struct {
    uint8_t	data[ 244];
  };
};

struct eeprom {
  union bone_cape_io_config cape_io_config;
  uint8_t                   step_io_config;
  // put microcode here ?
};

int get_step_io_config( const char* eeprom_path)
{
  uint8_t step_io_config;
  int fd = open( eeprom_path, O_RDONLY);
  int result = -1;
  if (fd < 0) {
    perror( "Failed to open EEPROM for reading");
  } else {
    if (lseek( fd, sizeof( union bone_cape_io_config), SEEK_SET) < 0) {
      perror( "Failed to lseek EEPROM");
    } else {
      int cnt = read( fd, &step_io_config, sizeof( step_io_config));
      if (cnt < 0) {
        perror( "Failed to read EEPROM");
      } else if (cnt != sizeof( step_io_config)) {
        // short write ?
      } else {
        result = step_io_config;
      }
    }
    close( fd);
  }
  return result;
}


int set_step_io_config( const char* eeprom_path, uint8_t value)
{
  uint8_t step_io_config = value;
  int fd = open( eeprom_path, O_WRONLY);
  int result = -1;
  if (fd < 0) {
    perror( "Failed to open EEPROM for writing");
  } else {
    if (lseek( fd, sizeof( union bone_cape_io_config), SEEK_SET) < 0) {
      perror( "Failed to lseek EEPROM");
    } else {
      int cnt = write( fd, &step_io_config, sizeof( step_io_config));
      if (cnt < 0) {
        perror( "Failed to read EEPROM");
      } else if (cnt != sizeof( step_io_config)) {
        // short write ?
      } else {
        result = 0;
      }
    }
    close( fd);
  }
  return result;
}


#ifdef STANDALONE

/*
 *  This code can also be used to alter the io_config setting.
 */
int main( int argc, char* argv[])
{
  int result = get_step_io_config( EEPROM_PATH);
  printf( "Current EEPROM step_io_config value is: 0x%02x (%d)\n", result, result);
  result = set_step_io_config( EEPROM_PATH, TB6560_DRIVERS);
  result = get_step_io_config( EEPROM_PATH);
  printf( "New EEPROM step_io_config value is: 0x%02x (%d)\n", result, result);
  return 0;
}

#endif
