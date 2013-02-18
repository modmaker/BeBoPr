/*
 * compile with:
 *	ARCH=arm ${CROSS_COMPILE}gcc -std=c99 eeprom.c -DSTAND_ALONE -o eeprom-tool
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stddef.h>
#include <stdlib.h>

#include "bebopr.h"
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

struct pru_code_block {
  uint32_t                   opcodes[ 2048];
};

struct eeprom {
  union bone_cape_io_config cape_io_config;
  uint8_t                   step_io_config;
  uint8_t                   reserved[ 11];
  struct pru_code_block     pru0_code;
  struct pru_code_block     pru1_code;
};

int eeprom_get_step_io_config( const char* eeprom_path)
{
  uint8_t step_io_config;
  int fd = open( eeprom_path, O_RDONLY);
  int result;
  if (fd < 0) {
    perror( "Failed to open EEPROM for reading");
    result = -1;
    goto done;
  }
  result = lseek( fd, offsetof( struct eeprom, step_io_config), SEEK_SET);
  if (result < 0) {
    perror( "Failed to lseek EEPROM");
    result = -1;
    goto done;
  }
  int cnt = read( fd, &step_io_config, sizeof( step_io_config));
  if (cnt < 0) {
    perror( "Failed to read EEPROM");
    result = -1;
    goto done;
  } else if (cnt != sizeof( step_io_config)) {
    // short write ?
    result = -1;
    goto done;
  }
  result = step_io_config;
done:
  close( fd);
  return result;
}


int eeprom_set_step_io_config( const char* eeprom_path, uint8_t value)
{
  uint8_t step_io_config = value;
  int fd = open( eeprom_path, O_WRONLY);
  int result;
  if (fd < 0) {
    perror( "Failed to open EEPROM for writing");
    result = -1;
    goto done;
  }
  result = lseek( fd, offsetof( struct eeprom, step_io_config), SEEK_SET);
  if (result < 0) {
    perror( "Failed to lseek EEPROM");
    result = -1;
    goto done;
  }
  int cnt = write( fd, &step_io_config, sizeof( step_io_config));
  if (cnt < 0) {
    perror( "Failed to read EEPROM");
    result = -1;
    goto done;
  } else if (cnt != sizeof( step_io_config)) {
    // short write ?
    result = -1;
    goto done;
  }
  result = 0;
done:
  close( fd);
  return result;
}

unsigned int eeprom_get_pru_code_offset( int pru_nr)
{
  unsigned int offset = 0;
  switch (pru_nr) {
  case 0: offset = offsetof( struct eeprom, pru0_code); break;
  case 1: offset = offsetof( struct eeprom, pru1_code); break;
  }
  return offset;
}

/*
 *  Write the file specified by fname to a pru code block in EEPROM.
 */
int eeprom_write_pru_code( const char* eeprom_path, int pru_nr, const char* fname)
{
  unsigned int offset = eeprom_get_pru_code_offset( pru_nr);
  int result;
  int ee_fd = -1;
  int fs_fd = -1;
  // Open destination (EEPROM) file at specified offset for writing
  ee_fd = open( eeprom_path, O_WRONLY);
  if (ee_fd < 0) {
    perror( "Failed to open EEPROM for writing");
    result = -1;
    goto done;
  }
  result = lseek( ee_fd, offset, SEEK_SET);
  if (result < 0) {
    perror( "Failed to lseek EEPROM");
    result = -1;
    goto done;
  }
  if (result != offset) {
    fprintf( stderr, "lseek returned position %d\n", result);
  }
  // Open source file for reading
  fs_fd = open( fname, O_RDONLY);
  if (fs_fd < 0) {
    perror( "Failed to open source for reading");
    result = -1;
    goto done;
  }
  //  Write contents of file to EEPROM
  for (int i = 0 ; i < 2048 ; ++i) {
    uint32_t opcode;
    int count = read( fs_fd, &opcode, sizeof( opcode));
    if (count != sizeof( opcode)) {
      // No more data (or some other failure), clear EEPROM location
      opcode = 0xFFFFFFFF;
    }
    count = write( ee_fd, &opcode, sizeof( opcode));
    if (count != sizeof( opcode)) {
      perror( "Failed to write to EEPROM");
      fprintf( stderr, "Failed to write opcode[%d] to EEPROM\n", i);
      result = -1;
      goto done;
    }
  }
  //  Verify EEPROM contents with file
  result = lseek( fs_fd, 0, SEEK_SET);
  close( ee_fd);
  ee_fd = open( eeprom_path, O_RDONLY);
  if (ee_fd < 0) {
    perror( "Failed to open EEPROM for reading");
    result = -1;
    goto done;
  }
  result = lseek( ee_fd, offset, SEEK_SET);
  for (int i = 0 ; i < 2048 ; ++i) {
    uint32_t ee_opcode;
    uint32_t fs_opcode;
    int count = read( fs_fd, &fs_opcode, sizeof( fs_opcode));
    if (count != sizeof( fs_opcode)) {
      // No more data (or some other failure), clear EEPROM location
      fs_opcode = 0xFFFFFFFF;
    }
    count = read( ee_fd, &ee_opcode, sizeof( ee_opcode));
    if (count != sizeof( ee_opcode)) {
      perror( "Failed to read from EEPROM");
      fprintf( stderr, "Failed to read opcode[%d] from EEPROM\n", i);
      result = -1;
      goto done;
    }
    if (ee_opcode != fs_opcode) {
      fprintf( stderr, "EEPROM verification failed at opcode[%d]: is %08x, should be %08x.\n",
              i, ee_opcode, fs_opcode);
      result = -1;
      goto done;
    }
  }
  // Success
  result = 0;

done:
  if (fs_fd >= 0) {
    close( fs_fd);
  }
  if (ee_fd >= 0) {
    close( ee_fd);
  }
  return result;
}

//----------------------------------------------------------------
#ifdef STANDALONE
//----------------------------------------------------------------


void usage( void)
{
  printf( "Usage:\n");
  printf( " -p<pru-nr> -f<filename>\n");
  printf( " -s<flagnr>\n");
  printf( " -c<flagnr>\n");
  exit( 1);
}

int main( int argc, char* argv[])
{
  int c;
  unsigned int pru_nr = 1;	// default PRU nr
  char fname[ 250] = { 0 };

  while ((c = getopt( argc, argv, "p:f:c:s:")) != -1) {
    switch (c) {
    case 'p':	// set pru nr
      pru_nr = atoi( optarg);
      break;
    case 'f':	// set filename
      strncpy( fname, optarg, sizeof( fname) - 1);
      break;
    case 'c':	// clear flag
      break;
    case 's':	// set flag
      break;
    default:
      usage();
    }
  }
  if (optind < argc) {
    usage();
  }
  if (*fname) {
    printf( "Writing code from file '%s' for PRU%d to EEPROM.\n(this may take a while!\n", fname, pru_nr);
    int result = eeprom_write_pru_code( EEPROM_PATH, pru_nr, fname);
  }

#if 0
  int result = eeprom_get_step_io_config( EEPROM_PATH);
  printf( "Current EEPROM step_io_config value is: 0x%02x (%d)\n", result, result);
  result = eeprom_set_step_io_config( EEPROM_PATH, TB6560_DRIVERS);
  result = eeprom_get_step_io_config( EEPROM_PATH);
  printf( "New EEPROM step_io_config value is: 0x%02x (%d)\n", result, result);
#endif
  return 0;
}

#endif
