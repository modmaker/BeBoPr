/*
 * This file contains the EEPROM contents definition for the BeBoPr and
 * interfaces to manipulate the contents.
 *
 * The first 244 bytes of the EEPROM are defined by the BeagleBone TRM and
 * are used to configure the BeagleBone I/O properly during kernel start.
 * The remainder is used by the BeBoPr to store flags and PRU code.
 *
 * This code can be compiled in STANDALONE mode to generated a commandline tool:
 * compile with:
 *	ARCH=arm ${CROSS_COMPILE}gcc -std=c99 eeprom.c -DSTANDALONE -o eeprom-tool
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
 /*
  *  Current flag assignment:
  *   flag 0 - stepper kind configuration (Pololu)
  *   flag 1 - bebopr configuration id
  */
  uint8_t                   flags[ 12];
  struct pru_code_block     pru0_code;
  struct pru_code_block     pru1_code;
};

//----------------------------------------------------------------

#define member_size( type, member) sizeof( ((type *)0)->member)

unsigned int eeprom_get_pru_code_offset( unsigned int pru_nr)
{
  unsigned int offset = 0;
  switch (pru_nr) {
  case 0: offset = offsetof( struct eeprom, pru0_code); break;
  case 1: offset = offsetof( struct eeprom, pru1_code); break;
  }
  return offset;
}

unsigned int eeprom_get_flag_offset( unsigned int flag_nr)
{
  // always return a legal offset, even for a bad flag_nr value!
  unsigned int offset = offsetof( struct eeprom, flags);
  if (flag_nr < member_size( struct eeprom, flags)) {
    offset += flag_nr;
  }
  return offset;
}

int eeprom_read_block( const char* eeprom_path, uint8_t* data, unsigned int datacount, unsigned int offset)
{
  int ee_fd = open( eeprom_path, O_RDONLY);
  if (ee_fd < 0) {
    perror( "Failed to open EEPROM for reading");
    return -1;
  }
  if (lseek( ee_fd, offset, SEEK_SET) < 0) {
    perror( "Failed to seek EEPROM");
    return -1;
  }
  int count = read( ee_fd, data, datacount);
  if (count != datacount) {
    perror( "Failed to read from EEPROM");
    fprintf( stderr, "Failed to read single byte at offset %d from EEPROM\n", offset);
    return -1;
  }
  return 0;
}

int eeprom_read_byte( const char* eeprom_path, uint8_t* data, unsigned int offset)
{
  return eeprom_read_block( eeprom_path, data, 1, offset);
}

/*
 *  Write the datablock determined by 'data' and 'datacount' into EEPROM at the specified offset.
 *
 *  NOTE: This is rather slow, measurements show that with a I2C bus
 *        operating at 100 kHz, every 20 ms one word is written.
 *        So 2k words written at 50 Hz takes around 41 seconds.
 */
int eeprom_write_block( const char* eeprom_path, uint8_t* data, unsigned int datacount, unsigned int offset)
{
  int result;
  int ee_fd = -1;
  int i;
  // Open destination (EEPROM) file at specified offset for writing
  ee_fd = open( eeprom_path, O_WRONLY);
  if (ee_fd < 0) {
    perror( "Failed to open EEPROM for writing");
    result = -1;
    goto done;
  }
  result = lseek( ee_fd, offset, SEEK_SET);
  if (result < 0 || result != offset) {
    perror( "Failed to lseek EEPROM");
    result = -1;
    goto done;
  }
  //  Write data to EEPROM
  int chunksize = 16;
  for (i = 0 ; i < datacount ; i += chunksize) {
    if (i + chunksize > datacount) {
	    chunksize = (int)datacount - i;
    }
    int count = write( ee_fd, &data[ i], chunksize);
    if (count != chunksize) {
      perror( "Failed to write to EEPROM");
      if (count >= 0) {
        fprintf( stderr, "Short write (%d) at byte %d.\n", count, i);
      }
      result = -1;
      goto done;
    }
  }
  //  Verify EEPROM contents against data
  close( ee_fd);
  ee_fd = open( eeprom_path, O_RDONLY);
  if (ee_fd < 0) {
    perror( "Failed to open EEPROM for reading");
    result = -1;
    goto done;
  }
  result = lseek( ee_fd, offset, SEEK_SET);
  for (i = 0 ; i < datacount ; ++i) {
    uint8_t ee_data;
    int count = read( ee_fd, &ee_data, sizeof( ee_data));
    if (count != sizeof( ee_data)) {
      perror( "Failed to read from EEPROM");
      if (count >= 0) {
        fprintf( stderr, "Short read (%d) at byte %d.\n", count, i);
      }
      result = -1;
      goto done;
    }
    if (ee_data != data[ i]) {
      fprintf( stderr, "EEPROM verification failed at opcode[%d]: is %02x, should be %02x.\n",
              i, ee_data, data[ i]);
      result = -1;
      goto done;
    }
  }
  // Success
  result = 0;
done:
  if (ee_fd >= 0) {
    close( ee_fd);
  }
  return result;
}

/*
 *  Write the file specified by fname to a pru code block in EEPROM.
 *
 *  NOTE: This is rather slow, measurements show that with a I2C bus
 *        operating at 100 kHz, every 20 ms one word is written.
 *        So 2k words written at 50 Hz takes around 41 seconds.
 */
int eeprom_write_pru_code( const char* eeprom_path, unsigned int pru_nr, const char* fname)
{
  unsigned int offset = eeprom_get_pru_code_offset( pru_nr);
  int result;
  int fs_fd = -1;
  uint32_t data[ 2048];		// PRU instruction memory size
  // Open source file for reading
  fs_fd = open( fname, O_RDONLY);
  if (fs_fd < 0) {
    perror( "Failed to open source for reading");
    result = -1;
    goto done;
  }
  memset( &data, 0xFF, sizeof( data));
  int datacount = read( fs_fd, &data, sizeof( data));
  if (datacount < 0) {
    perror( "Failed to read from file");
    result = -1;
    goto done;
  }
  result = eeprom_write_block( eeprom_path, (void*)data, (unsigned)datacount, offset);
  if (result < 0) {
    result = -1;
    goto done;
  }
  // Success
  result = 0;

done:
  if (fs_fd >= 0) {
    close( fs_fd);
  }
  return result;
}

//----------------------------------------------------------------

int eeprom_write_flag( const char* eeprom_path, unsigned int flag_nr, uint8_t value)
{
  unsigned int offset = eeprom_get_flag_offset( flag_nr);
  return eeprom_write_block( eeprom_path, &value, 1, offset);
}

int eeprom_read_flag( const char* eeprom_path, unsigned int flag_nr, uint8_t* value)
{
  unsigned int offset = eeprom_get_flag_offset( flag_nr);
  return eeprom_read_byte( eeprom_path, value, offset);
}

//----------------------------------------------------------------

/*
 *  Application specific interface, these map the assigned flags
 *  Only read calls are provided, writing is done with the eeprom-tool!
 */

int eeprom_get_flag( const char* eeprom_path, int flagno)
{
  uint8_t flag;
  int result = eeprom_read_flag( eeprom_path, flagno, &flag);
  if (result < 0) {
    return result;
  }
  return flag;
}

//----------------------------------------------------------------
#ifdef STANDALONE
//----------------------------------------------------------------

#include "beaglebone.h"

static void eeprom_dump( const char* eeprom_path, unsigned int offset, unsigned int bytecount, unsigned int wordsize)
{
  int i;
  int ee_fd = open( eeprom_path, O_RDONLY);
  if (ee_fd < 0) {
    perror( "Failed to open EEPROM for reading");
    return;
  }
  int result = lseek( ee_fd, offset, SEEK_SET);
  int rowsize = (wordsize == 4) ? 32 : 16;
  for (i = 0 ; i < bytecount ; i += wordsize) {
    uint32_t ee_code = 0;
    int count = read( ee_fd, &ee_code, wordsize);
    if (count != wordsize) {
      perror( "Failed to read from EEPROM");
      fprintf( stderr, "Failed to read %d bytes @ %d from EEPROM\n", wordsize, i);
      result = -1;
      break;
    }
    if (i % rowsize == 0) {
      printf( "%06x:", offset + i);
    }
    switch (wordsize) {
    case 2:  printf( " %04x", ee_code); break;   
    case 4:  printf( " %08x", ee_code); break;   
    default: printf( " %02x", ee_code);
    }
    if (i % rowsize >= rowsize - wordsize || i >= bytecount - wordsize) {
      printf( "\n");
    }
  }
}

static void eeprom_flag_dump( const char* eeprom_path, unsigned int flag_nr)
{
  int ee_fd = open( eeprom_path, O_RDONLY);
  if (ee_fd < 0) {
    perror( "Failed to open EEPROM for reading");
    return;
  }
  unsigned int offset = eeprom_get_flag_offset( flag_nr);

  int result = lseek( ee_fd, offset, SEEK_SET);
  uint8_t ee_code;
  int count = read( ee_fd, &ee_code, 1);
  if (count != 1) {
    perror( "Failed to read from EEPROM");
    fprintf( stderr, "Failed to read flag %d at offset %u from EEPROM\n", flag_nr, offset);
    result = -1;
    return;
  }
  printf( "flag-%2d = $%02x (%u)\n", flag_nr, ee_code, ee_code);
}

static void eeprom_code_dump( const char* eeprom_path, unsigned int pru_nr)
{
  unsigned int offset = eeprom_get_pru_code_offset( pru_nr);
  eeprom_dump( eeprom_path, offset, 8192, 4);
}


static void usage( void)
{
  printf( "Usage:\n");
  printf( " -w -p<pru_nr> <filename>\n");
  printf( " -w -f<flag_nr> <flag_value>\n");
  printf( " -d -f<flag_nr>\n");
  printf( " -d -p<pru_nr>\n");
  printf( " with <flag_nr> in range 0..11\n");
  printf( " with <pru_nr> in range 0..1\n");
  exit( 1);
}

int main( int argc, char* argv[])
{
  int pru_nr = -1;
  int flag_nr = -1;
  enum { e_null, e_write, e_dump } mode = e_null;
  int c;

  // parse commandline
  while ((c = getopt( argc, argv, "wdp:f:")) != -1) {
    switch (c) {
    case 'w':	// write to EEPROM
      if (mode != e_null) {
        goto not_ok;
      }
      mode = e_write;
      break;
    case 'd':	// dump EEPROM
      if (mode != e_null) {
        goto not_ok;
      }
      mode = e_dump;
      break;
    case 'p':	// specify and validate pru nr
      pru_nr = atoi( optarg);
      if (pru_nr < 0 || pru_nr > 1) {
        goto not_ok;
      }
      break;
    case 'f':	// specify and validate flag_nr
      flag_nr = atoi( optarg);
      if (flag_nr < 0 || flag_nr >= member_size( struct eeprom, flags)) {
        goto not_ok;
      }
      break;
    default:
      goto not_ok;
    }
  }
  // execute command
  if (mode == e_write) {
    if (pru_nr >= 0 && flag_nr < 0) {
      // write file to EEPROM
      if (optind == argc - 1) {
        eeprom_write_pru_code( EEPROM_PATH, pru_nr, argv[ optind]);
      } else {
        goto not_ok;
      }
    } else if (flag_nr >= 0 && pru_nr < 0) {
      // set flag in EEPROM
      uint8_t flag_value = 0;
      if (optind == argc - 1) {
        // no error checking, keep this simple!
        flag_value = (uint8_t) atoi( argv[ optind]);
        eeprom_write_flag( EEPROM_PATH, flag_nr, flag_value);
      } else {
        goto not_ok;
      }
    } else {
      goto not_ok;
    }
  } else if (mode == e_dump) {
    if (pru_nr >= 0 && flag_nr < 0 && optind == argc) {
      eeprom_code_dump( EEPROM_PATH, pru_nr);
    } else if (flag_nr >= 0 && pru_nr < 0 && optind == argc) {
      eeprom_flag_dump( EEPROM_PATH, flag_nr);
    } else if (optind == argc) {
      for (flag_nr = 0 ; flag_nr < 12 ; ++flag_nr) {
        eeprom_flag_dump( EEPROM_PATH, flag_nr);
      }
    } else {
      goto not_ok;
    }
  } else {
    goto not_ok;
  }
  return 0;

not_ok:
  usage();
}

//----------------------------------------------------------------
#endif
//----------------------------------------------------------------
