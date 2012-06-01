#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <stdint.h>

#include <dirent.h>
#include <string.h> 
#include <errno.h>
#include <math.h>

#include "pruss.h"
#include "algo2cmds.h"
#include "beaglebone.h"

#define PRU_NR		1

#define TI_DRIVER	1

#define MAX_UIO_MAPS	5	// TODO: should be taken from the right UIO .h file!

// Generic struct for access to 'command' field for all commands.
typedef struct {
  unsigned int			: 24;
  unsigned int	value		:  4;
  unsigned int	axis		:  3;
  unsigned int                  :  1;
} CommandStruct;

// CMD_AXIS_SET_ORIGIN
typedef struct {
  unsigned int			: 24;
  unsigned int	command		:  4;
  unsigned int	axis		:  3;
  unsigned int			:  1;
  unsigned int	position	: 32;
} SetOriginStruct;

// CMD_AXIS_SET_ACCEL
typedef struct {
  unsigned int	accelCount	: 24;
  unsigned int	command		:  4;
  unsigned int	axis		:  3;
  unsigned int			:  1;
  unsigned int			: 32;
  unsigned int	stepCycle	: 32;
} SetAccelStruct;

// CMD_AXIS_ACCEL
typedef struct {
  unsigned int			: 24;
  unsigned int	command		:  4;
  unsigned int	axis		:  3;
  unsigned int			:  1;
  int		moveDelta	: 32;
  unsigned int	stepCycle	: 32;
} AccelStruct;

// CMD_AXIS_DECEL
typedef struct {
  unsigned int			: 24;
  unsigned int	command		:  4;
  unsigned int	axis		:  3;
  unsigned int			:  1;
  int		moveDelta	: 32;
  unsigned int	stepCycle	: 32;
} DecelStruct;

// CMD_AXIS_DWELL
typedef struct {
  unsigned int			: 24;
  unsigned int	command		:  4;
  unsigned int	axis		:  3;
  unsigned int			:  1;
  int		moveDelta	: 32;
  unsigned int	stepCycle	: 32;
} DwellStruct;

// CMD_AXIS_SET_PULSE
typedef struct {
  unsigned int	duration	: 24;
  unsigned int	command		:  4;
  unsigned int	axis		:  3;
} SetPulseStruct;

// CMD_SET_IDLE_TIMEOUT
typedef struct {
  unsigned int	timeout		:  8;
  unsigned int			: 16;
  unsigned int	command		:  4;
} SetIdleTimeoutStruct;

// CMD_SET_ENABLE
typedef struct {
  unsigned int	on		:  1;
  unsigned int			: 23;
  unsigned int	command		:  4;
} SetEnableStruct;

// CMD_AXIS_CONFIG_AXIS
typedef struct {
  unsigned int	reverse		:  1;
  unsigned int			: 23;
  unsigned int	command		:  4;
  unsigned int	axis		:  3;
  unsigned int			:  1;
  unsigned int	stepSizeT	: 16;
  unsigned int	stepSizeN	: 16;
  unsigned int	stepSize	: 32;
} ConfigAxisStruct;

/* TODO: can we mmap an array of this struct directly onto SRAM ? */
typedef union {
  uint32_t		gen[ 3];
  CommandStruct		command;
  SetOriginStruct	set_origin;
  SetAccelStruct	set_accel;
  AccelStruct		accel;
  DecelStruct		decel;
  DwellStruct		dwell;
  SetPulseStruct	set_pulse;
  SetIdleTimeoutStruct	timeout;
  SetEnableStruct	enable;
  ConfigAxisStruct	config;
} PruCommandUnion;


#if defined( TI_DRIVER)
#define debug 1
#define debug_level 2
#define UIO_DRIVER "pruss_uio"
#define PRUSS_DEVICE "pruss_evt0"
#else
#define debug 0
#define PRUSS_DEVICE "pruss"
#define UIO_DRIVER "uio_pdrv_genirq"
#endif


#define PRUSS_RAM0_OFFSET  0x00000000
#define PRUSS_RAM1_OFFSET  0x00002000
#define PRUSS_RAM2_OFFSET  0x00010000
#define PRUSS_CTL0_OFFSET  0x00022000
#define PRUSS_DBG0_OFFSET  0x00022400
#define PRUSS_CTL1_OFFSET  0x00024000
#define PRUSS_DBG1_OFFSET  0x00024400
#define PRUSS_CFG_OFFSET   0x00026000
#define PRUSS_ECAP0_OFFSET 0x00030000
#define PRUSS_IRAM0_OFFSET 0x00034000
#define PRUSS_IRAM1_OFFSET 0x00038000


#ifndef PRU_NR
#define PRU_NR	0
#endif

#if PRU_NR == 0

#define PRUSS_RAM_OFFSET   PRUSS_RAM0_OFFSET
#define PRUSS_CTL_OFFSET   PRUSS_CTL0_OFFSET
#define PRUSS_DBG_OFFSET   PRUSS_DBG0_OFFSET
#define PRUSS_IRAM_OFFSET  PRUSS_IRAM0_OFFSET

#elif PRU_NR == 1

#define PRUSS_RAM_OFFSET   PRUSS_RAM1_OFFSET
#define PRUSS_CTL_OFFSET   PRUSS_CTL1_OFFSET
#define PRUSS_DBG_OFFSET   PRUSS_DBG1_OFFSET
#define PRUSS_IRAM_OFFSET  PRUSS_IRAM1_OFFSET

#else

#error Illegal PRU_NR setting

#endif

static unsigned int start_addr;
static int start_addr_arg;
static int verbose;
static int daemonize;
static char ucodename[ 100];


static struct uio_dev_info {
  char dev[ 30];
  int  fd;
  int  size;
  void* addr;
  int  offset;
  void* maddr;
} map_info[ 5];


#define IX_IN	(PRUSS_RAM_OFFSET + 0xC0)
#define IX_OUT	(PRUSS_RAM_OFFSET + 0xC1)


static int map_device_l2( const char* path, struct uio_dev_info* info)
{
  char buffer[ NAME_MAX];
  char buffer2[ 15];
  int fd;

  if (debug && debug_level > 3) {
    printf( "    Determine mapping from '%s'\n", path);
  }
  if (!info) {
    return 0;
  }
  snprintf( buffer, sizeof( buffer), "%s/addr", path);
  fd = open( buffer, O_RDONLY );
  if (fd < 0) {
    perror( "    Cannot open UIO map 'addr' descriptor file.\n");
    return 0;
  }
  read( fd, buffer2, sizeof( buffer2));
  info->addr = (void *)strtoul( buffer2, NULL, 0);
  close( fd);

  snprintf( buffer, sizeof( buffer), "%s/size", path);
  fd = open( buffer, O_RDONLY );
  if (fd < 0) {
    perror( "    Cannot open UIO map 'size' descriptor file.\n");
    return 0;
  }
  read( fd, buffer2, sizeof( buffer2));
  info->size = (int)strtol( buffer2, NULL, 0);
  close( fd);

  snprintf( buffer, sizeof( buffer), "%s/offset", path);
  fd = open( buffer, O_RDONLY );
  if (fd < 0) {
    perror( "    Cannot open UIO map 'offset' descriptor file.\n");
    return 0;
  }
  read( fd, buffer2, sizeof( buffer2));
  info->offset = (int)strtol( buffer2, NULL, 0);
  close( fd);

  if (debug && debug_level > 3) {
    printf( "    Map info found: addr=0x%08x, size=0x%08x, offset=0x%08x\n",
	    (unsigned int)info->addr, info->size, info->offset);
  }
  return 1;	// Success
}

static int map_device( const char* uio_name)
{
  char path[ NAME_MAX];
  struct uio_dev_info* info;
  int map_nr;

  if (debug && debug_level > 1) {
    printf( "Map device for '%s' called\n", uio_name);
  }
  snprintf( path, sizeof( path), "/sys/class/uio/%s/maps/map0", uio_name);

  // Ordered lookup because scanning the directory might
  // provide the maps in the wrong order.
  for (map_nr = 0 ; map_nr < MAX_UIO_MAPS ; ++map_nr) {
    DIR* dir = opendir( path);
    if (debug && debug_level > 2) {
      printf( "  opendir( '%s') returned %d, errno = %d\n", path, (int)dir, errno);
    }
    if (dir == NULL) {
      if (errno == ENOENT) {
	break;	// done
      } else {
	perror( "  Cannot open map directory");
	exit( 1);
      }
    }
    closedir( dir);
    info = &map_info[ map_nr];

    // Read the map information from the files in this directory
    map_device_l2( path, info);

    // also store the corresponding devicename
    snprintf( info->dev, sizeof( info->dev), "/dev/%s", uio_name);
    // get a file descriptor to map from
    info->fd = open( info->dev, O_RDWR);
    if (info->fd < 0) {
      perror( "Cannot open device to map");
      return 0;
    }
    // According to the UserSpace I/O Howto, mapping N is selected by specifying
    // an offset of (N * pagesize) for the mmap call. This seems to work!
    // This implies that the offset field cannot be used as normal!
    info->maddr = mmap( NULL, info->size, PROT_READ | PROT_WRITE, MAP_SHARED, info->fd, map_nr * getpagesize());
    if (info->maddr == MAP_FAILED) {
      perror( "  Cannot mmap device");
      return 0;
    }
    if (debug && debug_level > 2) {
      printf( "  Mapped '%s' map%d [0x%08x..0x%08x] onto VA 0x%08x\n",
	      info->dev, map_nr, (unsigned int)info->addr,
	      (unsigned int)(info->addr + info->size), (unsigned int)info->maddr);
    }
    close( info->fd);
    path[ strlen( path) - 1] = '1' + map_nr;
  }
  return 1;
}

//============================================================================

static int locate_pruss_device_l4( const char* path, const char* device_name)
{
  char buffer[ NAME_MAX];
  int  found = 0;

  snprintf( buffer, sizeof( buffer), "%s/name", path);
  if (debug && debug_level > 2) {
    printf( "Open %s for reading...\n", buffer);
  }
  int fd = open( buffer, O_RDONLY);
  int count = read( fd, buffer, sizeof( buffer));
  if (count < 0) {
    perror( "Reading 'name' file failed");
    exit( 1);	
  }
  if (count >= 1 && buffer[ count - 1] == '\n') {
    --count;	// setup to remove newline
  }
  buffer[ count] = '\0';
  if (debug && debug_level > 2) {
    printf( "Read string '%s' from file.\n", buffer);
  }
  if (strcmp( device_name, buffer) == 0) {
    // Found the right uio device!
    found = 1;
  }
  return found;
}

static int locate_pruss_device_l3( const char* path, const char* device_name)
{
  char buffer[ NAME_MAX];
  DIR* dir;
  struct dirent* de;
  int  found = 0;

  snprintf( buffer, sizeof( buffer), "%s/%s", path, device_name);
  dir = opendir( buffer);
  if (dir <= 0) {
    perror( "Platform UIO device not found");
    exit( 1);
  }
  for (de = readdir( dir) ; de ; de = readdir( dir)) {
    if (de < 0) {
      perror( "Problem reading directory");
      exit( 1);
    }
    // find the 'name' entry with the proper contents
    if (strcmp( "name", de->d_name) == 0) {
      if (debug && debug_level > 3) {
      	printf( "found name file on path %s\n", buffer);
      }
      if (locate_pruss_device_l4( buffer, PRUSS_DEVICE)) {
	found = 1;
	break;
      }
    }
  }
  return found;
}

static int locate_pruss_device_l2( const char* device_name, char* uio_name, int uio_name_len)
{
  char buffer[ NAME_MAX];
  DIR* dir;
  struct dirent* de;
  int  found = 0;
  const char uio_dev_name[] = "uio";

  snprintf( buffer, sizeof( buffer), "/sys/bus/platform/devices/%s/%s", device_name, uio_dev_name);
  dir = opendir( buffer);
  if (dir <= 0) {
    perror( "Platform device not found");
    exit( 1);
  }
  for (de = readdir( dir) ; de ; de = readdir( dir)) {
    if (de < 0) {
      perror( "Problem reading directory");
      exit( 1);
    }
    // find entries that are instances of the uio device
    if (strncmp( uio_dev_name, de->d_name, strlen( uio_dev_name)) == 0) {
      if (debug && debug_level > 3) {
	printf( "found device instance '%s'\n", de->d_name);
      }
      if (locate_pruss_device_l3( buffer, de->d_name)) {
	if (uio_name) {
	  strncpy( uio_name, de->d_name, uio_name_len);
	}
	found = 1;
	break;
      }
    }
  }
  return found;
}

static int locate_pruss_device( const char* driver_name, char* drv_name, int drv_name_len, char* uio_name, int uio_name_len)
{
  char buffer[ NAME_MAX];
  DIR* dir;
  struct dirent* de;
  int  found = 0;
  // for each driver instance of type 'driver_name', scan all subdirs of
  // /sys/bus/platform/devices/<driver_name>.<instance>/uio/
  snprintf( buffer, sizeof( buffer), "/sys/bus/platform/drivers/%s", driver_name);
  dir = opendir( buffer);
  if (dir <= 0) {
    perror( "Platform driver not found");
    exit( 1);
  }
  for (de = readdir( dir) ; de ; de = readdir( dir)) {
    if (de < 0) {
      perror( "Problem reading directory");
      exit( 1);
    }
    if (debug && debug_level > 3) {
      printf( "found driver device file '%s'\n", de->d_name);
    }
    // find entries that are instances of the driver
    if (strncmp( driver_name, de->d_name, strlen( driver_name)) == 0) {
      if (debug && debug_level > 1) {
	printf( "found driver device instance '%s'\n", de->d_name);
      }
      if (locate_pruss_device_l2( de->d_name, uio_name, uio_name_len)) {
	if (drv_name) {
	  strncpy( drv_name, de->d_name, drv_name_len);
	}
	found = 1;
	break;
      }
    }
  }
  return found;
}

//============================================================================

uint32_t pruss_rd32( unsigned int addr)
{
  struct uio_dev_info* info = &map_info[ 0];

  void* p = info->maddr + info->offset + addr; 
  return *(uint32_t*)p;
}

uint16_t pruss_rd16( unsigned int addr)
{
  struct uio_dev_info* info = &map_info[ 0];

  void* p = info->maddr + info->offset + addr; 
  return *(uint16_t*)p;
}

uint8_t pruss_rd8( unsigned int addr)
{
  struct uio_dev_info* info = &map_info[ 0];

  void* p = info->maddr + info->offset + addr; 
  return *(uint8_t*)p;
}

void pruss_wr32( unsigned int addr, uint32_t data)
{
  struct uio_dev_info* info = &map_info[ 0];

  if ((addr & 3) != 0) {
    printf( "Warning, unaligned 32-bit memory access!\n");
  }
  void* p = info->maddr + info->offset + addr;
  *(uint32_t*)p = data;
}

void pruss_wr16( unsigned int addr, uint16_t data)
{
  struct uio_dev_info* info = &map_info[ 0];

  if ((addr & 1) != 0) {
    printf( "Warning, unaligned 16-bit memory access!\n");
  }
  void* p = info->maddr + info->offset + addr;
  *(uint16_t*)p = data;
}

void pruss_wr8( unsigned int addr, uint8_t data)
{
  struct uio_dev_info* info = &map_info[ 0];

  void* p = info->maddr + info->offset + addr;
  *(uint8_t*)p = data;
}


int pruss_load_code( const char* fname, unsigned int* start_addr)
{
  int count;
  int fd = open( fname, O_RDONLY);
  unsigned int opcode;
  unsigned int address;
  unsigned int data;
  int skip_zeros;
  int error = 0;

  if (fd < 0) {
    perror( "Cannot open microcode file.\n");
    return 0;
  }
  for (address = 0, skip_zeros = 1 ; address < 8192 ; address += 4) {
    count = read( fd, &opcode, sizeof( opcode));
    if (count < 4) {
      break;
    }
    pruss_wr32( PRUSS_IRAM_OFFSET + address, opcode);
    if (verbose) {
      printf( "IRAM%d+%d <= 0x%08x\n", PRU_NR, address, opcode);
    }
    if (skip_zeros && opcode) {
      if (start_addr) {
	*start_addr = address / 4;
      }
      skip_zeros = 0;
    }
  }
  lseek( fd, 0, SEEK_SET);
  for (address = 0 ; address < 8192 ; address += 4) {
    count = read( fd, &opcode, sizeof( opcode));
    if (count < 4) {
      break;
    }
    data = pruss_rd32( PRUSS_IRAM_OFFSET + address);
    if (opcode != data) {
      printf( "IRAM%d Verify error at address 0x%06x, read 0x%08x, should be 0x%08x.\n",
	      PRU_NR, address, data, opcode);
      error = 1;
    }
  }
  close( fd);
  return (error) ? 0 : 1;
}

static void eCapInit( void)
{
#define O_TSCTR		0
#define O_CTRPHS	4
#define O_CAP1		8	/* cycle */
#define O_CAP2		12	/* compare / duty cycle */
#define O_CAP3		16
#define O_CAP4		20
#define O_ECCTL1	40
#define O_ECCTL2	42
#define O_ECEINT	44
#define O_ECFLG		46
#define O_ECCLR		48
#define O_ECFRC		50
#define O_REVID		92

  uint32_t eCapId = pruss_rd32( PRUSS_ECAP0_OFFSET + O_REVID);
  uint32_t value;

  if (eCapId != 0x44d22100) {
    printf( "*** ERROR: invalid eCapId! Found 0x%08x, should be 0x44d22100\n", eCapId);
    exit( 1);
  }
  // Stop counter
  pruss_wr16( PRUSS_ECAP0_OFFSET + O_ECCTL2, (1 << 9) | (1 << 7) | (0 << 4));
  // Reset counter and phase registers
  pruss_wr32( PRUSS_ECAP0_OFFSET + O_TSCTR, 0);
  pruss_wr32( PRUSS_ECAP0_OFFSET + O_CTRPHS, 0);
  // Setup for large (detection) cycle
  pruss_wr32( PRUSS_ECAP0_OFFSET + O_CAP1, 20000000);	// PWM period 0.1 s
  pruss_wr32( PRUSS_ECAP0_OFFSET + O_CAP2, 10000000);	// PWM on time, don't care
  pruss_wr32( PRUSS_ECAP0_OFFSET + O_CAP1, 200);		// set PWM period to 1 us 
  pruss_wr32( PRUSS_ECAP0_OFFSET + O_CAP2, 100);		// PWM on time, don't care
  // Clear pending interrupt
  pruss_wr16( PRUSS_ECAP0_OFFSET + O_ECCLR, (1 << 6));
  if (pruss_rd16( PRUSS_ECAP0_OFFSET + O_ECFLG) & (1 << 6)) {
    printf( "*** WARNING: could not clear eCAP0 interrupt\n");
    exit( 1);
  }
  // Start counter in APWM mode
  pruss_wr16( PRUSS_ECAP0_OFFSET + O_ECCTL2, (1 << 9) | (1 << 7) | (1 << 4));
  do {
    value = pruss_rd16( PRUSS_ECAP0_OFFSET + O_ECFLG);
  } while ((value & (1 << 6)) == 0);
  pruss_wr16( PRUSS_ECAP0_OFFSET + O_ECCLR, (1 << 6));
  printf( "eCap0 initialized and found operational\n");
}

int pruss_halt_pruss( void)
{
  int timeout = 25;
  while (pruss_rd32( PRUSS_CTL_OFFSET) & (1 << 15)) {
    pruss_wr32( PRUSS_CTL_OFFSET + 0, 0x00000001);	// disable
    printf( ".");
    if (--timeout == 0) {
      // Sometimes disable won't work...
      printf( "cannot disable, soft reset ... ");
      pruss_wr32( PRUSS_CTL_OFFSET + 0, 0x00000000);	// reset
    }
  }
  return 0;
}

int pruss_init( void)
{
  char uio_name[ NAME_MAX];
  char drv_name[ NAME_MAX];
  //  PruCommandUnion pruCmd;
  int ix_in, ix_out;
  unsigned int status;

  //#undef MM_PRUSS_BASE
  //#undef MM_PRUSS_CFG
#undef MM_PRUSS_SYSCFG
#undef MM_PRUSS_REVID

  // set option defaults
  start_addr = 0;
  start_addr_arg = 0;
  verbose = 0;
  daemonize = 1;
  strcpy( ucodename, "stepper.bin");

  //  parse_arguments( argc, argv);

  if (locate_pruss_device( UIO_DRIVER, drv_name, sizeof( drv_name), uio_name, sizeof( uio_name))) {
    printf( "Located driver '%s' for the 'pruss' device '%s'.\n", drv_name, uio_name);
    // Read PRUSS version register as simple check to verify proper mapping

    map_device( uio_name);

#if defined( TI_DRIVER)
    // Note that PRUSS_BASE is 0 because it is remapped!
    //#define MM_PRUSS_BASE   (0UL)
    //#define MM_PRUSS_CFG    (MM_PRUSS_BASE + 0x00026000UL)
#define MM_PRUSS_REVID  (PRUSS_CFG_OFFSET + 0x00000000UL)
#define MM_PRUSS_SYSCFG (PRUSS_CFG_OFFSET + 0x00000004UL)
#define MM_PRUSS_GPCFG0 (PRUSS_CFG_OFFSET + 0x00000008UL)
#define MM_PRUSS_GPCFG1 (PRUSS_CFG_OFFSET + 0x0000000CUL)

    // Enable clock for the outgoing OCP_HP0&1 (L3_FAST) busses
    // This will only work if the PRUSS is operational (not reset)!
    if (pruss_rd32( MM_PRUSS_REVID) == 0x47000000UL) {
      printf( "Valid PRUSS ID found.\n");
      status = pruss_rd32( MM_PRUSS_SYSCFG);
      if ((status & 0x10) != 0) {
	pruss_wr32( MM_PRUSS_SYSCFG, status & ~(1 << 4));
	printf( "PRUSS enabled OCP master ports.\n");
      }
    } else {
      printf( "PRUSS ID is not found.\n");
      return 1;
    }
#endif

    if (pruss_rd32( PRUSS_CTL_OFFSET) & (1 << 15)) {
      printf( "Found running PRU%d, disable ..", PRU_NR);
#if 0
      int timeout = 25;
      while (pruss_rd32( PRUSS_CTL_OFFSET) & (1 << 15)) {
	pruss_wr32( PRUSS_CTL_OFFSET + 0, 0x00000001);	// disable
	printf( ".");
	if (--timeout == 0) {
	  // Sometimes disable won't work...
	  printf( "cannot disable, soft reset ... ");
	  pruss_wr32( PRUSS_CTL_OFFSET + 0, 0x00000000);	// reset
	}
      }
#else
      pruss_halt_pruss();
#endif
      if (pruss_rd32( PRUSS_CTL_OFFSET) & (1 << 15)) {
	printf( "Strange....\n");
      }
      printf( " done.\n");
    } else {
      printf( "Found halted/idle PRUSS\n");
    }
  } else {
    printf( "PRUSS driver not found, bailing out\n");
    return 1;
  }

  eCapInit();

  // Reset PRUSS counters
  printf( "Clearing PRUSS counters, old: cycle = %u, stall = %u\n",
	  pruss_rd32( PRUSS_CTL_OFFSET + 12), pruss_rd32( PRUSS_CTL_OFFSET + 16));
  pruss_wr32( PRUSS_CTL_OFFSET + 12, 0);
  pruss_wr32( PRUSS_CTL_OFFSET + 16, 0);
  //  printf( "PRUSS counters: cycle = %d, stall = %d\n",
  //	  pruss_rd32( PRUSS_CTL_OFFSET + 12), pruss_rd32( PRUSS_CTL_OFFSET + 16));

  printf( "Loading microcode from file '%s'\n", ucodename);
  printf( "------- LOAD -------\n");
  if (!pruss_load_code( ucodename, (start_addr_arg) ? NULL : &start_addr)) {
    return 1;
  }
  {
    int i;
    printf( "Clearing register space...\n");
    for (i = 0 ; i < 30 ; ++i) {
      pruss_wr32( PRUSS_DBG_OFFSET + 4 * i, 0);
    }
  }
  {
    int i;
    printf( "Initializing 8KB SRAM with deadbeef pattern...\n");
    for (i = 0 ; i < 2048 ; ++i) {
      pruss_wr32( PRUSS_RAM_OFFSET + 4 * i, 0xdeadbeef);
    }
  }
  ix_out = ix_in = 0;
  printf( "Setting SRAM ix_in to %d and ix_out to %d...\n", ix_in, ix_out);
  pruss_wr8( IX_IN, ix_in);		// in
  pruss_wr8( IX_OUT, ix_out);		// out
  pruss_wr16( IX_OUT + 1, 0xdeaf);	// filler
  {
    int i, j;
    printf( "Initializing SRAM buffer with fixed patterns...\n");
    for (i = 0 ; i < 8 ; ++i) {
      for (j = 0 ; j < 4 ; ++j) {
	pruss_wr32( PRUSS_RAM_OFFSET + 256 + 4 * (4 * i + j), 0xcafe0000 + 256 * i + j);
      }
    }
  }
  printf( "------- START -------\n");
  printf( "Reset: ");
  pruss_wr32( PRUSS_CTL_OFFSET + 0, (start_addr << 16) | 0x00000000);	// pc + #softreset
  if (pruss_rd32( PRUSS_CTL_OFFSET + 4) != start_addr) {
    printf( "Failed set set PRUSS code start address (PC)\n");
    exit( EXIT_FAILURE);
  }
  //  printf( "Control: "); READ_CTL0( 0);
  pruss_wr32( PRUSS_CTL_OFFSET, 0x0000000B);		// pc:=0 + enable + counter_enable
  if ((pruss_rd32( PRUSS_CTL_OFFSET + 0) & (1 <<15)) == 0) {
    printf( "Failed to start PRUSS code\n");
    //    exit( EXIT_FAILURE);
  }
  printf( "Pruss setting axis origins\n");
  for (int i = 1 ; i <= 4 ; ++i) {
    pruss_queue_set_origin( i);
  }
  return 0;
}

void pruss_wait_for_halt( void)
{
  do {
    // The microcode is running, wait for HALT
  } while (pruss_rd32( PRUSS_CTL_OFFSET + 0) & (1 << 15));
}

// return old 'enable' state
int pruss_stop_pruss( void)
{
  uint32_t pruss_ctrl = pruss_rd32( PRUSS_CTL_OFFSET + 0);
  if (pruss_ctrl & (1 << 15)) {
    pruss_ctrl &= ~ (1 << 1);	// clear enable bit
    pruss_wr32( PRUSS_CTL_OFFSET + 0, pruss_ctrl);
    pruss_wait_for_halt();
    return 1;
  } else {
    return 0;
  }
}

void pruss_start_pruss( void)
{
  uint32_t pruss_ctrl = pruss_rd32( PRUSS_CTL_OFFSET + 0);
  if ((pruss_ctrl & (1 << 15)) == 0) {
    pruss_ctrl |=  (1 << 1);	// set enable bit
    pruss_wr32( PRUSS_CTL_OFFSET + 0, pruss_ctrl);
  }
  do {
  } while ((pruss_rd32( PRUSS_CTL_OFFSET + 0) & (1 << 15)) == 0);
}

int pruss_dump_state( void)
{
  int i;
  int pc, sp, ret;
  //    uint32_t dataOffset = pruss_rd32( PRUSS_DBG_OFFSET + 128 + 4 * 24);

  int pruss_ena = pruss_stop_pruss();

  printf( "PRUSS was halted, extracting debug info...\n");
  for (i = 0 ; i < 32 ; ++i) {
    uint32_t reg = pruss_rd32( PRUSS_DBG_OFFSET + 4 * i);
    printf( "    R%-2d = 0x%08x (%10u)", i, reg, reg);
    if (i % 2 == 1) {
      printf( "\n");
    }
  }
  for (i = 24 ; i <= 25 ; ++i) {
    uint32_t reg = pruss_rd32( PRUSS_DBG_OFFSET + 128 + 4 * i);
    printf( "    C%-2d = 0x%08x (%10u)", i, reg, reg);
    if (i % 2 == 1) {
      printf( "\n");
    }
  }
  ret = pruss_rd16( PRUSS_DBG_OFFSET + 4 * 2);		// R2.w0
  sp  = pruss_rd16( PRUSS_DBG_OFFSET + 4 * 2 + 2);	// R2.w2
  pc  = pruss_rd16( PRUSS_CTL_OFFSET + 4);
  printf( "PC == 0x%03x (%d), SP == 0x%03x (%d), RET == 0x%04x (%4d)\n", pc, pc, sp, sp, ret, ret);
  for ( sp = (sp & 0xFFF0), i = 0 ; ; ++i) {
    ret = pruss_rd16( PRUSS_RAM_OFFSET + sp);
    if (ret == 0xdead || ret == 0xbeef) {
      break;
    }
    printf( "Stack[ %3d] @ SRAM[ 0x%03x] == 0x%04x (%4d)\n", i, sp, ret, ret);
    sp += 2;
  }

#define	FullADSize		(12 * 4)

#if 0
  /* Dump memory */
  for (i = 0 ; i < 256 ; ++i) {
    uint32_t a = PRUSS_RAM_OFFSET + i;
    if (i % 16 == 0) {
      printf( "0x%08x -", a);
    }
    printf( " %02x", pruss_rd8( a));
    if (i % 16 == 15) {
      printf( "\n");
    }
  }
#endif
  /* Dump per axis data */
  {
    uint32_t base[ 4];
    uint32_t data[ 4];
    uint16_t axes_config = pruss_rd16( PRUSS_DBG_OFFSET + 4 * 11 + 2);

#define DUMP_LINE( field, size, offset, format)     			          \
    do {                                                                          \
      int i;                                                                      \
      for (i = 0 ; i < 4 ; ++i) {				                  \
        data[ i] = pruss_rd ## size( PRUSS_RAM_OFFSET + i * FullADSize + offset); \
      }									          \
      printf( "%20s    " format "    " format "    " format "    " format "\n",   \
	       field, data[ 0], data[ 1], data[ 2], data[ 3]);			  \
    } while (0)

    printf( "%20s    %14s    %14s    %14s    %14s\n", "variable", "X-axis", "Y-axis", "Z-axis", "E-axis");
    for (i = 0 ; i < 4 ; ++i) {
      base[ i] = PRUSS_RAM_OFFSET + i * FullADSize;
    }
    printf( "%20s    %14x    %14x    %14x    %14x\n", "Base Address", base[ 0], base[ 1], base[ 2], base[ 3]);

    DUMP_LINE( "cycleTimer",       32,  0, "%14u");
    DUMP_LINE( "stepPulseTime",    32,  4, "%14u");
    DUMP_LINE( "minStepCycleTime", 32,  8, "%14u");
    DUMP_LINE( "stepCycleTime",    32, 12, "%14u");
    DUMP_LINE( "stateInfo",         8, 16, "%14x");
    DUMP_LINE( "stepPinBitNo",      8, 17, "%14u");
    // following code uses stepPinBitNo from data[] !
    for (i = 0 ; i < 4 ; ++i) {
      data[ i] = !!(axes_config & (1 << data[ i]));
    }
    printf( "%20s    %14x    %14x    %14x    %14x\n", "Axis reversal", data[ 0], data[ 1], data[ 2], data[ 3]);

    DUMP_LINE( "virtPosT",         16, 18, "%14u");
    DUMP_LINE( "virtPos",          32, 20, "%14x");

    DUMP_LINE( "stepSize",         32, 24, "%14u");
    DUMP_LINE( "stepSizeT",        16, 28, "%14u");
    DUMP_LINE( "stepSizeN",        16, 30, "%14u");

    DUMP_LINE( "requestedPos",     32, 32, "%14x");
    DUMP_LINE( "moveDelta",        32, 36, "%14u");
    DUMP_LINE( "accelCount",       32, 40, "%14u");
    DUMP_LINE( "dividend",         32, 44, "%14u");
  }

  printf( "Number of PRUSS cycles = %d, stall count = %d\n",
	  pruss_rd32( PRUSS_CTL_OFFSET + 12), pruss_rd32( PRUSS_CTL_OFFSET + 16));
  if (pruss_ena) {
    pruss_start_pruss();
  }
  return 0;
}


int pruss_is_halted( void)
{
  return ((pruss_rd32( PRUSS_CTL_OFFSET + 0) & (1 <<15)) == 0);
}

int pruss_get_nr_of_free_buffers( void)
{
  int ix_in  = pruss_rd8( IX_IN);
  int ix_out = pruss_rd8( IX_OUT);

  return (8 + ix_out - ix_in - 1) % 8;
}

int pruss_queue_full( void)
{
  return (pruss_get_nr_of_free_buffers() == 0);
}

int pruss_wait_for_queue_space( void)
{
  while (pruss_queue_full()) {
    if (pruss_is_halted()) {
      return -1;
    }
    // TODO: sleep until PRUSS interrupt ?
  }
  return 0;
}

// Write command structure to buffer[ ix_in] on PRUSS (must be free)
int pruss_write_command_struct( int ix_in, PruCommandUnion* data)
{
  int i;
  // TODO: rewrite to generic code: sizeof( PruCommandUnion)...
  for (i = 0 ; i < NR_ITEMS( data->gen) ; ++i) {
    uint32_t u = data->gen[ i];
    uint32_t a = ix_in * 16 + 4 * i;
    pruss_wr32( PRUSS_RAM_OFFSET + 256 + a, u);
    //    printf( "pruss_write_command_struct: wrote 0x%08x (%d) to offset %d\n", u, u, a);
  }
  ix_in = (ix_in + 1) % 8;
  pruss_wr8( IX_IN, ix_in);
  return ix_in;
}

// Write command structure to PRUSS, wait for free buffer is nescessary
int pruss_command( PruCommandUnion* cmd)
{
  int ix_in = pruss_rd8( IX_IN);
  //  int ix_out = pruss_rd8( IX_OUT);
  if (pruss_wait_for_queue_space() < 0) {
    printf( "pruss_command, bailing out because of failing pruss_wait_for_queue_space()\n");
    return -1;
  }
  //  printf( "pruss_command - write to SRAM buffer at index %d, out index is %d.\n", ix_in, ix_out);
  (void) pruss_write_command_struct( ix_in, cmd);
  //  ix_in = writeCommandStruct( ix_in, cmd);
  //  ix_out = pruss_rd8( PRUSS_RAM_OFFSET + 129);
  return 0;
}


static const double step_size = 6.250E-6;	// [mm] derived from 160 steps/mm
static const double max_step_rate = 62500;	// [Hz] derived from 16us interval
static const double fclk = 200000000;
//static const double v_max = step_size * max_step_rate;
static const double v_max = 6.250E-6 * 62500;


#define VIRT_POS_MID_SCALE	0x80000000

int pruss_queue_set_origin( int axis)
{
  PruCommandUnion pruCmd = {
    .set_origin.command		= CMD_AXIS_SET_ORIGIN,
    .set_origin.axis		= axis,
    .set_origin.position 	= VIRT_POS_MID_SCALE
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_set_pulse_length( int axis, uint16_t length)
{
  PruCommandUnion pruCmd = {
    .set_pulse.command		= CMD_AXIS_SET_PULSE_LENGTH,
    .set_pulse.axis		= axis,
    .set_pulse.duration 	= length
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_set_accel( int axis, uint32_t c0)
{
  PruCommandUnion pruCmd = {
    .set_accel.command		= CMD_AXIS_SET_ACCEL,
    .set_accel.axis		= axis,
    .set_accel.accelCount 	= 0,
    .set_accel.stepCycle 	= c0
  };
#if 0
  if (c0 >= (1 << 24)) {
    fprintf( stderr, "*** WARNING: clipping c0 because it's larger than 24-bits, speed may be too high!\n");
    pruCmd.set_accel.stepCycle = (1 << 24) - 1;
  }
#endif
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_accel_more( int axis, uint32_t cmin, int32_t delta)
{
  PruCommandUnion pruCmd = {
    .accel.command		= CMD_AXIS_RAMP_UP,
    .accel.axis			= axis,
    .accel.moveDelta		= delta,
    .accel.stepCycle		= cmin
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_accel( int axis, uint32_t c0, uint32_t cmin, int32_t delta)
{
  if (pruss_queue_set_accel( axis, c0) < 0) {
    return -1;
  }
  return pruss_queue_accel_more( axis, cmin, delta);
}

int pruss_queue_dwell( int axis, uint32_t cmin, int32_t delta)
{
  PruCommandUnion pruCmd = {
    .dwell.command		= CMD_AXIS_DWELL,
    .dwell.axis			= axis,
    .dwell.moveDelta		= delta,
    .dwell.stepCycle		= cmin
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_decel( int axis, int32_t delta)
{
  PruCommandUnion pruCmd = {
    .decel.command		= CMD_AXIS_RAMP_DOWN,
    .decel.axis			= axis,
    .decel.moveDelta		= delta
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_execute( void)
{
  if (pruss_is_halted()) {
    fprintf( stderr, "FATAL: PRUSS found halted when queueing execute command\n");
    pruss_dump_state();
    exit( 1);
    return -1;
  }
  //  fprintf( stderr, "pruss_queue_execute(): free buffers = %d\n",
  //	   pruss_get_nr_of_free_buffers());
  PruCommandUnion pruCmd = {
    .command.value		= CMD_AXES_EXECUTE,
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_config_axis( int axis, uint32_t ssi, uint16_t sst, uint16_t ssn, int reverse)
{
  PruCommandUnion pruCmd = {
    .config.command		= CMD_AXIS_CONFIG_AXIS,
    .config.axis		= axis,
    .config.reverse		= (reverse) ? 1 : 0,
    .config.stepSize		= ssi,
    .config.stepSizeT		= sst,
    .config.stepSizeN		= ssn,
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

/*
 * If the idle timeout is not set to 0, the motors will be enabled
 * and disabled automatically. The first step pulse asserts the
 * enable outputs, (period/10) seconds after the last tep pulse,
 * the enables are negated again.
 */
int pruss_queue_set_idle_timeout( uint8_t period)
{
  PruCommandUnion pruCmd = {
    .timeout.command		= CMD_SET_IDLE_TIMEOUT,
    .timeout.timeout		= period,
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

/*
 * Manually enable or disable the stepperdriver enable signals.
 * Necessary if the automatic idle timeout is disabled.
 */
int pruss_queue_set_enable( int on)
{
  PruCommandUnion pruCmd = {
    .enable.command		= CMD_SET_ENABLE,
    .enable.on			= !!on,
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

void pruss_dump_position( int axis)
{
  const char axes[] = { '?', 'X', 'Y', 'Z', 'E' };

  //  if (axis >= 1 && axis <= 4) {
  printf( "\n");
  for (axis = 1 ; axis <= 4 ; ++axis) {
    uint32_t base = PRUSS_RAM_OFFSET + (axis - 1) * FullADSize;
    int32_t virtPosI = pruss_rd32( base + 20) - VIRT_POS_MID_SCALE;
    uint16_t virtPosT = pruss_rd16( base + 18);
    uint16_t stepSizeN = pruss_rd16( base + 30);
    uint16_t stepSizeT = pruss_rd16( base + 28);
    if (stepSizeT == 0) {
      printf( "  %c-virtPos = %d", axes[ axis], virtPosI);
    } else {
      printf( "  %c-virtPos = %d+%u/%u", axes[ axis], virtPosI, virtPosT, stepSizeN);
    }
  }
  printf( "\n");
}

