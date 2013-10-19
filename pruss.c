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
#include <sched.h>

#define PRU_NR		1

#include "pruss.h"
#include "algo2cmds.h"
#include "beaglebone.h"
#include "debug.h"
#include "bebopr.h"
#include "sys_paths.h"

#define MAX_UIO_MAPS	5	// TODO: should be taken from the right UIO .h file!

#define debug 1
#define debug_level 2
#define UIO_DRIVER "pruss_uio"
#define PRUSS_DEVICE "pruss_evt0"

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


static struct uio_dev_info {
  char dev[ 30];
  int  fd;
  int  size;
  void* addr;
  int  offset;
  void* maddr;
} map_info[ 5];


static int map_device_l2( const char* path, struct uio_dev_info* info)
{
  char buffer[ NAME_MAX];
  char buffer2[ 15];
  int fd;

  if (debug && debug_level > 3) {
    printf( "    Determine mapping from '%s'\n", path);
  }
  if (!info) {
    return -1;
  }
  snprintf( buffer, sizeof( buffer), "%s/addr", path);
  fd = open( buffer, O_RDONLY );
  if (fd < 0) {
    perror( "    Cannot open UIO map 'addr' descriptor file");
    return -1;
  }
  read( fd, buffer2, sizeof( buffer2));
  info->addr = (void *)strtoul( buffer2, NULL, 0);
  close( fd);

  snprintf( buffer, sizeof( buffer), "%s/size", path);
  fd = open( buffer, O_RDONLY );
  if (fd < 0) {
    perror( "    Cannot open UIO map 'size' descriptor file");
    return -1;
  }
  read( fd, buffer2, sizeof( buffer2));
  info->size = (int)strtol( buffer2, NULL, 0);
  close( fd);

  snprintf( buffer, sizeof( buffer), "%s/offset", path);
  fd = open( buffer, O_RDONLY );
  if (fd < 0) {
    perror( "    Cannot open UIO map 'offset' descriptor file");
    return -1;
  }
  read( fd, buffer2, sizeof( buffer2));
  info->offset = (int)strtol( buffer2, NULL, 0);
  close( fd);

  if (debug && debug_level > 3) {
    printf( "    Map info found: addr=0x%08x, size=0x%08x, offset=0x%08x\n",
	    (unsigned int)info->addr, info->size, info->offset);
  }
  return 0;	// Success
}

int map_device( const char* uio_name)
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
    if (dir == NULL) {
      if (debug && debug_level > 2) {
        printf( "  opendir( '%s') failed, errno = %d\n", path, errno);
      }
      if (errno == ENOENT) {
	break;	// done
      } else {
	perror( "  Cannot open map directory");
	exit( EXIT_FAILURE);
      }
    } else {
      if (debug && debug_level > 2) {
        printf( "  opendir( '%s') returned %d\n", path, (int)dir);
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
      return -1;
    }
    // According to the UserSpace I/O Howto, mapping N is selected by specifying
    // an offset of (N * pagesize) for the mmap call. This seems to work!
    // This implies that the offset field cannot be used as normal!
    info->maddr = mmap( NULL, info->size, PROT_READ | PROT_WRITE, MAP_SHARED, info->fd, map_nr * getpagesize());
    if (info->maddr == MAP_FAILED) {
      perror( "  Cannot mmap device");
      return -1;
    }
    if (debug && debug_level > 2) {
      printf( "  Mapped '%s' map%d [0x%08x..0x%08x] onto VA 0x%08x\n",
	      info->dev, map_nr, (unsigned int)info->addr,
	      (unsigned int)(info->addr + info->size), (unsigned int)info->maddr);
    }
    close( info->fd);
    path[ strlen( path) - 1] = '1' + map_nr;
  }
  return 0;
}

//============================================================================

int locate_pruss_device_l4( const char* path, const char* device_name)
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
    exit( EXIT_FAILURE);	
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
    exit( EXIT_FAILURE);
  }
  for (de = readdir( dir) ; de ; de = readdir( dir)) {
    if (de < 0) {
      perror( "Problem reading directory");
      exit( EXIT_FAILURE);
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

  if (get_kernel_type() == e_kernel_3_8) {
    char path[ NAME_MAX];
    sys_path_finder( path, sizeof( path), "/sys/devices/ocp.*");
    snprintf( buffer, sizeof( buffer), "%s/4a300000.pruss/%s", path, uio_dev_name);
  }
  if (get_kernel_type() == e_kernel_3_2) {
    snprintf( buffer, sizeof( buffer), "/sys/bus/platform/devices/%s/%s", device_name, uio_dev_name);
  }

  dir = opendir( buffer);
  if (dir <= 0) {
    perror( "Platform device not found");
    exit( EXIT_FAILURE);
  }
  for (de = readdir( dir) ; de ; de = readdir( dir)) {
    if (de < 0) {
      perror( "Problem reading directory");
      exit( EXIT_FAILURE);
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


int locate_pruss_device( const char* driver_name, char* drv_name, int drv_name_len, char* uio_name, int uio_name_len)
{
  char buffer[ NAME_MAX];
  DIR* dir;
  int  found = 0;

  // for each driver instance of type 'driver_name', scan all subdirs of
  // /sys/bus/platform/devices/<driver_name>.<instance>/uio/
  snprintf( buffer, sizeof( buffer), "/sys/bus/platform/drivers/%s", driver_name);
  dir = opendir( buffer);
  if (dir <= 0) {
    printf( "module '%s' is not loaded, trying to load it...\n", driver_name);
    system( "/sbin/modprobe uio_pruss");
    // FIXME: find proper solution: mmap later on will fail if we don't delay here
    system( "sleep 1");
    dir = opendir( buffer);
    if (dir <= 0) {
      perror( "Platform driver not found");
      exit( EXIT_FAILURE);
    }
  }

  if (get_kernel_type() == e_kernel_3_8) {
    if (locate_pruss_device_l2( driver_name, uio_name, uio_name_len)) {
      if (drv_name) {
	strncpy( drv_name, driver_name, drv_name_len);
      }
      found = 1;
    }
  }

  if (get_kernel_type() == e_kernel_3_2) {
    struct dirent* de;
    for (de = readdir( dir) ; de ; de = readdir( dir)) {
      if (de < 0) {
	perror( "Problem reading directory");
	exit( EXIT_FAILURE);
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

#define VERBOSE 0

int pruss_load_code( const char* fname, unsigned int offset, unsigned int* start_addr, struct ucode_signature* signature)
{
  int count;
  unsigned int opcode;
  unsigned int address;
  unsigned int data;
  int skip_zeros;
  int error = 0;

  int fd = open( fname, O_RDONLY);
  if (fd < 0) {
    perror( "Cannot open STEPPER code file");
    return -1;
  }
  /* Position file read pointer to start of the code */
  if (offset) {
    if (lseek( fd, offset, SEEK_SET) < 0) {
      close( fd);
      perror( "Cannot lseek STEPPER code file");
      return -1;
    }
  }
  for (address = 0, skip_zeros = 1 ; address < 8192 ; address += 4) {
    count = read( fd, &opcode, sizeof( opcode));
    if (count < 4) {
      break;
    }
    if (skip_zeros && opcode != 0) {
      /* If caller wants to know the start address... */
      if (start_addr) {
	*start_addr = address / 4;
      }
      /* Do this only once */
      skip_zeros = 0;
      /* Check code signature */
      count = read( fd, signature, sizeof( *signature));
      if (count != sizeof( *signature)) {
        fprintf( stderr, "Short read on STEPPER code signature!\n");
        break;
      }
      if (signature->pruss_magic != PRUSS_MAGIC) {
        fprintf( stderr, "This is not valid PRUSS code, bailing out!\n");
        close( fd);
        return -1;
      }
      lseek( fd, offset + address + 4, SEEK_SET);
    }
    pruss_wr32( PRUSS_IRAM_OFFSET + address, opcode);
    if (VERBOSE) {
      printf( "IRAM%d+%d <= 0x%08x\n", PRU_NR, address, opcode);
    }
  }
  /* Reposition file read pointer to the start of the code */
  lseek( fd, offset, SEEK_SET);
  /*  Verify contents of instruction RAM with file */
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
      break;
    }
  }
  close( fd);
  return (error) ? -1 : 0;
}

int pruss_halt_pruss( void)
{
  /* Do not return until the PRU is halted! */
  int timeout = 25;
  uint32_t ctlreg = pruss_rd32( PRUSS_PRU_CTRL_CONTROL);
  while (ctlreg & PRUSS_PRU_CTRL_CONTROL_RUNSTATE) {
    /* Clear the enable bit to stop the PRU running */
    pruss_wr32( PRUSS_PRU_CTRL_CONTROL, ctlreg & ~PRUSS_PRU_CTRL_CONTROL_ENABLE);
    printf( ".");
    if (--timeout == 0) {
      /* Sometimes disable won't work... */
      printf( "cannot disable, soft reset ... ");
      pruss_wr32( PRUSS_PRU_CTRL_CONTROL,  ctlreg & ~PRUSS_PRU_CTRL_CONTROL_NRESET);
    } 
    ctlreg = pruss_rd32( PRUSS_PRU_CTRL_CONTROL);
  }
  return 0;
}

int pruss_init( const char* ucodename, unsigned int offset, struct ucode_signature* signature)
{
  char uio_name[ NAME_MAX];
  char drv_name[ NAME_MAX];
  int start_addr_arg = 0;

  // set option defaults
  unsigned int start_addr = 0;

  if (locate_pruss_device( UIO_DRIVER, drv_name, sizeof( drv_name), uio_name, sizeof( uio_name))) {
    if (debug_flags & DEBUG_PRUSS) {
      printf( "Located driver '%s' for the 'pruss' device '%s'.\n", drv_name, uio_name);
    }
    // Read PRUSS version register as simple check to verify proper mapping

    map_device( uio_name);

    // Note that PRUSS_BASE is 0 because it is remapped!
#define MM_PRUSS_REVID  (PRUSS_CFG_OFFSET + 0x00000000UL)
#define MM_PRUSS_SYSCFG (PRUSS_CFG_OFFSET + 0x00000004UL)
#define MM_PRUSS_GPCFG0 (PRUSS_CFG_OFFSET + 0x00000008UL)
#define MM_PRUSS_GPCFG1 (PRUSS_CFG_OFFSET + 0x0000000CUL)

    // Enable clock for the outgoing OCP_HP0&1 (L3_FAST) busses
    // This will only work if the PRUSS is operational (not reset)!
    if (pruss_rd32( MM_PRUSS_REVID) == 0x47000000UL) {
      if (debug_flags & DEBUG_PRUSS) {
        printf( "Valid PRUSS ID found.\n");
      }
      int status = pruss_rd32( MM_PRUSS_SYSCFG);
      if ((status & 0x10) != 0) {
        pruss_wr32( MM_PRUSS_SYSCFG, status & ~(1 << 4));
        printf( "PRUSS enabled OCP master ports.\n");
      }
    } else {
      fprintf( stderr, "PRUSS ID is not found.\n");
      return -1;
    }

    if (pruss_rd32( PRUSS_PRU_CTRL_CONTROL) & PRUSS_PRU_CTRL_CONTROL_RUNSTATE) {
      if (debug_flags & DEBUG_PRUSS) {
        printf( "Found running PRU%d, disable it...", PRU_NR);
      }
      pruss_halt_pruss();
      if (debug_flags & DEBUG_PRUSS) {
        printf( " done.\n");
      }
    } else {
      if (debug_flags & DEBUG_PRUSS) {
        printf( "Found halted/idle PRU%d\n", PRU_NR);
      }
    }
  } else {
    printf( "PRUSS driver not found, bailing out\n");
    return -1;
  }

  // Reset PRUSS counters
  if (debug_flags & DEBUG_PRUSS) {
    printf( "Clearing PRUSS counters, old: cycle = %u, stall = %u\n",
            pruss_rd32( PRUSS_PRU_CTRL_CYCLE), pruss_rd32( PRUSS_PRU_CTRL_STALL));
  }
  pruss_wr32( PRUSS_PRU_CTRL_CYCLE, 0);
  pruss_wr32( PRUSS_PRU_CTRL_STALL, 0);

  // Load PRUSS code
  if (debug_flags & DEBUG_PRUSS) {
    printf( "Loading STEPPER code from file '%s'\n", ucodename);
  }
  if (pruss_load_code( ucodename, offset, (start_addr_arg) ? NULL : &start_addr, signature) < 0) {
    return -1;
  }

  // Clear/init memory and registers
  if (debug_flags & DEBUG_PRUSS) {
    printf( "Clearing register space...\n");
  }
  for (int i = 0 ; i < 30 ; ++i) {
    pruss_wr32( PRUSS_DBG_OFFSET + 4 * i, 0);
  }
  if (debug_flags & DEBUG_PRUSS) {
    printf( "Initializing 8KB SRAM with deadbeef pattern...\n");
  }
  for (int i = 0 ; i < 2048 ; ++i) {
    pruss_wr32( PRUSS_RAM_OFFSET + 4 * i, 0xdeadbeef);
  }
  if (debug_flags & DEBUG_PRUSS) {
    printf( "Initializing SRAM buffer with fixed patterns...\n");
  }
  for (int i = 0 ; i < 8 ; ++i) {
    for (int j = 0 ; j < 4 ; ++j) {
      pruss_wr32( PRUSS_RAM_OFFSET + 256 + 4 * (4 * i + j), 0xcafe0000 + 256 * i + j);
    }
  }
  if (debug_flags & DEBUG_PRUSS) {
    printf( "Reset PRU%d and set program counter to %d\n", PRU_NR, start_addr);
  }
  /* clear bit 0 to reset the PRU, this is a self clearing (setting) bit! */
  pruss_wr32( PRUSS_PRU_CTRL_CONTROL, (start_addr << 16) | 0x00000000);	// pc + #softreset
  if (pruss_rd32( PRUSS_PRU_CTRL_STATUS) != start_addr) {
    fprintf( stderr, "Failed to set PRUSS code start address (PC)\n");
    return -1;
  }
  return 0;
}

int pruss_is_halted( void)
{
  return ((pruss_rd32( PRUSS_PRU_CTRL_CONTROL) & PRUSS_PRU_CTRL_CONTROL_RUNSTATE) == 0);
}

void pruss_wait_for_halt( void)
{
  do {
    // The STEPPER code may still be running, wait for HALT
  } while (!pruss_is_halted());
}

// return old 'enable' state
int pruss_stop_pruss( void)
{
  if (!pruss_is_halted()) {
    uint32_t pruss_ctrl = pruss_rd32( PRUSS_PRU_CTRL_CONTROL);
    pruss_ctrl &= ~ PRUSS_PRU_CTRL_CONTROL_ENABLE;  	// clear enable bit
    pruss_wr32( PRUSS_PRU_CTRL_CONTROL, pruss_ctrl);
    pruss_wait_for_halt();
    return 1;
  } else {
    return 0;
  }
}

void pruss_start_pruss( void)
{
  uint32_t pruss_ctrl;
  int retries = 1;
  pruss_ctrl = pruss_rd32( PRUSS_PRU_CTRL_CONTROL);
  do {
    if ((pruss_ctrl & PRUSS_PRU_CTRL_CONTROL_RUNSTATE) == 0) {
      // Keep reset PC, counter enable and #reset bits, assert pruss (run) enable
      pruss_ctrl &= (0xFFFF << 16) | PRUSS_PRU_CTRL_CONTROL_NRESET | PRUSS_PRU_CTRL_CONTROL_COUNTER_ENABLE;
      pruss_ctrl |= PRUSS_PRU_CTRL_CONTROL_ENABLE;
      pruss_wr32( PRUSS_PRU_CTRL_CONTROL, pruss_ctrl);
    }
    pruss_ctrl = pruss_rd32( PRUSS_PRU_CTRL_CONTROL);
    if ((pruss_ctrl & PRUSS_PRU_CTRL_CONTROL_RUNSTATE) == 0) {
      uint16_t pc = pruss_rd16( PRUSS_PRU_CTRL_STATUS);
      uint32_t opcode = pruss_rd32( PRUSS_IRAM_OFFSET + 4 * pc);
      if (opcode == 0x2a000000) { /* HALT */
        if (debug_flags & DEBUG_PRUSS) {
          printf( "PRUSS might have been started, but is now found executing a HALT instruction\n");
        }
      } else {
        fprintf( stderr, "PRUSS didn't start properly, opcode 0x%08x found at PC=0x%03x!\n", opcode, pc);
      }
    }
  } while (retries-- > 0 && (pruss_ctrl & PRUSS_PRU_CTRL_CONTROL_RUNSTATE) == 0);
}

void pruss_single_step_pruss( void)
{
  uint32_t pruss_ctrl = pruss_rd32( PRUSS_PRU_CTRL_CONTROL);
  if ((pruss_ctrl & PRUSS_PRU_CTRL_CONTROL_RUNSTATE) == 0) {
    uint16_t pc = pruss_rd16( PRUSS_PRU_CTRL_STATUS);
    uint32_t opcode = pruss_rd32( PRUSS_IRAM_OFFSET + 4 * pc);
    printf( "PRUSS found halted at PC=%06x, OPCODE=%08x\n", pc, opcode);
    pruss_ctrl |= PRUSS_PRU_CTRL_CONTROL_ENABLE;
    pruss_ctrl |= PRUSS_PRU_CTRL_CONTROL_NRESET;
    pruss_ctrl |= PRUSS_PRU_CTRL_CONTROL_SINGLE_STEP;
    pruss_wr32( PRUSS_PRU_CTRL_CONTROL, pruss_ctrl);		// STEP
    pruss_ctrl = pruss_rd32( PRUSS_PRU_CTRL_CONTROL);
    if ((pruss_ctrl & PRUSS_PRU_CTRL_CONTROL_RUNSTATE) == 0) {
      pruss_ctrl &= ~PRUSS_PRU_CTRL_CONTROL_SINGLE_STEP;
      pruss_ctrl &= ~PRUSS_PRU_CTRL_CONTROL_ENABLE;
      pruss_wr32( PRUSS_PRU_CTRL_CONTROL, pruss_ctrl);
    } else {
      fprintf( stderr, "Single step failure, PRUSS not halted after step.\n");
    }
  } else {
    fprintf( stderr, "Single step failure, PRUSS not halted at entry.\n");
  }
}

/*
 *  Resume stopped PRUSS
 */
void pruss_resume_pruss( void)
{
  if (pruss_is_halted()) {
    uint32_t pruss_ctrl = pruss_rd32( PRUSS_PRU_CTRL_CONTROL);
    uint16_t pc = pruss_rd16( PRUSS_PRU_CTRL_STATUS);
    uint32_t opcode = pruss_rd32( PRUSS_IRAM_OFFSET + 4 * pc);
    if (opcode == 0x2a000000) {
      printf( "PRUSS found halted with PC=%06x, OPCODE=%08x (HALT)\n", pc, opcode);
      pruss_wr32( PRUSS_IRAM_OFFSET + 4 * pc, 0x12e0e0e0);	// NOP
      pruss_ctrl |= PRUSS_PRU_CTRL_CONTROL_ENABLE;
      pruss_ctrl |= PRUSS_PRU_CTRL_CONTROL_NRESET;
      pruss_ctrl |= PRUSS_PRU_CTRL_CONTROL_SINGLE_STEP;
      pruss_wr32( PRUSS_PRU_CTRL_CONTROL, pruss_ctrl);		// STEP
      usleep( 100);
      pruss_wait_for_halt();
      /*
       * We've stepped over the inserted NOP, now restore the HALT
       */
      pruss_wr32( PRUSS_IRAM_OFFSET + 4 * pc, 0x2a000000);	// restore HALT
      uint16_t pc1 = pruss_rd16( PRUSS_PRU_CTRL_STATUS);
      if (pc1 == pc) {
          fprintf( stderr, "Single step failure 1\n");
	  return;
      }
    } else {
      printf( "PRUSS found halted with PC=%06x, OPCODE=%08x\n", pc, opcode);
    }
    /*
     *  Resume, keep reset PC, counter enable and #reset bits, assert pruss (run) enable
     */
    pruss_ctrl = pruss_rd32( PRUSS_PRU_CTRL_CONTROL);
    pruss_ctrl &= ~PRUSS_PRU_CTRL_CONTROL_SINGLE_STEP;
    pruss_ctrl |= PRUSS_PRU_CTRL_CONTROL_ENABLE;
    pruss_wr32( PRUSS_PRU_CTRL_CONTROL, pruss_ctrl);		// RUN
  } else {
    printf( "PRUSS resume called while running!\n");
  }
}

/*
 * Dump PRUSS state (if needed, halt PRUSS first).
 * Exit with PRUSS halted, return original running state.
 */
int pruss_dump_state( void)
{
  int i;
  int pruss_ena = 0;

  if (!pruss_is_halted()) {
    pruss_stop_pruss();
    pruss_ena = 1;
  }
  printf( "PRUSS %shalted, extracting debug info...\n", (pruss_ena) ? "is temporarily " : "was found ");
  for (i = 0 ; i < 32 ; ++i) {
    uint32_t reg = pruss_rd32( PRUSS_DBG_OFFSET + 4 * i);
    printf( "    R%-2d = 0x%08x (%10u)", i, reg, reg);
    if (i % 2 == 1) {
      printf( "\n");
    }
  }
  for (i = 24 ; i <= 28 ; i += 4) {
    uint32_t reg = pruss_rd32( PRUSS_DBG_OFFSET + 128 + 4 * i);
    printf( "    C%-2d = 0x%08x (%10u)", i, reg, reg);
    if (i % 2 == 1) {
      printf( "\n");
    }
  }
  if (i % 2 != 1) {
    printf( "\n");
  }
  return pruss_ena;
}
