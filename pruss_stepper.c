
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <dirent.h>
#include <string.h> 
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>

#include "pruss_stepper.h"
#define PRU_NR		1
#include "pruss.h"
#include "beaglebone.h"
#include "debug.h"
#include "bebopr.h"
#include "timestamp.h"
#include "eeprom.h"


// Generic struct for access to 'command' field for all commands.
typedef struct {
  unsigned int			: 24;
  unsigned int	value		:  5;
  unsigned int	axis		:  3;
} CommandStruct;

// CMD_AXIS_SET_ORIGIN
typedef struct {
  unsigned int			: 24;
  unsigned int	command		:  5;
  unsigned int	axis		:  3;
  unsigned int	position	: 32;
} SetOriginStruct;

// CMD_AXIS_MOVE
typedef struct {
  unsigned int	n0		: 24;
  unsigned int	command		:  5;
  unsigned int	axis		:  3;
  int		position	: 32;
  unsigned int	c0		: 32;
  unsigned int	cn		: 32;
} MoveStruct;

// CMD_AXIS_SET_PULSE_LENGTH
typedef struct {
  unsigned int	duration	: 24;
  unsigned int	command		:  5;
  unsigned int	axis		:  3;
} SetPulseStruct;

// CMD_SET_IDLE_TIMEOUT
typedef struct {
  unsigned int	timeout		:  8;
  unsigned int			: 16;
  unsigned int	command		:  5;
} SetIdleTimeoutStruct;

// CMD_SET_ENABLE
typedef struct {
  unsigned int	mode		:  8;
  unsigned int			: 16;
  unsigned int	command		:  5;
} SetEnableStruct;

// CMD_AXIS_CONFIG_AXIS
typedef struct {
  unsigned int	reverse		:  1;
  unsigned int			: 23;
  unsigned int	command		:  5;
  unsigned int	axis		:  3;
  unsigned int			: 32;
  unsigned int	stepSize	: 32;
  unsigned int	reciprStepSize	: 32;
} ConfigAxisStruct;

// CMD_AXIS_CONFIG_LIMSW
typedef struct {
  unsigned int			: 24;
  unsigned int	command		:  5;
  unsigned int	axis		:  3;
  unsigned int  min_gpio	:  8;
  unsigned int  min_invert	:  8;
  unsigned int  max_gpio	:  8;
  unsigned int  max_invert	:  8;
} ConfigLimswStruct;

// CMD_AXIS_ADJUST_ORIGIN
typedef struct {
  unsigned int			: 24;
  unsigned int	command		:  5;
  unsigned int	axis		:  3;
  unsigned int	position	: 32;
} AdjustOriginStruct;

/* TODO: can we mmap an array of this struct directly onto SRAM ? */
typedef union {
  uint32_t		gen[ 4];
  CommandStruct		command;
  SetOriginStruct	set_origin;
  MoveStruct		move;
  SetPulseStruct	set_pulse;
  SetIdleTimeoutStruct	timeout;
  SetEnableStruct	enable;
  ConfigAxisStruct	config;
  ConfigLimswStruct	limsw;
  AdjustOriginStruct	adjust_origin;
} PruCommandUnion;


#define IX_IN		(PRUSS_RAM_OFFSET + 0xC0)
#define IX_OUT		(PRUSS_RAM_OFFSET + 0xC1)
#define BUSY_FLAG	(PRUSS_RAM_OFFSET + 0xC4)

static int pruss_ecap_init( void)
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
    exit( EXIT_FAILURE);
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
    exit( EXIT_FAILURE);
  }
  // Start counter in APWM mode
  pruss_wr16( PRUSS_ECAP0_OFFSET + O_ECCTL2, (1 << 9) | (1 << 7) | (1 << 4));
  do {
    value = pruss_rd16( PRUSS_ECAP0_OFFSET + O_ECFLG);
  } while ((value & (1 << 6)) == 0);
  pruss_wr16( PRUSS_ECAP0_OFFSET + O_ECCLR, (1 << 6));
  if (debug_flags & DEBUG_PRUSS) {
    printf( "eCap0 initialized and found operational\n");
  }
  return 0;
}

/*
 * Extract code signature from specified file, return 0 on success, errorcode otherwise.
 */
static int get_code_signature( const char* fname, int offset, struct ucode_signature* signature)
{
  int error = 0;
  int fd = open( fname, O_RDONLY);
  if (fd < 0) {
    error = 1;		// failed to open file
  } else {
    /* Position file read pointer to start of the code */
    if (offset) {
      if (lseek( fd, offset, SEEK_SET) < 0) {
        error = 5;	// code start seek failed
      }
    }
    /* stay compatible with old loader format: allow for leading zero's in file */
    unsigned int opcode;
    if (error == 0) {
      do {
        if (read( fd, &opcode, sizeof( opcode)) < sizeof( opcode)) {
          error = 2;	// failed to read non-zero opcode
          break;
        }
      } while (opcode == 0);
    }
    /* extract the code signature from the file */
    if (error == 0) {
      if (read( fd, signature, sizeof( *signature)) < sizeof( *signature)) {
        error = 3;	// failed to read proper signature
      } else if (signature->pruss_magic != PRUSS_MAGIC) {
        error = 4;	// not a valid code file
      }
    }
  }
  if (fd >= 0) {
    close( fd);
  }
  return error;		// 0 == valid signature
}

static int pruss_command( PruCommandUnion* cmd);

int pruss_stepper_init( void)
{
  // Check for a valid pruss code update file
  struct ucode_signature fs_signature;
  const char* fs_fname = UCODE_FILE;
  const unsigned int fs_offset = 0;
  int fs_sig_state = get_code_signature( fs_fname, fs_offset, &fs_signature);
  if (fs_sig_state == 0) {
//    if (debug_flags & DEBUG_PRUSS) {
      printf( "Valid STEPPER code update found in file '%s' (version %d.%d).\n",
	      fs_fname, fs_signature.fw_version, fs_signature.fw_revision);
//    }
  }
  // Check for valid pruss code in EEPROM
  struct ucode_signature ee_signature;
  const char* ee_fname = EEPROM_PATH;
  const unsigned int ee_offset = eeprom_get_pru_code_offset( PRU_NR);
  int ee_sig_state = get_code_signature( ee_fname, ee_offset, &ee_signature);
  if (ee_sig_state == 0) {
//    if (debug_flags & DEBUG_PRUSS) {
      printf( "Valid STEPPER code found in EEPROM '%s' (version %d.%d).\n",
	      ee_fname, ee_signature.fw_version, ee_signature.fw_revision);
//    }
  }
  // Fail if neither was found
  if (fs_sig_state != 0 && ee_sig_state != 0) {
    fprintf( stderr, "ERROR: no valid STEPPER code was found (status %d.%d)\n",
	    fs_sig_state, ee_sig_state);
    return -1;
  }
#ifdef FORCE_STEPPER_CODE_FROM_FILE
  // this will force loading from file
  ee_sig_state = 9;
#else
  // Overwrite EEPROM code if an update file is present and it differs
  if (fs_sig_state == 0) {
    if (ee_sig_state != 0 || (
	 (ee_signature.fw_version != fs_signature.fw_version) ||
	 (ee_signature.fw_revision != fs_signature.fw_revision) ) )
    {
//      if (debug_flags & DEBUG_PRUSS) {
	printf( "*** Start EEPROM update, this may take a while! ****\n");
//      }
      int result = eeprom_write_pru_code( ee_fname, PRU_NR, fs_fname);
      if (result == 0) {
//        if (debug_flags & DEBUG_PRUSS) {
          printf( "*** EEPROM updated successfully, restart the application! ****\n");
//	}
      } else {
	fprintf( stderr, "*** ERROR: EEPROM code update failed! ****\n");
      }
      return -1;		// Don't commence, require a restart
    }
  }
#endif
  // At this point we should have valid PRUSS code (somewhere)...
  const char* code_fname;
  int code_offset;
  if (ee_sig_state == 0) {
    // use code from EEPROM
    code_fname = ee_fname;
    code_offset = ee_offset;
  } else {
    // use code from file
    code_fname = fs_fname;
    code_offset = fs_offset;
  }
  if (debug_flags & DEBUG_PRUSS) {
    printf( "Will load STEPPER code for PRU%d from '%s'\n", PRU_NR, code_fname);
  }
  struct ucode_signature signature;
  if (pruss_init( code_fname, code_offset, &signature) < 0) {
    // Generate some stdout output too!
    if (debug_flags & DEBUG_PRUSS) {
      printf( "PRUSS initialization failed!\n");
    }
    return -1;
  }
  /*
   * TODO: due to the new loading from either file or EEPROM, there is some redundancy here!
   */
  if (signature.ucode_magic == UCODE_MAGIC && signature.fw_version == FW_VERSION) {
    if (debug_flags & DEBUG_PRUSS) {
      printf( "PRU%d now contains STEPPER code version %d.%d.\n",
	      PRU_NR, signature.fw_version, signature.fw_revision);
    }
  } else {
    if (signature.ucode_magic == UCODE_MAGIC) {
      // This is stepper code, must be an incompatible version
      fprintf( stderr, "ERROR: the STEPPER code in file '%s' (version %d.%d) has not the required version (%d.x)!\n",
	      code_fname, signature.fw_version, signature.fw_revision, FW_VERSION);
    } else {
      // This is not stepper code.
      fprintf( stderr, "ERROR: the code in file '%s' is not STEPPER firmware!\n", code_fname);
    }
    return -1;
  }

  if (pruss_ecap_init() < 0) {
    return -1;
  }

  int ix_out = 0;
  int ix_in  = 0;
  if (debug_flags & DEBUG_PRUSS) {
    printf( "Setting FIFO pointers to %d (in) and %d (out).\n", ix_in, ix_out);
  }
  pruss_wr8( IX_IN, ix_in);		// in
  pruss_wr8( IX_OUT, ix_out);		// out
//  pruss_wr16( IX_OUT + 1, 0xdeaf);	// filler

  // for each axis clear CB storage
  for (int i = 0 ; i < 4 ; ++i) {
    for (int j = 0 ; j < 5 ; ++j) {
      pruss_wr32( PRUSS_RAM_OFFSET + 256 * (16 + i) + 4 * j, 0);
    }
  }

 /*
  * Now start the code: Enable the PRUSS.
  */
  uint16_t pc = pruss_rd16( PRUSS_PRU_CTRL_STATUS);
  pruss_start_pruss();

  if (debug_flags & DEBUG_PRUSS) {
    printf( "PRUSS successfully started at PC=%d.\n", pc);
  }
  for (int i = 1 ; i <= 4 ; ++i) {
    if (pruss_queue_set_origin( i) < 0) {
      fprintf( stderr, "Failed to execute PRUSS queue command.\n");
      exit( EXIT_FAILURE);
    }
  }
  PruCommandUnion pruCmd;
  pruCmd.enable.command	= CMD_SET_ENABLE;
  pruCmd.enable.mode	= (config_use_pololu_drivers()) ? 2 : 3;
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
#if 0
  if (DEBUG_PRUSS) {
    debug_flags &= ~DEBUG_PRUSS;
  }
#endif
  return 0;
}

int pruss_stepper_dump_state( void)
{
  int i, j;

  int pruss_ena = pruss_dump_state();

  /*
   *  Dump contents of shared memory.
   *  The pruss needn't be halted to do this, but if not, memory could change!
   */
  unsigned int ret = pruss_rd16( PRUSS_DBG_OFFSET + 4 * 2);		// R2.w0
  unsigned int sp  = pruss_rd16( PRUSS_DBG_OFFSET + 4 * 2 + 2);	// R2.w2
  unsigned int pc  = pruss_rd16( PRUSS_CTL_OFFSET + 4);
  printf( "PC == 0x%03x (%d), SP == 0x%03x (%d), RET == 0x%04x (%4d)\n", pc, pc, sp, sp, ret, ret);
  for ( sp = (sp & 0xFFF0), i = 0 ; i < 16 ; ++i) {
    ret = pruss_rd16( PRUSS_RAM_OFFSET + sp);
    if (ret == 0xdead || ret == 0xbeef) {
      break;
    }
    printf( "Stack[%3d] @ SRAM[ 0x%03x] == 0x%04x (%5d)\n", i, sp, ret, ret);
    sp += 2;
  }

  unsigned int r_state = pruss_rd16( PRUSS_DBG_OFFSET + 4 * 6);
  unsigned int cb_state = pruss_rd8( PRUSS_DBG_OFFSET + 4 * 6 + 2);
  unsigned int pruss_axis = pruss_rd8( PRUSS_DBG_OFFSET + 4 * 7 + 3);
  printf( "  Gstate= pend:%02x,act:%02x,%c%c%c, CB= state:%d,pend:%02x, Raxis= %d\n",
	  (r_state >> 9) & 31, (r_state >>1) & 31, (r_state & (1 << 15)) ? 'T' : 't',
	  (r_state & (1 << 6)) ? 'B' : 'b', (r_state & (1 << 7)) ? 'A' : 'a',
	  (cb_state >> 6) & 3, (cb_state >> 1) & 31, pruss_axis);

#define	FullADSize		(12 * 4)

  /* Dump per axis data */
  {
    uint32_t base[ 4];
    uint32_t data[ 4];

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
    printf( "%20s    %14x    %14x    %14x    %14x\n", "Base Address x", base[ 0], base[ 1], base[ 2], base[ 3]);

    DUMP_LINE( "cycleTimer .",       32,  0, "%14u");
    DUMP_LINE( "stepActiveTime .",   32,  4, "%14u");
    DUMP_LINE( "minStepCycleTime .", 32,  8, "%14u");
    DUMP_LINE( "stepCycleTime .",    32, 12, "%14u");
    DUMP_LINE( "stateInfo x",        16, 16, "%14x");
    // following code uses stepPinBitNo from data[] !
    printf( "%20s    %14u    %14u    %14u    %14u\n", "stepPinBitNo .",
	    (data[ 0] >> 12), (data[ 1] >> 12), (data[ 2] >> 12), (data[ 3] >> 12));
    // following code uses moveReverseBit from data[] !
    printf( "%20s    %14u    %14u    %14u    %14u\n", "Axis reversal .",
	    !!(data[ 0] & (1 << 11)), !!(data[ 1] & (1 << 11)),
	    !!(data[ 2] & (1 << 11)), !!(data[ 3] & (1 << 11)));
    // following code uses decelerateBit from data[] !
    printf( "%20s    %14u    %14u    %14u    %14u\n", "Accelerate .",
	    !!(data[ 0] & (1 << 0)), !!(data[ 1] & (1 << 0)),
	    !!(data[ 2] & (1 << 0)), !!(data[ 3] & (1 << 0)));
    printf( "%20s    %14u    %14u    %14u    %14u\n", "Decelerate .",
	    !!(data[ 0] & (1 << 1)), !!(data[ 1] & (1 << 1)),
	    !!(data[ 2] & (1 << 1)), !!(data[ 3] & (1 << 1)));
    printf( "%20s    %14u    %14u    %14u    %14u\n", "Move Done .",
	    !!(data[ 0] & (1 << 3)), !!(data[ 1] & (1 << 3)),
	    !!(data[ 2] & (1 << 3)), !!(data[ 3] & (1 << 3)));
    DUMP_LINE( "moveCounter .",      16, 18, "%14u");
    DUMP_LINE( "virtPos x",          32, 20, "%14x");
    printf( "%20s    %14d    %14d    %14d    %14d\n", "virtPos .",
	    data[ 0] - VIRT_POS_MID_SCALE,
	    data[ 1] - VIRT_POS_MID_SCALE,
	    data[ 2] - VIRT_POS_MID_SCALE,
	    data[ 3] - VIRT_POS_MID_SCALE);

    DUMP_LINE( "stepSize .",         32, 24, "%14u");

    DUMP_LINE( "requestedPos x",     32, 32, "%14x");
    printf( "%20s    %14d    %14d    %14d    %14d\n", "requestedPos .",
	    data[ 0] - VIRT_POS_MID_SCALE,
	    data[ 1] - VIRT_POS_MID_SCALE,
	    data[ 2] - VIRT_POS_MID_SCALE,
	    data[ 3] - VIRT_POS_MID_SCALE);
    DUMP_LINE( "nextStepCycleTime .",32, 36, "%14u");
    DUMP_LINE( "accelCount .",       32, 40, "%14u");
    DUMP_LINE( "dividend .",         32, 44, "%14u");
  }

  for (i = 0 ; i < 4 ; ++i) {
    // for each axis dump first part of CB storage
    printf( "CB[ %d] -", i);
    for (j = 0 ; j < 4 ; ++j) {
	    uint32_t data = pruss_rd32( PRUSS_RAM_OFFSET + 256 * (16 + i) + 4 * j);
      printf( " 0x%08x", data);
    }
    printf( "\n");
  }

  if (pruss_ena) {
    // Set bit15 in R6 to signal the PRUSS we're resuming from suspend
    uint32_t reg = pruss_rd32( PRUSS_DBG_OFFSET + 6 * 4);
    pruss_wr32( PRUSS_DBG_OFFSET + 6 * 4, reg | (1 << 15));
    pruss_start_pruss();
    printf( "PRUSS is resuming execution (enabled)\n");
  }
  return 0;
}

static inline int pruss_get_nr_of_free_buffers( void)
{
  int ix_in  = pruss_rd8( IX_IN);
  int ix_out = pruss_rd8( IX_OUT);
  // original formula: NR_CMD_FIFO_ENTRIES - 1 - (NR_CMD_FIFO_ENTRIES + ix_in - ix_out) % NR_CMD_FIFO_ENTRIES;
  return ((ix_out > ix_in) ? 0 : NR_CMD_FIFO_ENTRIES) + ix_out - ix_in - 1;
}

int pruss_queue_full( void)
{
  return (pruss_get_nr_of_free_buffers() == 0);
}

int pruss_queue_empty( void)
{
  // Note that one buffer cannot be used because of the two indexes scheme!
  return (pruss_get_nr_of_free_buffers() == NR_CMD_FIFO_ENTRIES - 1);
}

// Simple wrapper prevents need for pruss.h inclusion
int pruss_stepper_halted( void)
{
  return pruss_is_halted();
}

// Simple wrapper prevents need for pruss.h inclusion
void pruss_stepper_resume( void)
{
  pruss_resume_pruss();
}

// Simple wrapper prevents need for pruss.h inclusion
void pruss_stepper_single_step( void)
{
  pruss_single_step_pruss();
}

int pruss_wait_for_queue_space( void)
{
  int timeout = 30000;	// set to 30 seconds
  while (pruss_queue_full()) {
    if (timeout <= 0) {
      return -1;
    }
    if (pruss_is_halted()) {
      return -1;
    }
    /*
     * Most time is spent here polling for PRUSS progress.
     * Until an interrupt driven interface is implemented, reduce the
     * cpu load and number of poll cycles by sleeping part of the time.
     */
    usleep( 1000);
    --timeout;
  }
  return 0;
}

int pruss_stepper_busy( void)
{
  return pruss_rd8( BUSY_FLAG);
}

// Write command structure to buffer[ ix_in] on PRUSS (must be free)
int pruss_write_command_struct( int ix_in, PruCommandUnion* data)
{
  uint32_t a = ix_in * sizeof( data->gen);
  for (int i = 0 ; i < NR_ITEMS( data->gen) ; ++i) {
    uint32_t u = data->gen[ i];
    pruss_wr32( PRUSS_RAM_OFFSET + 1 * 256 + a, u);
//    printf( "pruss_write_command_struct: wrote 0x%08x (%d) to offset %d\n", u, u, a);
    a += sizeof( *data->gen);
  }
  if (++ix_in >= NR_CMD_FIFO_ENTRIES) {
    ix_in = 0;
  }
  pruss_wr8( IX_IN, ix_in);
  return ix_in;
}

// Write command structure to PRUSS, wait for free buffer is nescessary
static int pruss_command( PruCommandUnion* cmd)
{
  double t0 = 0;
  int ix_in = pruss_rd8( IX_IN);
  int ix_out = pruss_rd8( IX_OUT);
  if (DBG( DEBUG_PRUSS + DEBUG_VERBOSE)) {
    t0 = timestamp_get();
  }
  if (pruss_wait_for_queue_space() < 0) {
    pruss_stepper_dump_state();
    printf( "ERROR: found pruss halted waiting for queue space for command %d, bailing out!\n",
	    cmd->command.value);
    exit( EXIT_FAILURE);
  }
  (void) pruss_write_command_struct( ix_in, cmd);
  if (DBG( DEBUG_PRUSS + DEBUG_VERBOSE)) {
    double t1 = timestamp_get();
    printf( "pruss_command started at %1.3lfs - wrote to SRAM buffer at index %d, out index was %d. Operation took %1.3fms.\n",
	    t0, ix_in, ix_out, 1000 * (t1 - t0));
  }
  return 0;
}

/*
 *  PRUSS STEPPER.BIN COMMAND INTERFACE
 */

int pruss_queue_set_position( int axis, int32_t pos)
{
  PruCommandUnion pruCmd = {
    .set_origin.command		= CMD_AXIS_SET_ORIGIN,
    .set_origin.axis		= axis,
    .set_origin.position 	= pos
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_set_origin( int axis)
{
  return pruss_queue_set_position( axis, 0);
}

/*
 *  Shift the origin of the PRUSS coordinates to a location.
 *  This should be the 'current' machine position in the higher
 *  level software.
 */
int pruss_queue_adjust_origin( int axis, int32_t delta)
{
  PruCommandUnion pruCmd = {
    .adjust_origin.command	= CMD_AXIS_ADJUST_ORIGIN,
    .adjust_origin.axis		= axis,
    .adjust_origin.position 	= delta
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

int pruss_queue_accel( int axis, uint32_t n0, uint32_t c0, uint32_t cmin, int32_t delta)
{
  PruCommandUnion pruCmd = {
    .move.n0	 		= n0,
    .move.command		= CMD_AXIS_MOVE,
    .move.axis			= axis,
    .move.position		= delta,
    .move.c0	 		= c0,
    .move.cn			= cmin,
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_dwell( int axis, uint32_t cmin, int32_t delta)
{
  PruCommandUnion pruCmd = {
    .move.n0	 		= 0,		// not used
    .move.command		= CMD_AXIS_MOVE,
    .move.axis			= axis,
    .move.position		= delta,
    .move.c0	 		= cmin,
    .move.cn			= cmin,		// equal to c0 to create dwell
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_decel( int axis, uint32_t nmin, uint32_t cmin, int32_t delta)
{
  PruCommandUnion pruCmd = {
    .move.n0	 		= nmin,
    .move.command		= CMD_AXIS_MOVE,
    .move.axis			= axis,
    .move.position		= delta,
    .move.c0	 		= cmin,
    .move.cn			= cmin + 1,	// greater than c0 to create ramp-down
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_exec_limited( uint8_t mask, uint8_t invert)
{
  if (pruss_is_halted()) {
    fprintf( stderr, "FATAL: PRUSS found halted when queueing execute command\n");
    pruss_stepper_dump_state();
    exit( EXIT_FAILURE);
  }
  //  fprintf( stderr, "pruss_queue_execute(): free buffers = %d\n",
  //	   pruss_get_nr_of_free_buffers());
  PruCommandUnion pruCmd;
  pruCmd.command.value		= CMD_AXES_EXECUTE;
  pruCmd.gen[ 1]		= mask | (invert << 8);
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_execute( void)
{
  return pruss_queue_exec_limited( 0, 0);	// no limits !
}

int pruss_queue_config_axis( int axis, uint32_t ssi, int reverse)
{
  PruCommandUnion pruCmd = {
    .config.command		= CMD_AXIS_CONFIG_AXIS,
    .config.axis		= axis,
    .config.reverse		= (reverse) ? 1 : 0,
    .config.stepSize		= ssi,
    .config.reciprStepSize	= (uint32_t) (0xFFFFFFFF / ssi),	// as close as possible
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_queue_config_limsw( int axis, uint8_t min_gpio, uint8_t min_invert, uint8_t max_gpio, uint8_t max_invert)
{
  PruCommandUnion pruCmd = {
    .limsw.command		= CMD_AXIS_CONFIG_LIMSW,
    .limsw.axis			= axis,
    .limsw.min_gpio		= min_gpio,
    .limsw.min_invert		= min_invert,
    .limsw.max_gpio		= max_gpio,
    .limsw.max_invert		= max_invert,
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_wait_for_completion( void)
{
  while (!pruss_queue_empty() || pruss_stepper_busy()) {
    if (pruss_stepper_halted()) {
      return -1;
    }
    usleep( 500);
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
    .enable.mode		= (on) ? 1 : 0,
  };
  if (pruss_command( &pruCmd) < 0) {
    return -1;
  }
  return 0;
}

int pruss_dump_position( void)
{
  const char axes[] = { '?', 'X', 'Y', 'Z', 'E' };
  int axis;

  for (axis = 1 ; axis <= 4 ; ++axis) {
    uint32_t base = PRUSS_RAM_OFFSET + (axis - 1) * FullADSize;
    int32_t virtPosI = pruss_rd32( base + 20) - VIRT_POS_MID_SCALE;
    printf( "  %c-virtPos = %d", axes[ axis], virtPosI);
  }
  printf( "\n");
  for (axis = 1 ; axis <= 4 ; ++axis) {
    uint32_t base = PRUSS_RAM_OFFSET + (axis - 1) * FullADSize;
    int32_t requestedPos = pruss_rd32( base + 32) - VIRT_POS_MID_SCALE;
    printf( "  %c-requestedPos = %d", axes[ axis], requestedPos);
  }
  printf( "\n");
  return 0;
}

int pruss_get_positions( int axis, int32_t* virtPosI, int32_t* requestedPos)
{
  uint32_t base = PRUSS_RAM_OFFSET + (axis - 1) * FullADSize;
  if (requestedPos) {
    *requestedPos = pruss_rd32( base + 32) - VIRT_POS_MID_SCALE;
  }
  if (virtPosI) {
    *virtPosI = pruss_rd32( base + 20) - VIRT_POS_MID_SCALE;
  }
  return 0;
}

/*
 * exit handling: return to safe state for program exit
 */
void pruss_queue_exit( void)
{
  bebopr_exit();
}
