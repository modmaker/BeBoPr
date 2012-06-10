
//  PRUSS_MAGIC is in the reserved opcodes list, so there is
//  less chance of finding it by accident. It identifies valid
//  microcode for the BeagleBone. It should be located immediately
//  after the first instruction (that should be jump to code start).
//  UCODE_MAGIC identifies the application, in this case code for
//  the BeBoPr (2191) board.

#define PRUSS_MAGIC             0xbeb0c0de
#define UCODE_MAGIC             0xba512191

//  PRUSS code and C-code must have the same FW_VERSION to be compatible
#define FW_VERSION              3

#define	CMD_AXIS_SET_ORIGIN	0
#define CMD_AXIS_RAMP_UP	1
#define CMD_AXIS_DWELL		2
#define CMD_AXIS_RAMP_DOWN	3
#define CMD_AXES_EXECUTE	4
#define	CMD_AXIS_SET_ACCEL	5
#define	CMD_AXIS_SET_PULSE_LENGTH 6
#define	CMD_AXIS_CONFIG_AXIS	7
#define	CMD_AXIS_ADJUST_ORIGIN	8

#define CMD_SET_IDLE_TIMEOUT	11
#define CMD_SET_ENABLE		12

#define CMD_HALT_PRU		15
