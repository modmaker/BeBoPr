
//  PRUSS_MAGIC is in the reserved opcodes list, so there is
//  less chance of finding it by accident. It identifies valid
//  STEPPER code for the BeagleBone. It should be located immediately
//  after the first instruction (that should be jump to code start).
//  UCODE_MAGIC identifies the application, in this case code for
//  the BeBoPr (2191) board.

#define PRUSS_MAGIC             0xbeb0c0de
#define UCODE_MAGIC             0xba512191

//  PRUSS code and C-code must have the same FW_VERSION to be compatible
#ifdef BONE_BRIDGE
#define FW_VERSION              7
#else
#define FW_VERSION              6
#endif

#define NR_CMD_FIFO_ENTRIES	64

//
// The stepper code uses a 32-bit unsigned integer to keep track of position.
// This gives a usable range of a little more than 4000 mm.
// To split this range from -2000 .. +2000, set the virtual origin midscale:
//
#define VIRT_POS_MID_SCALE	0x80000000

//
// Once a command is decoded, it behaves according to
// one of the three following groups:
//
// 1. ASYNC and simple SYNC commands, id 0 - 7
//    these commands are executed immediately (ASYNC) or wait first
//    for the steppers to become not busy (SYNC)
#define CMD_AXES_EXECUTE		0
#define CMD_SET_ENABLE			1
#define CMD_SET_IDLE_TIMEOUT		2
#define CMD_HALT_PRU			3
// 2. complex SYNC  commands, id 8 -15
//    these commands are axis specific and wait
//    for the steppers to become not busy
#define START_CMD_AXIS_COMPLEX		8
#define	CMD_AXIS_SET_PULSE_LENGTH	8
#define	CMD_AXIS_CONFIG_AXIS		9
#define	CMD_AXIS_ADJUST_ORIGIN		10
#define	CMD_AXIS_SET_ORIGIN		12
#define	CMD_AXIS_CONFIG_LIMSW		13
#define	CMD_AXIS_SET_ACCEL		14
// 3. QUEUE commands, id 16-31
//    these commands are queued before being started with the execute command
#define START_CMD_AXIS_QUEUE		16
#define CMD_AXIS_MOVE			16
//
