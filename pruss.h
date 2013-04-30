#ifndef _PRUSS_H
#define _PRUSS_H

#include <stdint.h>

#ifndef PRU_NR
#  define PRU_NR	0
#endif

// These are global map offsets !
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

#if PRU_NR == 0
#  define PRUSS_RAM_OFFSET   PRUSS_RAM0_OFFSET
#  define PRUSS_CTL_OFFSET   PRUSS_CTL0_OFFSET
#  define PRUSS_DBG_OFFSET   PRUSS_DBG0_OFFSET
#  define PRUSS_IRAM_OFFSET  PRUSS_IRAM0_OFFSET
#elif PRU_NR == 1
#  define PRUSS_RAM_OFFSET   PRUSS_RAM1_OFFSET
#  define PRUSS_CTL_OFFSET   PRUSS_CTL1_OFFSET
#  define PRUSS_DBG_OFFSET   PRUSS_DBG1_OFFSET
#  define PRUSS_IRAM_OFFSET  PRUSS_IRAM1_OFFSET
#else
#  error Illegal PRU_NR setting
#endif


/*
 * PRUSS control register offsets and bits
 */
#define PRUSS_PRU_CTRL_CONTROL          (PRUSS_CTL_OFFSET + 0)
#define PRUSS_PRU_CTRL_STATUS           (PRUSS_CTL_OFFSET + 4)
#define PRUSS_PRU_CTRL_WAKEUP_EN        (PRUSS_CTL_OFFSET + 8)
#define PRUSS_PRU_CTRL_CYCLE            (PRUSS_CTL_OFFSET + 12)
#define PRUSS_PRU_CTRL_STALL            (PRUSS_CTL_OFFSET + 16)
#define PRUSS_PRU_CTRL_CTBIR0           (PRUSS_CTL_OFFSET + 32)
#define PRUSS_PRU_CTRL_CTBIR1           (PRUSS_CTL_OFFSET + 36)
#define PRUSS_PRU_CTRL_CTPPR0           (PRUSS_CTL_OFFSET + 40)
#define PRUSS_PRU_CTRL_CTPPR1           (PRUSS_CTL_OFFSET + 44)

#define PRUSS_PRU_CTRL_CONTROL_RUNSTATE         (1 << 15)
#define PRUSS_PRU_CTRL_CONTROL_NRESET           (1 <<  0)
#define PRUSS_PRU_CTRL_CONTROL_ENABLE           (1 <<  1)
#define PRUSS_PRU_CTRL_CONTROL_COUNTER_ENABLE   (1 <<  3)
#define PRUSS_PRU_CTRL_CONTROL_SINGLE_STEP      (1 <<  8)

struct ucode_signature {
	uint32_t	pruss_magic;
	uint32_t	ucode_magic;
	uint16_t	fw_revision;
	uint16_t	fw_version;
	uint32_t	spare[ 6];
};

/* Low level interface */

extern uint32_t pruss_rd32( unsigned int addr);
extern uint16_t pruss_rd16( unsigned int addr);
extern uint8_t pruss_rd8( unsigned int addr);
extern void pruss_wr32( unsigned int addr, uint32_t data);
extern void pruss_wr16( unsigned int addr, uint16_t data);
extern void pruss_wr8( unsigned int addr, uint8_t data);

/* High level interface */

extern int locate_pruss_device( const char* driver_name, char* drv_name, int drv_name_len, char* uio_name, int uio_name_len);
extern int map_device( const char* uio_name);
extern int pruss_load_code( const char* fname, unsigned int offset, unsigned int* start_addr, struct ucode_signature* signature);
extern int pruss_init( const char* ucodename, unsigned int offset, struct ucode_signature* signature);
extern void pruss_wait_for_halt( void);
extern int pruss_dump_state( void);
extern int pruss_halt_pruss( void);
extern int pruss_stop_pruss( void);
extern void pruss_start_pruss( void);
extern int pruss_is_halted( void);
extern void pruss_resume_pruss( void);
extern void pruss_single_step_pruss( void);


#endif
