#ifndef _BEAGLEBONE_H
#define _BEAGLEBONE_H

#include <stdint.h>
#include <sched.h>

/* Where to find the BeBoPr's EEPROM: */
#define EEPROM_PATH ( (get_kernel_type() == e_kernel_3_8) ? \
			"/sys/class/i2c-adapter/i2c-1/1-0054/eeprom" : \
			"/sys/class/i2c-adapter/i2c-3/3-0054/eeprom" )

/*
 * Posix defines priorities from 0 upto 99.
 *  0 (lowest) is for SCHED_OTHER, the default scheduling algorithm
 *  1 (low)    is for SCHED_RR and SCHED_FIFO
 * 99 (high)   is for SCHED_RR and SCHED_FIFO
 *
 * The Linux kernel uses other values: Posix 0 gives Linux 120,
 * Posix priority 1 gives Linux priority 98, Posix priority 99 gives Linux priority 0.
 */

#define TOP_PRIO	60
#define ELEV_PRIO	55
#define NORM_PRIO	50

#define COMM_PRIO	ELEV_PRIO	/* keep connection allive at all times */
#define COMM_SCHED	SCHED_FIFO

#define LIMSW_PRIO      TOP_PRIO
#define LIMSW_SCHED	SCHED_FIFO

#define MENDEL_PRIO     TOP_PRIO
#define MENDEL_SCHED	SCHED_FIFO

#define HEATER_PRIO	NORM_PRIO
#define HEATER_SCHED	SCHED_RR

#define ANALOG_PRIO	ELEV_PRIO
#define ANALOG_SCHED	SCHED_RR

#define HOME_PRIO	ELEV_PRIO
#define HOME_SCHED	SCHED_RR

#define NR_ITEMS( x) (sizeof( (x)) / sizeof( *(x)))

/* convert [mm/min] into [m/s] */
#define	FEED2SI( f) ((f) / 60000.0)
/* convert into reciprocal */
#define RECIPR( x) (1.0 / (x))
/* convert SI unit [s] into [ms] */
#define SI2MS( x) (1.0E3 * (x))
/* convert SI unit [m] into [mm] */
#define SI2MM( x) (1.0E3 * (x))
/* convert SI unit [m] into [um] */
#define SI2UM( x) (1.0E6 * (x))
/* convert SI unit [m] into [nm] */
#define SI2NM( x) (1.0E9 * (x))

#define MM2POS( x) (int32_t)(1.0E6 * (x))
#define POS2MM( x) (double)(1.0E-6 * (x))
#define SI2POS( x) (int32_t)(1.0E9 * (x))
#define POS2SI( x) (double)(1.0E-9 * (x))

// Define the frequency of the PRUSS clock
#define PRUSS_CLOCK	200.0E6

typedef const char* channel_tag;
static inline const char* tag_name( channel_tag tag) { return (char*)tag; }

#endif
