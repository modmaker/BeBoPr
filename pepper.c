
#include <unistd.h>
#include <stdio.h>

#include "gpio.h"
#include "pruss.h"
#include "pepper.h"
#include "bebopr.h"

/*
 *  The penta-stepper board (PEPPER) is supported
 *  on bridged capes only !
 *
 */
#if !defined( DEPEND) && !defined( BONE_BRIDGE) && !defined( BONE_ENA_PATCH)
# error "unsupported configuration !"
#endif

#ifdef PEPPER

static inline void pepper_set_data( int value)
{
  // the signal is inverted on the PEPPER !
  gpio_write_value_to_pin_file( 15, "value", (value)?"0":"1");
}

static inline void pepper_set_clock( int value)
{
  // the signal is inverted on the PEPPER !
  gpio_write_value_to_pin_file( 14, "value", (value)?"0":"1");
}

void delay( void)
{
//  usleep( 2000);
  usleep( 20);
}

/*
 *  Bit-Bang SPI for now (until the PRU takes over).
 */
void pepper_send_byte( uint8_t data)
{
  // start time window for serial output
  gpio_write_value_to_pin_file( 66, "value", "1");	// negate
  usleep( 250);
  pepper_set_clock( 0);
  pepper_set_data( 0);
  gpio_write_value_to_pin_file( 66, "value", "0");	// (re-)assert
  usleep( 250);

  // unlock, data negation while clock is active
#if 0
  delay();
  pepper_set_clock( 0);
  delay();
  pepper_set_data( 1);
  delay();
  pepper_set_clock( 1);
  delay();
  pepper_set_data( 0);
//  delay();
  pepper_set_clock( 0);
  delay();
#endif

  // send byte, lsb first
  for (int i = 0 ; i < 8 ; ++i) {
    pepper_set_clock( 0);
    pepper_set_data( (data & 0x80)?1:0);
    data <<= 1;
    delay();
    pepper_set_clock( 1);
//    delay();
  }
  pepper_set_clock( 0);
  delay();
}

uint16_t crc16_update( uint16_t crc, uint8_t data)
{
  crc ^= data;
  for (int i = 0 ; i < 8 ; ++i) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

struct ms_struct {
  uint8_t x      : 3;
  uint8_t y      : 3;
  uint8_t z      : 3;
  uint8_t a      : 3;
  uint8_t b      : 3;
};

struct ena_reg_struct {
  uint8_t pwm    : 5;
  uint8_t decay  : 2;
  uint8_t ena    : 1;
};

struct ena_struct {
  struct ena_reg_struct x, y, z, a, b;
};

union pepper_config_union {
  struct {
    uint16_t header;
    uint8_t version    : 4;
    uint8_t revision   : 4;
    struct ms_struct ms;
    struct ena_struct ena[ 4];
    uint16_t check;
  };
  uint8_t bytes[ 0];
} cfg;

static inline unsigned int get_mode_bits( axis_e axis)
{
  switch (config_get_micro_step_factor( axis)) {
  case 2:  return 1;
  case 4:  return 2;
  case 8:  return 3;
  case 16: return 4;
  case 32: return 5;
  }
  return 0;
}

int pepper_init( void)
{
  // set header and version info, clear checksum field
  cfg.header   = 0xba51;
  cfg.version  = 1;
  cfg.revision = 0;
  cfg.check    = 0;
  // set stepper driver mode select fields
  cfg.ms.x = get_mode_bits( x_axis);
  cfg.ms.y = get_mode_bits( y_axis);
  cfg.ms.z = get_mode_bits( z_axis);
  cfg.ms.a = get_mode_bits( e_axis);
  cfg.ms.b = 0;
  // set all enable state registers
  for (int j = 0 ; j < 4 ; ++j) {	// all four enable states
    struct ena_struct* es = &cfg.ena[ j];

    for (int i = 0 ; i < 5 ; ++i) {	// all five axes

      struct ena_reg_struct* ers = NULL;
      axis_e axis;

      switch (i) {
      case 0: ers = &es->x; axis = x_axis; break;
      case 1: ers = &es->y; axis = y_axis; break;
      case 2: ers = &es->z; axis = z_axis; break;
      case 3: ers = &es->a; axis = e_axis; break;
      case 4: ers = &es->b; axis = e_axis; break;	// TODO: fix after extension to 5 axes
      }
      unsigned int active_current = config_get_active_current( axis);
      unsigned int hold_current = config_get_idle_current( axis);
      if (j == 0) {	// enable mode 0
        ers->pwm = hold_current;
	ers->ena = (hold_current > 0) ? 1 : 0;
        ers->decay = 0;		// slow
      } else {		// enable modes 1,2&3
        ers->pwm = active_current;
        ers->ena = 1;
        ers->decay = 2;		// mixed
//        ers->decay = 0;		// slow
      }
    }
  }

  int pruss_idle = pruss_is_halted();
  if (!pruss_idle) {
    pruss_halt_pruss();	// keep pruss from setting gpio signals
  }

  pepper_send_byte( sizeof( cfg));
  pepper_send_byte( 0x51);
  pepper_send_byte( sizeof( cfg));
  uint16_t check = 0;
  for (int i = 0 ; i < sizeof( cfg) ; ++i) {
    if (i < sizeof( cfg) - sizeof( cfg.check)) {
      check = crc16_update( check, cfg.bytes[ i]);
    } else if (i == sizeof( cfg) - sizeof( cfg.check)) {
      cfg.check = check;
    }
    pepper_send_byte( cfg.bytes[ i]);
  }
//  pepper_send_byte( sizeof( cfg));
//  pepper_send_byte( sizeof( cfg));
//  pepper_send_byte( sizeof( cfg));

  if (!pruss_idle) {
    pruss_resume_pruss();	// restore running state
  }

  printf( "PEPPER configuration: sent %d bytes, crc16 is $%04X\n",
	  sizeof( cfg), cfg.check);
  
  return 0;
}

#endif
