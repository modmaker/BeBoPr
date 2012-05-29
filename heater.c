#include	<stdlib.h>
#include	<stdlib.h>
#include	<stdint.h>
#include	<fcntl.h>
#include	<stdio.h>
#include	<unistd.h>
#include	<pthread.h>

#include	"heater.h"
#include	"debug.h"
#include	"temp.h"
#include	"beaglebone.h"
#include	"mendel.h"


typedef struct {
  temp_sensor_e		sensor;
  pwm_output_e		pwm_output;
  double		setpoint;
  pid_struct		pid_settings;
  double		(*get_temperature)( void);
} heater_struct;

static heater_struct heaters[ e_heater_num_outputs];

// Use control_lock for access to control loop settings
static pthread_rwlock_t	control_lock;

/*
 * Configuration settings are stored seperately (bebopr_rx.c) and
 * a configuration call is used to communicate these with this
 * code.
 */
static const heater_config_struct* heater_config_data = NULL;
static int heater_config_items = 0;

int heater_config( const heater_config_struct* config_data, int nr_config_items)
{
  heater_config_data  = config_data;
  heater_config_items = nr_config_items;
  return 0;
}

/*
 * This is the worker thread that reads the analog inputs and
 * calls the callbacks to export the read values.
 */
void* heater_thread( void* arg)
{
  while (1) {
    usleep( 1000);
    sched_yield();
  }
}

static pthread_t worker;

/*
 * set defaults, init pwm hardware, set heater state to off.
 * create the heaters thread
 * 
 */
int heater_init( void)
{
  if (heater_config_data != NULL) {
    // No need to locks as there's no thread running yet!
    for (int i = 0 ; i < heater_config_items ; ++i) {
      heater_e		heater		= heater_config_data[ i].heater;
      temp_sensor_e	sensor		= heater_config_data[ i].sensor;
      pwm_output_e	pwm_output	= heater_config_data[ i].pwm_output;
      heaters[ heater].sensor		= sensor;
      heaters[ heater].pwm_output	= pwm_output;
      //      analog_set_update_callback( channel, temp_update, (update_channel_t)sensor);
      heaters[ heater].setpoint = 0.0;
      heaters[ heater].pid_settings.k = 0.0;
      heaters[ heater].pid_settings.p = 0.0;
      heaters[ heater].pid_settings.i = 0.0;
      heaters[ heater].pid_settings.d = 0.0;
      heaters[ heater].get_temperature = NULL;
    }
    // Initialize temperature and pwm subsystems
    mendel_sub_init( "temp", temp_init);
    mendel_sub_init( "pwm", pwm_init);
    // Start worker thread
    return mendel_thread_create( "heater", &worker, NULL, &heater_thread, NULL);
  }
  fprintf( stderr, "temp_init: no configuration data!\n");
  extern void mendel_log_init( const char* name, int result);
  return -1;
}

/*
 * turn heater channel on or off
 */
int heater_enable( heater_e heater, int state)
{
  return -1;
}

/*
 * set heater pwm output for given channel to value
 */
int heater_set_raw_pwm( int heater, double percentage)
{
  return -1;
}

/*
 * set setpoint for a heater
 */
int heater_set_setpoint( int heater, double setpoint)
{
  if (heater >= 0 && heater < NR_ITEMS( heaters)) {
    pthread_rwlock_wrlock( &control_lock);
    heaters[ heater].setpoint = setpoint;
    pthread_rwlock_unlock( &control_lock);
    return 0;
  }
  return -1;
}

/*
 * get setpoint for a heater
 */
int heater_get_setpoint( int heater, double* setpoint)
{
  if (setpoint != NULL && heater >= 0 && heater < NR_ITEMS( heaters)) {
    pthread_rwlock_rdlock( &control_lock);
    *setpoint = heaters[ heater].setpoint;
    pthread_rwlock_unlock( &control_lock);
    return 0;
  }
  return -1;
}

/*
 * set PID values for a heater
 */
int heater_set_pid_values( int heater, const pid_struct* pid_settings)
{
  if (heater >= 0 && heater < NR_ITEMS( heaters)) {
    pthread_rwlock_wrlock( &control_lock);
    heaters[ heater].pid_settings = *pid_settings;
    pthread_rwlock_unlock( &control_lock);
    return 0;
  }
  return -1;
}

/*
 * get PID values for a heater
 */
int heater_get_pid_values( int heater, pid_struct* pid_settings)
{
  if (heater >= 0 && heater < NR_ITEMS( heaters)) {
    pthread_rwlock_rdlock( &control_lock);
    *pid_settings = heaters[ heater].pid_settings;
    pthread_rwlock_unlock( &control_lock);
    return 0;
  }
  return -1;
}

const char* fname = "./heater-pid-factors";

/*
 * Write PID factors to persistent storage
 */
int heater_save_settings( void)
{
  int fd;
  int ret;
  
  fd = open( fname, O_WRONLY);
  if (fd < 0) {
    perror( "heater: opening of file '%s' failed");
    return -1;
  }
  pthread_rwlock_rdlock( &control_lock);
  ret = write( fd, heaters, sizeof( heaters));
  pthread_rwlock_unlock( &control_lock);
  if (ret < 0) {
    perror( "heater: writing file '%s' failed");
    return ret;
  } else if (ret != sizeof( heaters)) {
    return -1;
  }
  return 0;
}


/*
 * Read PID factors from persistent storage
 */
int heater_load_settings( void)
{
  int fd;
  int ret;
  
  fd = open( fname, O_WRONLY);
  if (fd < 0) {
    perror( "heater: opening of file '%s' failed");
    return -1;
  }
  pthread_rwlock_wrlock( &control_lock);
  ret = read( fd, heaters, sizeof( heaters));
  pthread_rwlock_unlock( &control_lock);
  if (ret < 0) {
    perror( "heater: reading file '%s' failed");
    return ret;
  } else if (ret != sizeof( heaters)) {
    return -1;
  }
  return 0;
}



#if 0

/**
	\var heaters_pid
	\brief this struct holds the heater PID factors

	PID is a fascinating way to control any closed loop control, combining the error (P), cumulative error (I) and rate at which we're approacing the setpoint (D) in such a way that when correctly tuned, the system will achieve target temperature quickly and with little to no overshoot

	At every sample, we calculate \f$OUT = k_P (S - T) + k_I \int (S - T) + k_D \frac{dT}{dt}\f$ where S is setpoint and T is temperature.

	The three factors kP, kI, kD are chosen to give the desired behaviour given the dynamics of the system.

	See http://www.eetimes.com/design/embedded/4211211/PID-without-a-PhD for the full story
*/
struct {
int32_t						p_factor; ///< scaled P factor
	int32_t						i_factor; ///< scaled I factor
	int32_t						d_factor; ///< scaled D factor
	int16_t						i_limit;  ///< scaled I limit, such that \f$-i_{limit} < i_{factor} < i_{limit}\f$
} heaters_pid[NUM_HEATERS];

/// \brief this struct holds the runtime heater data- PID integrator history, temperature history, sanity checker
struct {
	int16_t						heater_i; ///< integrator, \f$-i_{limit} < \sum{\Delta t} < i_{limit}\f$

	uint16_t					temp_history[TH_COUNT]; ///< store last TH_COUNT readings in a ring, so we can smooth out our differentiator
	uint8_t						temp_history_pointer;   ///< pointer to last entry in ring

	#ifdef	HEATER_SANITY_CHECK
		uint16_t					sanity_counter;				///< how long things haven't seemed sane
		uint16_t					sane_temperature;			///< a temperature we consider sane given the heater settings
	#endif

	uint8_t						heater_output;					///< this is the PID value we eventually send to the heater
} heaters_runtime[NUM_HEATERS];

/// default scaled P factor, equivalent to 8.0
#define		DEFAULT_P				8192
/// default scaled I factor, equivalent to 0.5
#define		DEFAULT_I				512
/// default scaled D factor, equivalent to 24
#define		DEFAULT_D				24576
/// default scaled I limit
#define		DEFAULT_I_LIMIT	384

/// this lives in the eeprom so we can save our PID settings for each heater
typedef struct {
	int32_t		EE_p_factor;
	int32_t		EE_i_factor;
	int32_t		EE_d_factor;
	int16_t		EE_i_limit;
	uint16_t	crc; ///< crc so we can use defaults if eeprom data is invalid
} EE_factor;

#if ARCH == arm
// TODO: implementation
EE_factor EE_factors[NUM_HEATERS];
#else
EE_factor EEMEM EE_factors[NUM_HEATERS];
#endif

/// \brief initialise heater subsystem
/// Set directions, initialise PWM timers, read PID factors from eeprom, etc
void heater_init() {
	heater_t i;
	// setup pins
	for (i = 0; i < NUM_HEATERS; i++) {
		if (heaters[i].heater_pwm) {
			*heaters[i].heater_pwm = 0;
			// this is somewhat ugly too, but switch() won't accept pointers for reasons unknown
#if ARCH == arm
			// TODO: implementation
#else
			switch((uint16_t) heaters[i].heater_pwm) {
				case (uint16_t) &OCR0A:
					TCCR0A |= MASK(COM0A1);
					break;
				case (uint16_t) &OCR0B:
					TCCR0A |= MASK(COM0B1);
					break;
				case (uint16_t) &OCR2A:
					TCCR2A |= MASK(COM2A1);
					break;
				case (uint16_t) &OCR2B:
					TCCR2A |= MASK(COM2B1);
					break;
				#ifdef TCCR3A
				case (uint16_t) &OCR3AL:
					TCCR3A |= MASK(COM3A1);
					break;
				case (uint16_t) &OCR3BL:
					TCCR3A |= MASK(COM3B1);
					break;
				case (uint16_t) &OCR3CL:
					TCCR3A |= MASK(COM3C1);
					break;
				#endif
				#ifdef	TCCR4A
				case (uint16_t) &OCR4AL:
					TCCR4A |= MASK(COM4A1);
					break;
				case (uint16_t) &OCR4BL:
					TCCR4A |= MASK(COM4B1);
					break;
				case (uint16_t) &OCR4CL:
					TCCR4A |= MASK(COM4C1);
					break;
				#endif
				#ifdef	TCCR5A
				case (uint16_t) &OCR5AL:
					TCCR5A |= MASK(COM5A1);
					break;
				case (uint16_t) &OCR5BL:
					TCCR5A |= MASK(COM5B1);
					break;
				case (uint16_t) &OCR5CL:
					TCCR5A |= MASK(COM5C1);
					break;
				#endif
			}
#endif
		}

		#ifdef	HEATER_SANITY_CHECK
			// 0 is a "sane" temperature when we're trying to cool down
			heaters_runtime[i].sane_temperature = 0;
		#endif

		#ifndef BANG_BANG
			// read factors from eeprom
			heaters_pid[i].p_factor = eeprom_read_dword((uint32_t *) &EE_factors[i].EE_p_factor);
			heaters_pid[i].i_factor = eeprom_read_dword((uint32_t *) &EE_factors[i].EE_i_factor);
			heaters_pid[i].d_factor = eeprom_read_dword((uint32_t *) &EE_factors[i].EE_d_factor);
			heaters_pid[i].i_limit = eeprom_read_word((uint16_t *) &EE_factors[i].EE_i_limit);

// 			if ((heaters_pid[i].p_factor == 0) && (heaters_pid[i].i_factor == 0) && (heaters_pid[i].d_factor == 0) && (heaters_pid[i].i_limit == 0)) {
			if (crc_block(&heaters_pid[i].p_factor, 14) != eeprom_read_word((uint16_t *) &EE_factors[i].crc)) {
				heaters_pid[i].p_factor = DEFAULT_P;
				heaters_pid[i].i_factor = DEFAULT_I;
				heaters_pid[i].d_factor = DEFAULT_D;
				heaters_pid[i].i_limit = DEFAULT_I_LIMIT;
			}
		#endif /* BANG_BANG */
	}
}

/// \brief Write PID factors to eeprom
void heater_save_settings() {
	#ifndef BANG_BANG
		heater_t i;
		for (i = 0; i < NUM_HEATERS; i++) {
			eeprom_write_dword((uint32_t *) &EE_factors[i].EE_p_factor, heaters_pid[i].p_factor);
			eeprom_write_dword((uint32_t *) &EE_factors[i].EE_i_factor, heaters_pid[i].i_factor);
			eeprom_write_dword((uint32_t *) &EE_factors[i].EE_d_factor, heaters_pid[i].d_factor);
			eeprom_write_word((uint16_t *) &EE_factors[i].EE_i_limit, heaters_pid[i].i_limit);
			eeprom_write_word((uint16_t *) &EE_factors[i].crc, crc_block(&heaters_pid[i].p_factor, 14));
		}
	#endif /* BANG_BANG */
}

/** \brief run heater PID algorithm
	\param h which heater we're running the loop for
	\param t which temp sensor this heater is attached to
	\param current_temp the temperature that the associated temp sensor is reporting
	\param target_temp the temperature we're trying to achieve
*/
void heater_tick(heater_t h, temp_sensor_t t, uint16_t current_temp, uint16_t target_temp) {
	uint8_t		pid_output;

	#ifndef	BANG_BANG
		int16_t		heater_p;
		int16_t		heater_d;
		int16_t		t_error = target_temp - current_temp;
	#endif	/* BANG_BANG */

	if (h >= NUM_HEATERS || t >= NUM_TEMP_SENSORS)
		return;

	if (target_temp == 0) {
		heater_set(h, 0);
		return;
	}

	#ifndef	BANG_BANG
		heaters_runtime[h].temp_history[heaters_runtime[h].temp_history_pointer++] = current_temp;
		heaters_runtime[h].temp_history_pointer &= (TH_COUNT - 1);

		// PID stuff
		// proportional
		heater_p = t_error;

		// integral
		heaters_runtime[h].heater_i += t_error;
		// prevent integrator wind-up
		if (heaters_runtime[h].heater_i > heaters_pid[h].i_limit)
			heaters_runtime[h].heater_i = heaters_pid[h].i_limit;
		else if (heaters_runtime[h].heater_i < -heaters_pid[h].i_limit)
			heaters_runtime[h].heater_i = -heaters_pid[h].i_limit;

		// derivative
		// note: D follows temp rather than error so there's no large derivative when the target changes
		heater_d = heaters_runtime[h].temp_history[heaters_runtime[h].temp_history_pointer] - current_temp;

		// combine factors
		int32_t pid_output_intermed = (
			(
				(((int32_t) heater_p) * heaters_pid[h].p_factor) +
				(((int32_t) heaters_runtime[h].heater_i) * heaters_pid[h].i_factor) +
				(((int32_t) heater_d) * heaters_pid[h].d_factor)
			) / PID_SCALE
		);

		// rebase and limit factors
		if (pid_output_intermed > 255)
			pid_output = 255;
		else if (pid_output_intermed < 0)
			pid_output = 0;
		else
			pid_output = pid_output_intermed & 0xFF;

		#ifdef	DEBUG
		if (DEBUG_PID && (debug_flags & DEBUG_PID))
#if ARCH == arm
			sersendf_P(PSTR("T{E:%d, P:%d * %d = %ld / I:%d * %d = %ld / D:%d * %d = %ld # O: %d = %u}\n"),
#else
			sersendf_P(PSTR("T{E:%d, P:%d * %ld = %ld / I:%d * %ld = %ld / D:%d * %ld = %ld # O: %ld = %u}\n"),
#endif
				   t_error, heater_p, heaters_pid[h].p_factor,
				   (int32_t) heater_p * heaters_pid[h].p_factor / PID_SCALE,
				   heaters_runtime[h].heater_i, heaters_pid[h].i_factor,
				   (int32_t) heaters_runtime[h].heater_i * heaters_pid[h].i_factor / PID_SCALE,
				   heater_d, heaters_pid[h].d_factor,
				   (int32_t) heater_d * heaters_pid[h].d_factor / PID_SCALE, pid_output_intermed,
				   pid_output);
		#endif
	#else
		if (current_temp >= target_temp)
			pid_output = BANG_BANG_OFF;
		else
			pid_output = BANG_BANG_ON;
	#endif

	#ifdef	HEATER_SANITY_CHECK
	// check heater sanity
	// implementation is a moving window with some slow-down to compensate for thermal mass
	if (target_temp > (current_temp + (TEMP_HYSTERESIS*4))) {
		// heating
		if (current_temp > heaters_runtime[h].sane_temperature)
			// hotter than sane- good since we're heating unless too hot
			heaters_runtime[h].sane_temperature = current_temp;
		else {
			if (heaters_runtime[h].sanity_counter < 40)
				heaters_runtime[h].sanity_counter++;
			else {
				heaters_runtime[h].sanity_counter = 0;
				// ratchet up expected temp
				heaters_runtime[h].sane_temperature++;
			}
		}
		// limit to target, so if we overshoot by too much for too long an error is flagged
		if (heaters_runtime[h].sane_temperature > target_temp)
			heaters_runtime[h].sane_temperature = target_temp;
	}
	else if (target_temp < (current_temp - (TEMP_HYSTERESIS*4))) {
		// cooling
		if (current_temp < heaters_runtime[h].sane_temperature)
			// cooler than sane- good since we're cooling
			heaters_runtime[h].sane_temperature = current_temp;
		else {
			if (heaters_runtime[h].sanity_counter < 125)
				heaters_runtime[h].sanity_counter++;
			else {
				heaters_runtime[h].sanity_counter = 0;
				// ratchet down expected temp
				heaters_runtime[h].sane_temperature--;
			}
		}
		// if we're at or below 60 celsius, don't freak out if we can't drop any more.
		if (current_temp <= 240)
			heaters_runtime[h].sane_temperature = current_temp;
		// limit to target, so if we don't cool down for too long an error is flagged
		else if (heaters_runtime[h].sane_temperature < target_temp)
			heaters_runtime[h].sane_temperature = target_temp;
	}
	// we're within HYSTERESIS of our target
	else {
		heaters_runtime[h].sane_temperature = current_temp;
		heaters_runtime[h].sanity_counter = 0;
	}

	// compare where we're at to where we should be
	if (labs((int16_t)(current_temp - heaters_runtime[h].sane_temperature)) > (TEMP_HYSTERESIS*4)) {
		// no change, or change in wrong direction for a long time- heater is broken!
		pid_output = 0;
		sersendf_P(PSTR("!! heater %d or temp sensor %d broken- temp is %d.%dC, target is %d.%dC, didn't reach %d.%dC in %d0 milliseconds\n"), h, t, current_temp >> 2, (current_temp & 3) * 25, target_temp >> 2, (target_temp & 3) * 25, heaters_runtime[h].sane_temperature >> 2, (heaters_runtime[h].sane_temperature & 3) * 25, heaters_runtime[h].sanity_counter);
	}
	#endif /* HEATER_SANITY_CHECK */

	heater_set(h, pid_output);
}

/** \brief manually set PWM output
	\param index the heater we're setting the output for
	\param value the PWM value to write

	anything done by this function is overwritten by heater_tick above if the heater has an associated temp sensor
*/
void heater_set(heater_t index, uint8_t value) {
	if (index >= NUM_HEATERS)
		return;

	heaters_runtime[index].heater_output = value;

	if (heaters[index].heater_pwm) {
		*(heaters[index].heater_pwm) = value;
		#ifdef	DEBUG
#if ARCH == arm
		// TODO: implementation
#else
		if (DEBUG_PID && (debug_flags & DEBUG_PID))
			sersendf_P(PSTR("PWM{%u = %u}\n"), index, OCR0A);
#endif
		#endif
	}
	else {
		if (value >= 8)
			*(heaters[index].heater_port) |= MASK(heaters[index].heater_pin);
		else
			*(heaters[index].heater_port) &= ~MASK(heaters[index].heater_pin);
	}
}

/** \brief turn off all heaters

	for emergency stop
*/
uint8_t heaters_all_off() {
	uint8_t i;
	for (i = 0; i < NUM_HEATERS; i++) {
		if (heaters_runtime[i].heater_output > 0)
			return 0;
	}

	return 255;
}

/** \brief set heater P factor
	\param index heater to change factor for
	\param p scaled P factor
*/
void pid_set_p(heater_t index, int32_t p) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].p_factor = p;
	#endif /* BANG_BANG */
}

/** \brief set heater I factor
	\param index heater to change I factor for
	\param i scaled I factor
*/
void pid_set_i(heater_t index, int32_t i) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].i_factor = i;
	#endif /* BANG_BANG */
}

/** \brief set heater D factor
	\param index heater to change D factor for
	\param d scaled D factor
*/
void pid_set_d(heater_t index, int32_t d) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].d_factor = d;
	#endif /* BANG_BANG */
}

/** \brief set heater I limit
	\param index heater to set I limit for
	\param i_limit scaled I limit
*/
void pid_set_i_limit(heater_t index, int32_t i_limit) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].i_limit = i_limit;
	#endif /* BANG_BANG */
}

#ifndef	EXTRUDER
/** \brief send heater debug info to host
	\param i index of heater to send info for
*/
void heater_print(uint16_t i) {
#if ARCH == arm
	sersendf_P(PSTR("P:%d I:%d D:%d Ilim:%u crc:%u "),
#else
	sersendf_P(PSTR("P:%ld I:%ld D:%ld Ilim:%u crc:%u "),
#endif
		   heaters_pid[i].p_factor, heaters_pid[i].i_factor, heaters_pid[i].d_factor,
		   heaters_pid[i].i_limit, crc_block(&heaters_pid[i].p_factor, 14));
}
#endif

#endif
