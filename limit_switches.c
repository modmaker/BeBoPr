
#include	<sys/types.h>
#include	<sys/stat.h>
#include	<fcntl.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>
#include	<unistd.h>
#include	<string.h>
#include	<poll.h>
#include	<pthread.h>

#include	"limit_switches.h"
#include "mendel.h"
#include "bebopr.h"
#include "gpio.h"

/*
 * Limit switch handling
 */
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT -1 /* no timeout */
#define MAX_BUF 64

// one may assume these values are always either 0 or 1 !
static int x_min_state;
static int x_max_state;
static int y_min_state;
static int y_max_state;
static int z_min_state;
static int z_max_state;

int limsw_max( axis_e axis)
{
  int active_state = !config_max_limit_switch_is_active_low( axis);
  switch (axis) {
  case x_axis:	return (x_max_state == active_state);
  case y_axis:	return (y_max_state == active_state);
  case z_axis:	return (z_max_state == active_state);
  default:	return 0;
  }
}

int limsw_min( axis_e axis)
{
  int active_state = !config_max_limit_switch_is_active_low( axis);
  switch (axis) {
  case x_axis:	return (x_min_state == active_state);
  case y_axis:	return (y_min_state == active_state);
  case z_axis:	return (z_min_state == active_state);
  default:	return 0;
  }
}

typedef struct {
  int	gpio_pin_nr;
  int	active_low;
  int	in_use;
  void	(*alarm_call_back)( int state);
  int	fd;
} limit_switch_struct;

/****************************************************************
 * limit_switch_init
 ****************************************************************/
static struct pollfd(* fdset)[] = NULL;

#define NR_ITEMS( x) (sizeof( (x)) / sizeof( *(x)))

#define XMIN_GPIO 10
#define XMAX_GPIO 11
#define YMIN_GPIO  8
#define YMAX_GPIO  9
#define ZMIN_GPIO 79
#define ZMAX_GPIO 78

static int limit_gpios[] = { XMIN_GPIO, XMAX_GPIO, YMIN_GPIO, YMAX_GPIO, ZMIN_GPIO, ZMAX_GPIO };
const int nr_limits = NR_ITEMS( limit_gpios);

static void* limsw_watcher( void* arg)
{
  int i;
  unsigned int gpio[ nr_limits];
  int gpio_fd[ nr_limits];
  char buf[ 3];
  int len;

  fprintf( stderr, "Limit switches watcher thread: opening fds");

  for (i = 0 ; i < nr_limits ; ++i) {
    gpio[ i]  = limit_gpios[ i];
    gpio_write_int_value_to_file( "export", gpio[ i]);
    gpio_write_value_to_pin_file( gpio[ i], "direction", "in");
    gpio_write_value_to_pin_file( gpio[ i], "edge", "both");
    gpio_fd[ i] = gpio_open_file( gpio[ i], "value");
    if (gpio_fd[ i] < 0) {
      fprintf( stderr, ", open failed for gpio%d\n", gpio[ i]);
      pthread_exit( NULL);
    }
  }

  for (i = 0 ; i < nr_limits ; ++i) {
    (*fdset)[ i].fd = gpio_fd[ i];
    (*fdset)[ i].events = POLLPRI;
  }
  fprintf( stderr, ", start polling on:");
  for (i = 0 ; i < nr_limits ; ++i) {
    fprintf( stderr, " gpio%d", gpio[ i]);
  }
  fprintf( stderr, "\n");

  while (1) {
    const int timeout = -1;	/* no timeout */

    int rc = poll( *fdset, nr_limits, timeout);      
    if (rc < 0) {
      fprintf( stderr, "\nlimsw_watcher: poll() failed!\n");
      break;
    }
    if (rc == 0) {
      fprintf( stderr, "OOPS");
    }
    for (i = 0 ; i < nr_limits ; ++i) {
      if ((*fdset)[ i].revents & POLLPRI) {
	int state;
	len = read( (*fdset)[ i].fd, buf, sizeof( buf));
	if (len == 2 && (buf[ 0] == '0' || buf[ 0] == '1')) {
	  // Get initial state info
	  state = buf[ 0] - '0';
	  fprintf( stderr, "*** GPIO %2d event, state %d ***\n", gpio[ i], state);
	} else {
	  fprintf( stderr, "\n*** GPIO %2d unexpected event, BUG??? ***\n", gpio[ i]);
	}
	switch (gpio[ i]) {
	case XMIN_GPIO: x_min_state = state; break;
	case XMAX_GPIO: x_max_state = state; break;
	case YMIN_GPIO: y_min_state = state; break;
	case YMAX_GPIO: y_max_state = state; break;
	case ZMIN_GPIO: z_min_state = state; break;
	case ZMAX_GPIO: z_max_state = state; break;
	}
	lseek( (*fdset)[ i].fd, 0, SEEK_SET);
      }
    }
  }
  // Gracefully handle exit
  for (i = 0 ; i < nr_limits ; ++i) {
    close( gpio_fd[ i]);
  }
  pthread_exit( NULL);
}

static pthread_t worker;

int limsw_init( void)
{
  int result;

  fdset = calloc( nr_limits, sizeof( struct pollfd));

#if 0
  pthread_attr_t attr;
  pthread_attr_init( &attr);
  pthread_attr_setschedpolicy( &attr, SCHED_FIFO);

  result = mendel_thread_create( "limit_switches", &worker, &attr, &limsw_watcher, NULL);
#else
  result = mendel_thread_create( "limit_switches", &worker, NULL, &limsw_watcher, NULL);
#endif
  if (result != 0) {
    exit( EXIT_FAILURE);
  }
  struct sched_param param = {
    .sched_priority = 74
  };
  //  pthread_setschedparam( worker, SCHED_FIFO, &param);
  pthread_setschedparam( worker, SCHED_RR, &param);

#if 0
  pthread_setschedprio( worker, 5);

  int policy = SCHED_RR;
  int priority = sched_get_priority_max( policy);

  struct sched_param sp = { 
    .sched_priority = priority
  };
  // Set realtime process scheduling using the FIFO scheduler
  // with absolute priority at maximum
  if (pthread_setthreadparam( worker, policy, &sp) < 0) {
  }
#endif
  return 0;
}
