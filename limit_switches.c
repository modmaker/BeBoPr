
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <pthread.h>
#include <errno.h>

#include "limit_switches.h"
#include "mendel.h"
#include "bebopr.h"
#include "gpio.h"
#include "debug.h"
#include "beaglebone.h"

/*
 * Limit switch handling
 */
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
  case x_axis:  return (x_max_state == active_state);
  case y_axis:  return (y_max_state == active_state);
  case z_axis:  return (z_max_state == active_state);
  default:      return 0;
  }
}

int limsw_min( axis_e axis)
{
  int active_state = !config_min_limit_switch_is_active_low( axis);
  switch (axis) {
  case x_axis:  return (x_min_state == active_state);
  case y_axis:  return (y_min_state == active_state);
  case z_axis:  return (z_min_state == active_state);
  default:      return 0;
  }
}

typedef struct {
  int   gpio_pin_nr;
  int   active_low;
  int   in_use;
  void  (*alarm_call_back)( int state);
  int   fd;
} limit_switch_struct;

/****************************************************************
 * limit_switch_init
 ****************************************************************/
static int limit_gpios[] = { XMIN_GPIO, XMAX_GPIO, YMIN_GPIO, YMAX_GPIO, ZMIN_GPIO, ZMAX_GPIO };
const int nr_limits = NR_ITEMS( limit_gpios);
static int gpio_fd[ NR_ITEMS( limit_gpios)];

static void limsw_exit( void)
{
  // Gracefully handle exit
  for (int i = 0 ; i < nr_limits ; ++i) {
    if (gpio_fd[ i]) {
      close( gpio_fd[ i]);
      gpio_fd[ i] = -1;
    }
    gpio_write_value_to_pin_file( limit_gpios[ i], "edge", "none");
    if (get_kernel_type() == e_kernel_3_2) {
      gpio_write_int_value_to_file( "unexport", limit_gpios[ i]);
    }
  }
}

static struct pollfd(* fdset)[] = NULL;

static void* limsw_watcher( void* arg)
{
  int i;
  unsigned int gpio[ nr_limits];
  char buf[ 3];
  int len;

  if (debug_flags & DEBUG_LIMSW) {
    printf( "Limit switches watcher thread: opening fds");
  }

  for (i = 0 ; i < nr_limits ; ++i) {
    gpio[ i] = limit_gpios[ i];
    if (get_kernel_type() == e_kernel_3_2) {
      gpio_write_int_value_to_file( "export", gpio[ i]);
      gpio_write_value_to_pin_file( gpio[ i], "direction", "in");
    }
    gpio_write_value_to_pin_file( gpio[ i], "edge", "both");
    gpio_fd[ i] = gpio_open_file( gpio[ i], "value");
    if (gpio_fd[ i] < 0) {
      fprintf( stderr, ", open failed for gpio%d, bailing out\n", gpio[ i]);
      pthread_exit( NULL);
    }
  }
  atexit( limsw_exit);
  for (i = 0 ; i < nr_limits ; ++i) {
    (*fdset)[ i].fd = gpio_fd[ i];
    (*fdset)[ i].events = POLLPRI;
  }
  if (debug_flags & DEBUG_LIMSW) {
    printf( ", start polling on:");
    for (i = 0 ; i < nr_limits ; ++i) {
      printf( " gpio%d", gpio[ i]);
    }
    printf( "\n");
  }

  while (1) {
    const int timeout = -1;     /* no timeout */

    int rc = poll( *fdset, nr_limits, timeout);      
    if (rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      perror( "limsw_watcher: poll() failed, bailing out!");
      break;
    }
    if (rc == 0) {
      // poll timeout, shouldn't happen with 'timeout' == -1 !!!
      fprintf( stderr, "\nlimsw_watcher: poll() unexpected timeout, bailing out!\n");
      break;
    }
    for (i = 0 ; i < nr_limits ; ++i) {
      if ((*fdset)[ i].revents & POLLPRI) {
        int state = 0;
        len = read( (*fdset)[ i].fd, buf, sizeof( buf));
        if (len == 2 && (buf[ 0] == '0' || buf[ 0] == '1')) {
          // Get initial state info
          state = buf[ 0] - '0';
          if (debug_flags & DEBUG_LIMSW) {
            printf( "*** GPIO %2d event, state %d ***\n", gpio[ i], state);
          }
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
  pthread_exit( NULL);
}

static pthread_t worker;

int limsw_init( void)
{
  fdset = calloc( nr_limits, sizeof( struct pollfd));

  if (mendel_thread_create( "limit_switches", &worker, NULL, &limsw_watcher, NULL) != 0) {
    return -1;
  }
  struct sched_param param = {
    .sched_priority = LIMSW_PRIO
  };
  pthread_setschedparam( worker, LIMSW_SCHED, &param);

  return 0;
}
