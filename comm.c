
#define _GNU_SOURCE     /* for pipe2 */

#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <pthread.h>
#include <errno.h>

#include "comm.h"
#include "mendel.h"
#include "bebopr.h"
#include "debug.h"
#include "beaglebone.h"

/*
 * Simple keep alive implementation needed until I find out
 * how to keep the socat connection alive otherwise
 */

static struct pollfd fds[ 4];

static int fd_stdin;
static int fd_stdout;
static int alt_stdin;
static int alt_stdout;

typedef enum {
  e_stdout_writeside = 0,
  e_stdout_readside,
  e_stdin_writeside,
  e_stdin_readside,
} fds_index;

static void* comm_thread( void* arg)
{
  if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
    printf( "Socket connection keep-alive thread: started.");
  }
  // read side of system stdin
  fds[ e_stdin_readside].fd = fd_stdin;
  fds[ e_stdin_readside].events = POLLIN | POLLPRI;
  // write side of system stdout
  fds[ e_stdout_writeside].fd = fd_stdout;
  fds[ e_stdout_writeside].events = POLLERR | POLLHUP;
  // read side of stdout pipe
  fds[ e_stdout_readside].fd = alt_stdout;
  fds[ e_stdout_readside].events = POLLIN | POLLPRI;
  // write side of stdin pipe
  fds[ e_stdin_writeside].fd = alt_stdin;
  fds[ e_stdin_writeside].events = POLLERR | POLLHUP;

  const int timeout = 100; /* ms */
  int output_pending = 0;
  int input_pending = 0;
  char pending_input;
  char pending_output;
  int active_fds = NR_ITEMS( fds);
  /*
   * The data from the stdout pipe does not become available until
   * stdout is flushed. So the timer is set to a short cycle that
   * flushes stdout with each timeout.
   */
  while (1) {
// handle output
    if (output_pending) {
      int cnt = write( fd_stdout, &pending_output, 1);
      if (cnt == 1) {
        output_pending = 0;
      } else {
        fds[ e_stdout_readside].events = 0;     // block more input
      }
    }
    if (!output_pending) {
      fds[ e_stdout_readside].events = POLLIN | POLLPRI;
    }
// handle input
    if (input_pending) {
      int cnt = write( alt_stdin, &pending_input, 1);
      if (cnt == 1) {
        input_pending = 0;
      } else {
        fds[ e_stdin_readside].events = 0;      // block more input
      }
    }
    if (!input_pending) {
      fds[ e_stdin_readside].events = POLLIN | POLLPRI;
    }
// wait for event
    int rc = poll( fds, active_fds, timeout);      
    if (rc < 0 && errno != EINTR) {
      perror( "comm_thread: poll() failed, bailing out!");
      break;
    } else if (rc == 0 || (rc < 0 && errno == EINTR)) {
      // timeout, send dummy character to keep connection alive
      static int prescaler = 0;
      if (++prescaler > 100) {
        printf( "%c", 255);
        if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
          fprintf( stderr, "<KEEP ALIVE SENT>\n");
        }
        prescaler = 0;
      }
      fflush( stdout);
    } else {
      for (int i = 0 ; i < active_fds ; ++i) {
        int events = fds[ i].revents;
        int fd = fds[ i].fd;
        if (events & POLLHUP) {
          if (fd == fd_stdin) {
            if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
              fprintf( stderr, "comm_thread: lost connection on STDIN, closing input pipe.\n");
            }
            close( alt_stdin);
	    active_fds = 2;
#if 0 
            input_eof = 1;
            close( alt_stdout);
            // FIXME: do not terminate until all input is processed !
            pthread_exit( NULL);
#endif
          } else {
            fprintf( stderr, "Poll on fd %d returns POLLHUP\n", fd);
          }
        } else if (events & POLLERR) {
          fprintf( stderr, "Poll on fd %d returns POLLERR\n", fd);
        } else if (events & POLLNVAL) {
          fprintf( stderr, "Poll on fd %d returns POLLNVAL\n", fd);
        } else if (events & POLLPRI) {
          fprintf( stderr, "Poll on fd %d returns POLLPRI\n", fd);
        } else if (events & POLLIN) {
          if (fd == alt_stdout) {
            // stdout pipe has output for stdout
            if (!output_pending) {
              int cnt = read( alt_stdout, &pending_output, 1);
              if (cnt == 1) {
                output_pending = 1;
              }
            }
          } else if (fd == fd_stdin) {
            // stdin has input for stdin pipe
            if (!input_pending) {
              int cnt = read( fd_stdin, &pending_input, 1);
              if (cnt == 1) {
                input_pending = 1;
              }
            }
          } else {
            fprintf( stderr, "Poll on fd %d returns POLLIN\n", fd);
          }
        } else if (events & POLLOUT) {
          fprintf( stderr, "Poll on fd %d returns POLLOUT\n", fd);
        }
      }
    }
  }
  pthread_exit( NULL);
}

static pthread_t worker;

/*
 *  Fixup file descriptors so that all stdin and stdout of the application
 *  is piped to alt_stdout (for stdout) and alt_stdin (for stdin).
 *  Now we can watch and modify the datastream from and to the application.
 *  That allows us to insert keep-alive characters to keep socat happy!
 */
int comm_init( void)
{
  int result;
  int fds[ 2];

  // keep a copy of the program's original fds for stdin and stdout
  fd_stdin  = fcntl( 0, F_DUPFD);
  fd_stdout = fcntl( 1, F_DUPFD);
  fprintf( stderr, "fd_stdin = %d, fd_stdout = %d\n", fd_stdin, fd_stdout);

  // create the stdout pipe with all new fds
  result = pipe2( fds, O_NONBLOCK);
  if (result < 0) {
    perror( "stdout pipe creation failed");
    return -1;
  }
  fprintf( stderr, "stdout pipe created: output side fd = %d, input side fd = %d\n", fds[ 0], fds[ 1]);
  // close application's stdout descriptor (1)
  close( 1);
  // connect input/write side of stdout pipe to application's stdout descriptor (1)
  result = fcntl( fds[ 1], F_DUPFD);
  close( fds[ 1]);
  fds[ 1] = result;
  fprintf( stderr, "stdout pipe fixed:   output side fd = %d, input side fd = %d\n", fds[ 0], fds[ 1]);
  alt_stdout = fds[ 0];
  fprintf( stderr, "alt_stdout = %d\n", alt_stdout);

  close( 0);
  result = pipe2( fds, 0);
  if (result < 0) {
    perror( "stdin pipe creation failed");
    return -1;
  }
  fprintf( stderr, "stdin pipe created: output side fd = %d, input side fd = %d\n", fds[ 0], fds[ 1]);
  alt_stdin = fds[ 1];
  fprintf( stderr, "alt_stdin = %d\n", alt_stdin);

  if (mendel_thread_create( "comm", &worker, NULL, &comm_thread, NULL) != 0) {
    return -1;
  }
  struct sched_param param = {
    .sched_priority = COMM_PRIO
  };
  pthread_setschedparam( worker, COMM_SCHED, &param);

  return 0;
}
