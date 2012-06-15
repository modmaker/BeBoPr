
#define _GNU_SOURCE     /* for pipe2 */

#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <pthread.h>

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


static void* comm_thread( void* arg)
{
  const int timeout = 10000; /* ms */

  if (debug_flags & DEBUG_LIMSW) {
    printf( "Socket connection keep-alive thread: started.");
  }

  // read from system stdin
  fds[ 0].fd = fd_stdin;
  fds[ 0].events = POLLIN | POLLPRI;
  // write to system stdout
  fds[ 1].fd = fd_stdout;
  fds[ 1].events = POLLERR | POLLHUP;
  // read from stdout pipe
  fds[ 2].fd = alt_stdout;
  fds[ 2].events = POLLIN | POLLPRI;
  // write to stdin pipe
  fds[ 3].fd = alt_stdin;
  fds[ 3].events = POLLERR | POLLHUP;

  while (1) {
    int rc = poll( fds, NR_ITEMS( fds), timeout);      
    if (rc < 0) {
      perror( "comm_thread: poll() failed, bailing out!");
      break;
    } else if (rc == 0) {
      // timeout, send dummy character to keep connection alive
      printf( "%c", 255);
      fflush( stdout);
      fprintf( stderr, "<KEEP ALIVE SENT>\n");
    } else {
      // input pending
      for (int i = 0 ; i < NR_ITEMS( fds) ; ++i) {
        int events = fds[ i].revents;
        if (events & POLLHUP) {
          if (i == 0) {
            fprintf( stderr, "comm_thread: lost connection on STDIN, closing down.\n");
            close( alt_stdout);
            close( alt_stdin);
            pthread_exit( NULL);
          } else {
            fprintf( stderr, "Poll on fd %d returns POLLHUP\n", i);
          }
        } else if (events & POLLERR) {
          fprintf( stderr, "Poll on fd %d returns POLLERR\n", i);
        } else if (events & POLLNVAL) {
          fprintf( stderr, "Poll on fd %d returns POLLNVAL\n", i);
        } else if (events & POLLPRI) {
          fprintf( stderr, "Poll on fd %d returns POLLPRI\n", i);
        } else if (events & POLLIN) {
          if (i == 2) {
            // stdout pipe has output for stdout
            char s[ 1];
            read(  fds[ 2].fd, s, sizeof( s));
            write( fds[ 1].fd, s, sizeof( s));
          } else if (i == 0) {
            // stdin has input for stdin pipe
            char s[ 1];
            read(  fds[ 0].fd, s, sizeof( s));
            write( fds[ 3].fd, s, sizeof( s));
          } else {
            fprintf( stderr, "Poll on fd %d returns POLLIN\n", i);
          }
        } else if (events & POLLOUT) {
          fprintf( stderr, "Poll on fd %d returns POLLOUT\n", i);
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
  result = pipe2( fds, 0);
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
    perror( "pipe_out creation failed");
    return -1;
  }
  fprintf( stderr, "stdin pipe created: output side fd = %d, input side fd = %d\n", fds[ 0], fds[ 1]);
  alt_stdin = fds[ 1];
  fprintf( stderr, "alt_stdin = %d\n", alt_stdout);

  result = mendel_thread_create( "comm", &worker, NULL, &comm_thread, NULL);
  if (result != 0) {
    exit( EXIT_FAILURE);
  }
  struct sched_param param = {
    .sched_priority = 74
  };
  // SCHED_RR seems to work for now, otherwise use SCHED_FIFO
  pthread_setschedparam( worker, SCHED_RR, &param);

  return 0;
}
