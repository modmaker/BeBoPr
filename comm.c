
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

#define LL_DEBUG 0	/* low level, very verbose debugging of comm code */

/*
 * Simple keep alive implementation needed until I find out
 * how to keep the socat connection alive otherwise
 */

static int fd_stdin;
static int fd_stdout;
static int alt_stdin;
static int alt_stdout;

typedef enum {
  e_stdout_readside = 0,
  e_stdout_writeside,
  e_stdin_readside,
  e_stdin_writeside,
} fds_index;

static void* comm_thread( void* arg)
{
  struct pollfd fds[ 4];
  unsigned int cnt_in = 0;
  unsigned int cnt_out = 0;

  if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
    printf( "Socket connection keep-alive thread: started.");
  }
  // read side of stdout pipe
  fds[ e_stdout_readside].fd = alt_stdout;
  fds[ e_stdout_readside].events = POLLIN;
  // write side of system stdout
  fds[ e_stdout_writeside].fd = fd_stdout;
  fds[ e_stdout_writeside].events = 0;
  // read side of system stdin
  fds[ e_stdin_readside].fd = fd_stdin;
  fds[ e_stdin_readside].events = POLLIN;
  // write side of stdin pipe
  fds[ e_stdin_writeside].fd = alt_stdin;
  fds[ e_stdin_writeside].events = 0;

  const int keep_alive_timeout = 10000; /* 10 sec. in ms */
  const int timeout = 100; /* ms */
  int output_pending = 0;
  int input_pending = 0;
  int eof_on_input = 0;
  char pending_input;
  char pending_output;
  int prescaler = 0;
  char keep_alive_char = config_keep_alive_char();
  /*
   * The data from the stdout pipe does not become available until
   * stdout is flushed. So the timer is set to a short cycle that
   * flushes stdout with each timeout.
   */
  while (1) {
    /* wait for event */
    int rc = poll( fds, (eof_on_input) ? 2 : 4, timeout);
#if LL_DEBUG
    fprintf( stderr, "<poll result = %d\n", rc);
    for (int i = 0 ; i < NR_ITEMS( fds) ; ++i) {
      fprintf( stderr, "<poll result for fd: %d, events = 0x%08x, revents = 0x%08x\n", fds[ i].fd, fds[ i].events, fds[ i].revents);
    }
#endif
    if (rc < 0 && errno != EINTR) {
      perror( "comm_thread: poll() failed, bailing out!");
      break;
    } else if (rc == 0 || (rc < 0 && errno == EINTR)) {
      // timeout, send dummy character to keep connection alive
      if (++prescaler > keep_alive_timeout / timeout) {
        printf( "%c", keep_alive_char);
        if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
          fprintf( stderr, "<KEEP ALIVE SENT>");
	}
        prescaler = 0;
      }
#if LL_DEBUG
      fprintf( stderr, "<STDOUT FLUSH>");
#endif
      fflush( stdout);
    } else {
      int events;
      if (!eof_on_input) {
      /****************************************
       ***  I N P U T  -  R E A D  S I D E  ***
       ****************************************/
        events = fds[ e_stdin_readside].revents;
        if (events & POLLIN) {
          // stdin has input for stdin pipe
          if (!input_pending) {
            int cnt = read( fd_stdin, &pending_input, 1);
            if (cnt == 1) {
#if LL_DEBUG
              fprintf( stderr, "stdin - processing '%c'\n", pending_input);
#endif
              fds[ e_stdin_readside].events = 0;
              fds[ e_stdin_writeside].events = POLLOUT;
              input_pending = 1;
              ++cnt_in;
            } else if (cnt == 0) {
              // EOF
              fprintf( stderr, "comm_thread: EOF on STDIN, closing input pipe.\n");
              close( alt_stdin); 
              eof_on_input = 1;
            }
          }
        } else if (events & POLLHUP) {
          /* Ignore POLLHUP if we're not accepting input to prevent
             characters from being dropped unintentionally */
          if (fds[ e_stdin_readside].events & POLLIN) {
            // write end was closed (EOF)
            if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
              fprintf( stderr, "comm_thread: HUP on STDIN, closing input pipe after forwarding %u of %u chars.\n",
		      cnt_out, cnt_in);
            }
            close( alt_stdin); 
            eof_on_input = 1;
          }
        } else if (events) {
          fprintf( stderr, "Poll on fd_stdin returns 0x%08x\n", events);
        }
      /******************************************
       ***  I N P U T  -  W R I T E  S I D E  ***
       ******************************************/
        events = fds[ e_stdin_writeside].revents;
        if (events & POLLOUT) {
          if (input_pending) {
            int cnt = write( alt_stdin, &pending_input, 1);
            if (cnt == 1) {
              ++cnt_out;
              input_pending = 0;
              fds[ e_stdin_readside].events = POLLIN;
              fds[ e_stdin_writeside].events = 0;
            }
          }
        } else if (events) {
          fprintf( stderr, "Poll on alt_stdin returns 0x%08x\n", events);
        }
      }
      /******************************************
       ***  O U T P U T  -  R E A D  S I D E  ***
       ******************************************/
      events = fds[ e_stdout_readside].revents;
      if (events & POLLIN) {
        /* stdout pipe has output for stdout (our program did write to stdout) */
        if (!output_pending) {
          int cnt = read( alt_stdout, &pending_output, 1);
          if (cnt == 1) {
            output_pending = 1;
            fds[ e_stdout_writeside].events = POLLOUT;
            fds[ e_stdout_readside].events = 0;
          }
        }
      } else if (events) {
        fprintf( stderr, "Poll on alt_stdout returns 0x%08x\n", events);
      }
      /********************************************
       ***  O U T P U T  -  W R I T E  S I D E  ***
       ********************************************/
      events = fds[ e_stdout_writeside].revents;
      if (events & POLLOUT) {
        if (output_pending) {
          int cnt = write( fd_stdout, &pending_output, 1);
          if (cnt == 1) {
            output_pending = 0;
            fds[ e_stdout_writeside].events = 0;
            fds[ e_stdout_readside].events = POLLIN;
            prescaler = 0;	// output acts as keep-alive!
          }
        }
      } else if (events) {
        fprintf( stderr, "Poll on fd_stdout returns 0x%08x\n", events);
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
  fd_stdin  = fcntl( 0, F_DUPFD, 0);
  if (fd_stdin == -1) {
    perror( "fcntl( 0, F_DUPFD, 0) failed");
  }
  fd_stdout = fcntl( 1, F_DUPFD, 0);
  if (fd_stdout == -1) {
    perror( "fcntl( 1, F_DUPFD, 0) failed");
  }
  // Note on debug output: Because we're messing with stdout,
  // all debug output for 'comm_init' is sent to stderr instead of stdout !
  if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
    fprintf( stderr, "fd_stdin = %d, fd_stdout = %d\n", fd_stdin, fd_stdout);
  }

  // create the output pipe with all new fds
  // replace application's stdout descriptor (1) by write side of output pipe
  result = pipe2( fds, O_NONBLOCK);
  if (result < 0) {
    perror( "output pipe creation failed");
    return -1;
  }
  close( 1);
  result = fcntl( fds[ 1], F_DUPFD, 0);
  if (result == -1) {
    perror( "fcntl( fds[ 1], F_DUPFD, 0) failed");
  }
  close( fds[ 1]);
  fds[ 1] = result;
  if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
    fprintf( stderr, "output pipe: output/read side fd = %d, input/write side fd = %d\n", fds[ 0], fds[ 1]);
  }
  alt_stdout = fds[ 0];

  // create the input pipe with all new fds
  // replace application's stdin descriptor (0) by read side of input pipe
  close( 0);
  result = pipe2( fds, 0);
  if (result < 0) {
    perror( "input pipe creation failed");
    return -1;
  }
  if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
    fprintf( stderr, "input  pipe: output/read side fd = %d, input/write side fd = %d\n", fds[ 0], fds[ 1]);
  }
  alt_stdin = fds[ 1];

  if (DEBUG_COMM && (debug_flags & DEBUG_COMM)) {
    fprintf( stderr, "alt_stdin = %d, alt_stdout = %d\n", alt_stdin, alt_stdout);
  }

  if (mendel_thread_create( "comm", &worker, NULL, &comm_thread, NULL) != 0) {
    return -1;
  }
  struct sched_param param = {
    .sched_priority = COMM_PRIO
  };
  pthread_setschedparam( worker, COMM_SCHED, &param);

  return 0;
}
