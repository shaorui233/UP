/*!
 * Segfault handler to print stack trace on crash.
 */

#include "include/Utilities/SegfaultHandler.h"
#include <cstdio>
#include <execinfo.h>
#include <csignal>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <include/Utilities/Utilities_print.h>

static char* error_message_buffer;

static void segfault_handler(int sig) {

  void* stack_frames[200];
  int size = backtrace(stack_frames, 200);
  fprintf_color(PrintColor::Red, stderr, "CRASH: Caught %d (%s)\n",
      sig, strsignal(sig));
  backtrace_symbols_fd(stack_frames, size, STDERR_FILENO);

  fflush(stderr);
  fflush(stdout);

  if(error_message_buffer)
    strcpy(error_message_buffer, "Segfault!\nCheck the robot controller output for more information.");
  exit(1);
}



void install_segfault_handler(char* error_message) {
  signal(SIGSEGV, segfault_handler);
  error_message_buffer = error_message;
}