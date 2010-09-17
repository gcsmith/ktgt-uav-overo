/*******************************************************************************
#                                                                              #
#      MJPG-streamer allows to stream JPG frames from an input-plugin          #
#      to several output plugins                                               #
#                                                                              #
#      Copyright (C) 2007 Tom Stöveken                                         #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; version 2 of the License.                      #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <linux/videodev.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <syslog.h>

#include "utils.h"
#include "mjpg_streamer.h"

/* globals */
static globals global;

/******************************************************************************
Description.: pressing CTRL+C sends signals to this process instead of just
              killing it plugins can tidily shutdown and free allocated
              ressources. The function prototype is defined by the system,
              because it is a callback function.
Input Value.: sig tells us which signal was received
Return Value: -
******************************************************************************/
void mjpg_signal_handler(int sig)
{
  //int i;

  /* signal "stop" to threads */
  global.stop = 1;
  usleep(1000*1000);

  /* clean up threads */
  input_stop();
  output_stop(0);
  usleep(1000*1000);

  pthread_cond_destroy(&global.db_update);
  pthread_mutex_destroy(&global.db);
}

// ----------------------------------------------------------------------------
void mjpg_streamer_stop()
{
    mjpg_signal_handler(SIGINT);
}

/******************************************************************************
Description.:
Input Value.:
Return Value:
******************************************************************************/
int control(int command, char *details) {

  switch(command) {
    case CONTROL_CMD_RECONFIGURE_INPUT:
      printf("will reload input plugin: %s\n", details);
      break;
    default:
      break;
  }
  return 0;
}


/******************************************************************************
Description.:
Input Value.:
Return Value:
******************************************************************************/
int mjpg_streamer_start(char *input, char *device, char *output)
{
  fprintf(stderr, "mjpg_streamer started\n");
  size_t tmp=0;

  global.outcnt = 1;

  global.control = control;

  // This may be useful... ?
#if 0 
  /* fork to the background */
  if ( daemon ) {
    LOG("enabling daemon mode");
    daemon_mode();
  }
#endif

  /* initialise the global variables */
  global.stop      = 0;
  global.buf       = NULL;
  global.size      = 0;
  global.in.plugin = NULL;

  /* this mutex and the conditional variable are used to synchronize access to the global picture buffer */
  if( pthread_mutex_init(&global.db, NULL) != 0 ) {
    fprintf(stderr, "Unable to initialize mutex variablei.\nExiting prematurely.\n");
    return -1;
  }
  if( pthread_cond_init(&global.db_update, NULL) != 0 ) {
    fprintf(stderr, "Unable to initialize condition variable.\nExiting prematurely.\n");
      return -1;
  }

  /* ignore SIGPIPE (send by OS if transmitting to closed TCP sockets) */
  signal(SIGPIPE, SIG_IGN);

  /* register signal handler for <CTRL>+C in order to clean up */
  if (signal(SIGINT, mjpg_signal_handler) == SIG_ERR) {
    fprintf(stderr, "Unable to register signal handler.\nExiting prematurely.\n");
      return -1;
  }

  /* open input plugin */
  tmp = (size_t)(strchr(input, ' ')-input);
  global.in.plugin = (tmp > 0)?strndup(input, tmp):strdup(input);

#if 0
  global.in.handle = dlopen(global.in.plugin, RTLD_LAZY);
  if ( !global.in.handle ) {
    return -1;
  }
  global.in.init = dlsym(global.in.handle, "input_init");
  if ( global.in.init == NULL ) {
    return -1;
  }
  global.in.stop = dlsym(global.in.handle, "input_stop");
  if ( global.in.stop == NULL ) {
    return -1;
  }
  global.in.run = dlsym(global.in.handle, "input_run");
  if ( global.in.run == NULL ) {
    return -1;
  }


  /* try to find optional command */
  global.in.cmd = dlsym(global.in.handle, "input_cmd");
  global.in.cmd_new = dlsym(global.in.handle, "input_cmd_new");
#endif

  global.in.param.parameter_string = strchr(input, ' ');
  global.in.param.global = &global;

  if ( /*global.in.init*/input_init(&global.in.param, device) ) {
      fprintf(stderr, "Error occurred in input_init().\n");
    return -1;
  }

  /* open output plugin */
  tmp = (size_t)(strchr(output, ' ')-output);
  global.out[0].plugin = (tmp > 0)?strndup(output, tmp):strdup(output);

#if 0
  global.out[0].handle = dlopen(global.out[0].plugin, RTLD_LAZY);
  if ( !global.out[0].handle ) {
    return -1;
  }
  global.out[0].init = dlsym(global.out[0].handle, "output_init");
  if ( global.out[0].init == NULL ) {
    return -1;
  }
  global.out[0].stop = dlsym(global.out[0].handle, "output_stop");
  if ( global.out[0].stop == NULL ) {
    return -1;
  }
  global.out[0].run = dlsym(global.out[0].handle, "output_run");
  if ( global.out[0].run == NULL ) {
    return -1
  }
  /* try to find optional command */
  global.out[0].cmd = dlsym(global.out[0].handle, "output_cmd");
#endif

  global.out[0].param.parameter_string = strchr(output, ' ');
  global.out[0].param.global = &global;
  global.out[0].param.id = 0;

  // Call specific input plugin fuctions
  if ( /*global.out[0].init*/output_init(&global.out[0].param) ) {
      fprintf(stderr, "Error occurred in output_init().\n");
    return -1;
  }

  /* start to read the input, push pictures into global buffer */
  if ( /*global.in.run()*/input_run() ) {
      fprintf(stderr, "Error occurred in input_run().\n");
    return -1;
  }

  //global.out[0].run(global.out[0].param.id);
  output_run(global.out[0].param.id);

  /* wait for signals */
  //pause();

  return 0;
}

