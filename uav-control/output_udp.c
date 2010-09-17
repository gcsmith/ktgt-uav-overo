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

/*
  This output plugin is based on code from output_file.c
  Writen by Dimitrios Zachariadis
  Version 0.1, May 2010

  It provides a mechanism to take snapshots with a trigger from a UDP packet.
  The UDP msg contains the path for the snapshot jpeg file
  It echoes the message received back to the sender, after taking the snapshot
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <linux/videodev.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <resolv.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <fcntl.h>
#include <time.h>
#include <syslog.h>

#include <dirent.h>

#include "utils.h"
#include "mjpg_streamer.h"

#define OUTPUT_PLUGIN_NAME "UDP output plugin"
#define MAX_ARGUMENTS 32

static pthread_t worker;
static globals *pglobal;
static int fd, delay, max_frame_size;
static char *folder = "/tmp";
static unsigned char *frame=NULL;
static char *command = NULL;

// UDP port
static int port = 2010;

/******************************************************************************
Description.: clean up allocated ressources
Input Value.: unused argument
Return Value: -
******************************************************************************/
void worker_cleanup(void *arg) {
  static unsigned char first_run=1;

  if ( !first_run ) {
    DBG("already cleaned up ressources\n");
    return;
  }

  first_run = 0;
  OPRINT("cleaning up ressources allocated by worker thread\n");

  if (frame != NULL) {
    free(frame);
  }
  close(fd);
}

/******************************************************************************
Description.: this is the main worker thread
              it loops forever, grabs a fresh frame and stores it to file
Input Value.: 
Return Value: 
******************************************************************************/
void *worker_thread( void *arg ) {
  int ok = 1, frame_size=0; //, rc = 0;
  //char buffer1[1024] = {0};
  unsigned char *tmp_framebuffer=NULL;

  /* set cleanup handler to cleanup allocated ressources */
  pthread_cleanup_push(worker_cleanup, NULL);
  
  // set UDP server data structures ---------------------------
  if (port <= 0) {
	OPRINT("a valid UDP port must be provided\n");
	return NULL;
  }
  struct sockaddr_in addr;
  int sd;
  int bytes;
  socklen_t addr_len=sizeof(addr);
  char udpbuffer[1024] = {0};
  sd = socket(PF_INET, SOCK_DGRAM, 0);
  bzero(&addr, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);
  if ( bind(sd, (struct sockaddr*)&addr, sizeof(addr)) != 0 )
	perror("bind");
  // -----------------------------------------------------------
    
  fprintf(stderr, "Waiting for client's message.\n");
  // UDP receive ---------------------------------------------
  memset(udpbuffer, 0, sizeof(udpbuffer));
  bytes = recvfrom(sd, udpbuffer, sizeof(udpbuffer), 0, (struct sockaddr*)&addr, &addr_len);
  // ---------------------------------------------------------
  
  while ( ok >= 0 && !pglobal->stop ) {
    fprintf(stderr, "waiting for fresh frame on port %d\n", port);
    pthread_cond_wait(&pglobal->db_update, &pglobal->db);
    fprintf(stderr, "got frame\n");
    /* read buffer */
    frame_size = pglobal->size;

    /* check if buffer for frame is large enough, increase it if necessary */
    if ( frame_size > max_frame_size ) {
      DBG("increasing buffer size to %d\n", frame_size);

      max_frame_size = frame_size+(1<<16);
      if ( (tmp_framebuffer = realloc(frame, max_frame_size)) == NULL ) {
        pthread_mutex_unlock( &pglobal->db );
        LOG("not enough memory\n");
        return NULL;
      }

      frame = tmp_framebuffer;
    }

    /* copy frame to our local buffer now */
    //memcpy(frame, pglobal->buf, frame_size);

    /* allow others to access the global buffer again */
    pthread_mutex_unlock( &pglobal->db );

    fprintf(stderr, "ready to send datagram\n");
    // send back client's message that came in udpbuffer
    // sendto(sd, udpbuffer, bytes, 0, (struct sockaddr*)&addr, sizeof(addr));
    sendto(sd, frame, frame_size, 0, (struct sockaddr*)&addr, sizeof(addr));
    fprintf(stderr, "datagram sent: %d bytes\n", frame_size);
  }

  // close UDP port
  if (port > 0)
	close(sd);
  fprintf(stderr, "port closed\n");
  /* cleanup now */
  pthread_cleanup_pop(1);

  return NULL;
}

/*** plugin interface functions ***/
/******************************************************************************
Description.: this function is called first, in order to initialise
              this plugin and pass a parameter string
Input Value.: parameters
Return Value: 0 if everything is ok, non-zero otherwise
******************************************************************************/
int output_init(output_parameter *param) {
  char *argv[MAX_ARGUMENTS]={NULL};
  int argc=1;

  delay = 0;

  /* convert the single parameter-string to an array of strings */
  argv[0] = OUTPUT_PLUGIN_NAME;
  if ( param->parameter_string != NULL && strlen(param->parameter_string) != 0 ) {
    char *arg=NULL, *saveptr=NULL, *token=NULL;

    arg=(char *)strdup(param->parameter_string);

    if ( strchr(arg, ' ') != NULL ) {
      token=strtok_r(arg, " ", &saveptr);
      if ( token != NULL ) {
        argv[argc] = strdup(token);
        argc++;
        while ( (token=strtok_r(NULL, " ", &saveptr)) != NULL ) {
          argv[argc] = strdup(token);
          argc++;
          if (argc >= MAX_ARGUMENTS) {
            fprintf(stderr, "ERROR: too many arguments to output plugin\n");
            return 1;
          }
        }
      }
    }
  }

  reset_getopt();
  while(1) {
    int option_index = 0, c=0;
    static struct option long_options[] = \
    {
      {"h", no_argument, 0, 0},
      {"help", no_argument, 0, 0},
      {"f", required_argument, 0, 0},
      {"folder", required_argument, 0, 0},
      {"d", required_argument, 0, 0},
      {"delay", required_argument, 0, 0},
      {"c", required_argument, 0, 0},
      {"command", required_argument, 0, 0},
      {"p", required_argument, 0, 0},
      {"port", required_argument, 0, 0},
      {0, 0, 0, 0}
    };

    c = getopt_long_only(argc, argv, "", long_options, &option_index);

    /* no more options to parse */
    if (c == -1) break;

    /* unrecognized option */
    if (c == '?'){
      //help();
      return 1;
    }

    switch (option_index) {
      /* h, help */
      case 0:
      case 1:
        DBG("case 0,1\n");
        //help();
        return 1;
        break;

      /* f, folder */
      case 2:
      case 3:
        DBG("case 2,3\n");
        folder = malloc(strlen(optarg)+1);
        strcpy(folder, optarg);
        if ( folder[strlen(folder)-1] == '/' )
          folder[strlen(folder)-1] = '\0';
        break;

      /* d, delay */
      case 4:
      case 5:
        DBG("case 4,5\n");
        delay = atoi(optarg);
        break;

      /* c, command */
      case 6:
      case 7:
        DBG("case 6,7\n");
        command = strdup(optarg);
        break;
		/* p, p */
		case 8:
	  case 9:
		DBG("case 8,9\n");
		port = atoi(optarg);
		break;
	}
  }
  
  pglobal = param->global;

  OPRINT("output folder.....: %s\n", folder);
  OPRINT("delay after save..: %d\n", delay);
   OPRINT("command...........: %s\n", (command==NULL)?"disabled":command);
  if (port > 0) {
	OPRINT("UDP port..........: %d\n", port);
  }
  else {
	OPRINT("UDP port..........: %s\n","disabled");
  }
  return 0;
}

/******************************************************************************
Description.: calling this function stops the worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int output_stop(int id) {
  DBG("will cancel worker thread\n");
  pthread_cancel(worker);
  fprintf(stderr, "Output canceling worker thread.\n");
  return 0;
}

/******************************************************************************
Description.: calling this function creates and starts the worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int output_run(int id) {
  DBG("launching worker thread\n");
  pthread_create(&worker, 0, worker_thread, NULL);
  pthread_detach(worker);
  fprintf(stderr, "Output starting worker thread.\n");
  return 0;
}

