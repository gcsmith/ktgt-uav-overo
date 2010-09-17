//pwm_lib.c
// Timothy Miller 2010

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "pwm_lib.h"
#include "pwm_module.h"


static int pwmFd = -1;

//-------------------------------------------
int pwm_init(void){
    if(pwmFd < 0){
        if( (pwmFd = open("/dev/pwm", O_RDWR)) < 0 ){
	    return -1;
	}
    }
    return 0;
}



