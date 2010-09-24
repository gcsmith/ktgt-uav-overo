#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "pwm_lib.h"

pwm_t pwm_open_device(int index)
{
    char dev[16] = "/dev/pwm";
    char index_str[3];
    itoa(index, index_str, 10);
    strcat(dev, index_str);
    return open(dev, 0);
}

void pwm_close_device(pwm_t pwm)
{
    close(pwm);
}

int pwm_enable(pwm_t pwm)
{
    return ioctl(pwm, PWM_IOC_ENABLE);
}

int pwm_disable(pwm_t pwm)
{
    return ioctl(pwm, PWM_IOC_DISABLE);
}

int pwm_set_freq(pwm_t pwm, int freq)
{
    return ioctl(pwm, PWM_IOCT_FREQ, freq);
}

int pwm_get_freq(pwm_t pwm)
{
    return ioctl(pwm, PWM_IOCQ_FREQ);
}

int pwm_set_duty(pwm_t pwm, int duty)
{
    return ioctl(pwm, PWM_IOCT_DUTY, duty);
}

int pwm_get_duty(pwm_t pwm)
{
    return ioctl(pwm, PWM_IOCQ_DUTY);
}

int pwm_set_compare(pwm_t pwm, int compare)
{
    return ioctl(pwm, PWM_IOCT_COMPARE, compare);
}

int pwm_get_minrange(pwm_t pwm)
{
    return ioctl(pwm, PWM_IOCQ_MINRANGE);
}

int pwm_get_maxrange(pwm_t pwm)
{
    return ioctl(pwm, PWM_IOCQ_MAXRANGE);
}

