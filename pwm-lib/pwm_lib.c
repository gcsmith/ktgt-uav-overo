#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include "pwm_lib.h"

pwm_t pwm_open_device(int index)
{
    char dev[16] = "/dev/pwm";
    char index_str[3];
    sprintf(index_str, "%d", index);
    sprintf(dev, "%s%s", dev, index_str);
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

unsigned int pwm_get_minrange(pwm_t pwm)
{
    return (unsigned int)ioctl(pwm, PWM_IOCQ_MINRANGE);
}

unsigned int pwm_get_maxrange(pwm_t pwm)
{
    return (unsigned int)ioctl(pwm, PWM_IOCQ_MAXRANGE);
}

int pwm_get_range(pwm_t pwm, unsigned int *lower, unsigned int *upper)
{
    *lower = (unsigned int)ioctl(pwm, PWM_IOCQ_MINRANGE);
    *upper = (unsigned int)ioctl(pwm, PWM_IOCQ_MAXRANGE);
    return 1;
}

