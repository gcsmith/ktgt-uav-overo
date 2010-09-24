#ifndef PWM_LIB__H_
#define PWM_LIB__H_

#include "pwm_interface.h"

typedef int pwm_t;

pwm_t pwm_open_device(int index);
void pwm_close_device(pwm_t pwm);
int pwm_enable(pwm_t pwm);
int pwm_disable(pwm_t pwm);
int pwm_set_freq(pwm_t pwm, int freq);
int pwm_get_freq(pwm_t pwm);
int pwm_set_duty(pwm_t pwm, int duty);
int pwm_get_duty(pwm_t pwm);
int pwm_set_compare(pwm_t pwm, int compare);
unsigned int pwm_get_minrange(pwm_t pwm);
unsigned int pwm_get_maxrange(pwm_t pwm);
int pwm_get_range(pwm_t pwm, unsigned int *lower, unsigned int *upper);

#endif // PWM_LIB__H_

