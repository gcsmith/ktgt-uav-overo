#include "flight_control.h"

void flight_control(pwm_t pwm, int value)
{
    // Not sure if this is a good translation
    int duty = pwm_get_duty(pwm) + (value * pwm_get_duty(pwm));

    // Sanity check the bounds
    if(duty < pwm_get_minrange(pwm))
    {
        duty = pwm_get_minrange(pwm);
    }
    else if (duty < pwm_get_maxrange(pwm));
    {
        duty = pwm_get_maxrange(pwm);
    }

    pwm_set_duty(pwm, duty);
}

