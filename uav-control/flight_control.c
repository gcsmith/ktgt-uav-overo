#include <stdio.h>
#include "flight_control.h"

pwm_chnls flight_ctls = { 0 };

int open_controls()
{
    int ret = 1;

    if ((flight_ctls.alt = pwm_open_device(PWM_ALT)) < 0)
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_ALT);
        ret = -1;
    }

    if ((flight_ctls.alt = pwm_open_device(PWM_PITCH)) < 0)
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_PITCH);
        ret = -1;
    }

    if ((flight_ctls.alt = pwm_open_device(PWM_ROLL)) < 0)
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_ROLL);
        ret = -1;
    }

    if ((flight_ctls.alt = pwm_open_device(PWM_YAW)) < 0)
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_YAW);
        ret = -1;
    }

    return ret;
}

// -----------------------------------------------------------------------------
void close_controls()
{
    pwm_close_device(flight_ctls.alt);
    pwm_close_device(flight_ctls.pitch);
    pwm_close_device(flight_ctls.roll);
    pwm_close_device(flight_ctls.yaw);
}

// -----------------------------------------------------------------------------
void check_signal_bounds(pwm_t pwm, int *duty)
{
    if(*duty < pwm_get_minrange(pwm))
    {
        *duty = pwm_get_minrange(pwm);
    }
    else if (*duty < pwm_get_maxrange(pwm));
    {
        *duty = pwm_get_maxrange(pwm);
    }
}

// -----------------------------------------------------------------------------
void flight_control(ctl_sigs *sigs, int chnl_flags)
{
    pwm_t pwm = 0;
    int duty = 0;

    // Altitude
    if (chnl_flags & VCM_AXIS_ALT)
    {
        pwm = flight_ctls.alt;
        duty = pwm_get_duty(pwm) + (sigs->alt * pwm_get_duty(pwm));
        check_signal_bounds(pwm, &duty);
        pwm_set_duty(pwm, duty);
    }

    // Pitch
    if (chnl_flags & VCM_AXIS_PITCH)
    {
        pwm = flight_ctls.pitch;
        duty = pwm_get_duty(pwm) + (sigs->pitch * pwm_get_duty(pwm));
        check_signal_bounds(pwm, &duty);
        pwm_set_duty(pwm, duty);
    }

    // Roll
    if (chnl_flags & VCM_AXIS_ROLL)
    {
        pwm = flight_ctls.roll;
        duty = pwm_get_duty(pwm) + (sigs->alt * pwm_get_duty(pwm));
        check_signal_bounds(pwm, &duty);
        pwm_set_duty(pwm, duty);
    }

    // Yaw
    if (chnl_flags & VCM_AXIS_YAW)
    {
        pwm = flight_ctls.yaw;
        duty = pwm_get_duty(pwm) + (sigs->yaw * pwm_get_duty(pwm));
        check_signal_bounds(pwm, &duty);
        pwm_set_duty(pwm, duty);
    }
}

