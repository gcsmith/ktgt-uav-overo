#include <stdio.h>
#include "flight_control.h"

#define PWM_DUTY_MAX   0.09f
#define PWM_DUTY_MIN   0.05f
#define PWM_DUTY_IDLE  0.07f

pwm_channel_t g_channels[4];

void assign_duty(pwm_channel_t *pwm, float duty)
{
    static int cmp_val = 0;
    
    // check bounds
    if (duty < PWM_DUTY_MIN)
        duty = PWM_DUTY_MIN;
    else if (duty > PWM_DUTY_MAX)
        duty = PWM_DUTY_MAX;

    cmp_val = (int)(pwm->rng_min + (int)((pwm->rng_max - pwm->rng_min) * duty));
    pwm_set_compare(pwm->handle, cmp_val);
}

void assign_value(pwm_channel_t *pwm, float value)
{
    static int cmp_val = 0, max = 0, min = 0;;
    static unsigned int hrange = 0;

    hrange = (pwm->rng_max - pwm->rng_min) >> 1;
    cmp_val = pwm->rng_min + hrange + (int)(hrange * value);

    max = pwm->rng_min + hrange + (int)(hrange * PWM_DUTY_MAX);
    min = pwm->rng_min + hrange + (int)(hrange * PWM_DUTY_MIN);

    if (cmp_val < min)
        cmp_val = min;
    else if (cmp_val > max)
        cmp_val = max;

    pwm_set_compare(pwm->handle, cmp_val);
}

int open_controls()
{
    if (0 > (g_channels[PWM_ALT].handle = pwm_open_device(PWM_DEV_ALT)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_ALT);
        return 0; 
    }

    // keep throttle signal at the current value it is
    pwm_get_range(g_channels[PWM_ALT].handle, &g_channels[PWM_ALT].rng_min,
        &g_channels[PWM_ALT].rng_max);

    if (0 > (g_channels[PWM_PITCH].handle = pwm_open_device(PWM_DEV_PITCH)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_PITCH);
        return 0; 
    }

    pwm_get_range(g_channels[PWM_PITCH].handle, &g_channels[PWM_PITCH].rng_min,
            &g_channels[PWM_PITCH].rng_max);
    assign_duty(&g_channels[PWM_PITCH], PWM_DUTY_IDLE);

    if (0 > (g_channels[PWM_ROLL].handle = pwm_open_device(PWM_DEV_ROLL)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_ROLL);
        return 0;
    }

    pwm_get_range(g_channels[PWM_ROLL].handle, &g_channels[PWM_ROLL].rng_min,
        &g_channels[PWM_ROLL].rng_max);
    assign_duty(&g_channels[PWM_PITCH], PWM_DUTY_IDLE);

    if (0 > (g_channels[PWM_YAW].handle = pwm_open_device(PWM_DEV_YAW)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_YAW);
        return 0;
    }

    pwm_get_range(g_channels[PWM_YAW].handle, &g_channels[PWM_YAW].rng_min,
            &g_channels[PWM_YAW].rng_max);
    assign_duty(&g_channels[PWM_PITCH], PWM_DUTY_IDLE);

    fprintf(stderr, "opened pwm device nodes\n");
    return 1;
}

// -----------------------------------------------------------------------------
void close_controls()
{
    int i;
    for (i = 0; i < 4; ++i)
        pwm_close_device(g_channels[i].handle);
}

// -----------------------------------------------------------------------------
void flight_control(ctl_sigs_t *sigs, int chnl_flags)
{
    // adjust altitude if specified
    if (chnl_flags & VCM_AXIS_ALT)
    {
        assign_value(&g_channels[PWM_ALT], sigs->alt);
    }

    // adjust pitch if specified
    if (chnl_flags & VCM_AXIS_PITCH)
    {
        assign_value(&g_channels[PWM_PITCH], sigs->pitch);
    }

    // adjust roll if specified
    if (chnl_flags & VCM_AXIS_ROLL)
    {
        assign_value(&g_channels[PWM_ROLL], sigs->roll);
    }

    // adjust yaw if specified
    if (chnl_flags & VCM_AXIS_YAW)
    {
        assign_value(&g_channels[PWM_YAW], sigs->yaw);
    }
}

