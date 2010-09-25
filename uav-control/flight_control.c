#include <stdio.h>
#include "flight_control.h"

#define PWM_DUTY_MAX   0.09f
#define PWM_DUTY_MIN   0.05f
#define PWM_DUTY_IDLE  0.07f

pwm_channel_t g_channels[4];

void assign_duty(pwm_channel_t *pwm, float duty)
{
    int cmp_val = 0;
    
    // check bounds
    if (duty < PWM_DUTY_MIN)
        duty = PWM_DUTY_MIN;
    else if (duty > PWM_DUTY_MAX)
        duty = PWM_DUTY_MAX;

    cmp_val = (int)(pwm->rng_min + (int)((pwm->rng_max - pwm->rng_min) * duty));
    fprintf(stderr, "rng_max = %u\n", pwm->rng_max);
    fprintf(stderr, "rng_min = %u\n", pwm->rng_min);
    fprintf(stderr, "cmp_val = %u\n", cmp_val);
    pwm_set_compare(pwm->handle, cmp_val);
}

void assign_value(pwm_channel_t *pwm, float value)
{
    int cmp, max, min;
    unsigned int range, hrange;

    range = pwm->rng_max - pwm->rng_min;
    min = pwm->rng_min + (int)(range * PWM_DUTY_MIN);
    max = pwm->rng_min + (int)(range * PWM_DUTY_MAX);
    hrange = (max - min) >> 1;
    cmp = min + hrange + (int)(hrange * value);

    pwm_set_compare(pwm->handle, cmp);
}

int open_controls()
{
    if (0 > (g_channels[PWM_ALT].handle = pwm_open_device(PWM_DEV_ALT)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_ALT);
        return 0; 
    }
    fprintf(stderr, "flight control: altitude channel opened\n");

    // keep throttle signal at the current value it is
    pwm_get_range(g_channels[PWM_ALT].handle, &g_channels[PWM_ALT].rng_min,
        &g_channels[PWM_ALT].rng_max);

    if (0 > (g_channels[PWM_PITCH].handle = pwm_open_device(PWM_DEV_PITCH)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_PITCH);
        return 0; 
    }
    fprintf(stderr, "flight control: pitch channel opened\n");

    pwm_get_range(g_channels[PWM_PITCH].handle, &g_channels[PWM_PITCH].rng_min,
            &g_channels[PWM_PITCH].rng_max);
    assign_duty(&g_channels[PWM_PITCH], PWM_DUTY_IDLE);

    if (0 > (g_channels[PWM_ROLL].handle = pwm_open_device(PWM_DEV_ROLL)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_ROLL);
        return 0;
    }
    fprintf(stderr, "flight control: roll channel opened\n");

    pwm_get_range(g_channels[PWM_ROLL].handle, &g_channels[PWM_ROLL].rng_min,
        &g_channels[PWM_ROLL].rng_max);
    assign_duty(&g_channels[PWM_ROLL], PWM_DUTY_IDLE);

    if (0 > (g_channels[PWM_YAW].handle = pwm_open_device(PWM_DEV_YAW)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_YAW);
        return 0;
    }
    fprintf(stderr, "flight control: yaw channel opened\n");

    pwm_get_range(g_channels[PWM_YAW].handle, &g_channels[PWM_YAW].rng_min,
            &g_channels[PWM_YAW].rng_max);
    assign_duty(&g_channels[PWM_YAW], PWM_DUTY_IDLE);

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

