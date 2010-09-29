#include <stdio.h>
#include "flight_control.h"

#define PWM_DUTY_MIN   0.05f
#define PWM_DUTY_MAX   0.09f
#define PWM_DUTY_IDLE  0.07f

#define ALT_DUTY_LO 0.046f
#define ALT_DUTY_HI 0.09f

#define PITCH_DUTY_LO 0.057f
#define PITCH_DUTY_HI 0.09f

#define ROLL_DUTY_LO 0.052f
#define ROLL_DUTY_HI 0.087f

#define YAW_DUTY_LO 0.047f
#define YAW_DUTY_HI 0.094f

pwm_channel_t g_channels[4];

void assign_duty(pwm_channel_t *pwm, float fmin, float fmax, float duty)
{
    int cmp_val = 0;
    
    // check bounds
    if (duty < fmin)
        duty = fmin;
    else if (duty > fmax)
        duty = fmax;

    cmp_val = (int)(pwm->rng_min + (int)((pwm->rng_max - pwm->rng_min) * duty));
    fprintf(stderr, "rng_max = %u\n", pwm->rng_max);
    fprintf(stderr, "rng_min = %u\n", pwm->rng_min);
    fprintf(stderr, "cmp_val = %u\n", cmp_val);
    pwm_set_compare(pwm->handle, cmp_val);
}

void assign_value(pwm_channel_t *pwm, float fmin, float fmax, float value)
{
    static float last_value = 0;
    int cmp, max, min;
    unsigned int range, hrange;

    range = pwm->rng_max - pwm->rng_min;
    min = pwm->rng_min + (int)(range * fmin);
    max = pwm->rng_min + (int)(range * fmax);
    hrange = (max - min) >> 1;

    if (g_channels[PWM_ALT].handle == pwm->handle)
    {
        // joystick is scrolling from up to down
        if ((last_value > value) && (last_value > 0))
        {
            fprintf(stderr, "Breaking early: l_v > v\n");
            last_value = value;
            return;
        }

        // joystick is scrolling from down to up
        if ((last_value < value) && (last_value < 0))
        {
            fprintf(stderr, "Breaking early: l_v < v\n");
            last_value = value;
            return;
        }

        // no delta
        if (last_value == value)
        {
            fprintf(stderr, "Breaking early: l_v == v\n");
            last_value = value;
            return;
        }

        fprintf(stderr, "flight_control: value = %f\n", value);
        cmp = min + hrange + (int)(hrange * value);
        last_value = value;
    }
    else
    {
        fprintf(stderr, "flight_control: value = %f\n", value);
        cmp = min + hrange + (int)(hrange * value);
    }

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
    pwm_set_freq_x100(g_channels[PWM_ALT].handle, 4580);
    pwm_get_range(g_channels[PWM_ALT].handle, &g_channels[PWM_ALT].rng_min,
        &g_channels[PWM_ALT].rng_max);
    assign_duty(&g_channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, ALT_DUTY_LO);

    if (0 > (g_channels[PWM_PITCH].handle = pwm_open_device(PWM_DEV_PITCH)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_PITCH);
        return 0; 
    }
    fprintf(stderr, "flight control: pitch channel opened\n");

    pwm_set_freq_x100(g_channels[PWM_PITCH].handle, 4580);
    pwm_get_range(g_channels[PWM_PITCH].handle, &g_channels[PWM_PITCH].rng_min,
            &g_channels[PWM_PITCH].rng_max);
    assign_duty(&g_channels[PWM_PITCH], PITCH_DUTY_LO, PITCH_DUTY_HI, PWM_DUTY_IDLE);

    if (0 > (g_channels[PWM_ROLL].handle = pwm_open_device(PWM_DEV_ROLL)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_ROLL);
        return 0;
    }
    fprintf(stderr, "flight control: roll channel opened\n");

    pwm_set_freq_x100(g_channels[PWM_ROLL].handle, 4580);
    pwm_get_range(g_channels[PWM_ROLL].handle, &g_channels[PWM_ROLL].rng_min,
        &g_channels[PWM_ROLL].rng_max);
    assign_duty(&g_channels[PWM_ROLL], ROLL_DUTY_LO, ROLL_DUTY_HI, PWM_DUTY_IDLE);

    if (0 > (g_channels[PWM_YAW].handle = pwm_open_device(PWM_DEV_YAW)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_YAW);
        return 0;
    }
    fprintf(stderr, "flight control: yaw channel opened\n");

    pwm_set_freq_x100(g_channels[PWM_YAW].handle, 4580);
    pwm_get_range(g_channels[PWM_YAW].handle, &g_channels[PWM_YAW].rng_min,
            &g_channels[PWM_YAW].rng_max);
    assign_duty(&g_channels[PWM_YAW], YAW_DUTY_LO, YAW_DUTY_HI, PWM_DUTY_IDLE);

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
        assign_value(&g_channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, sigs->alt);
    }

    // adjust pitch if specified
    if (chnl_flags & VCM_AXIS_PITCH)
    {
        assign_value(&g_channels[PWM_PITCH], PITCH_DUTY_LO, PITCH_DUTY_HI, sigs->pitch);
    }

    // adjust roll if specified
    if (chnl_flags & VCM_AXIS_ROLL)
    {
        assign_value(&g_channels[PWM_ROLL], ROLL_DUTY_LO, ROLL_DUTY_HI, sigs->roll);
    }

    // adjust yaw if specified
    if (chnl_flags & VCM_AXIS_YAW)
    {
        assign_value(&g_channels[PWM_YAW], YAW_DUTY_LO, YAW_DUTY_HI, sigs->yaw);
    }
}

