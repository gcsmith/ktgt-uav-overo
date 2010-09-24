#include <stdio.h>
#include "flight_control.h"

// Data structure containing the helicopter's pwm channels
typedef struct pwm_channel
{
    pwm_t handle;
    float value;
} pwm_channel_t;

pwm_channel_t g_channels[4];

int open_controls()
{
    int ret = 1;

    if (0 > (g_channels[PWM_ALT].handle = pwm_open_device(PWM_DEV_ALT)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_ALT);
        ret = -1;
    }

    if (0 > (g_channels[PWM_PITCH].handle = pwm_open_device(PWM_DEV_PITCH)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_PITCH);
        ret = -1;
    }

    if (0 > (g_channels[PWM_ROLL].handle = pwm_open_device(PWM_DEV_ROLL)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_ROLL);
        ret = -1;
    }

    if (0 > (g_channels[PWM_YAW].handle = pwm_open_device(PWM_DEV_YAW)))
    {
        fprintf(stderr, "Error opening pwm device %d\n", PWM_DEV_YAW);
        ret = -1;
    }

    fprintf(stderr, "opened pwm device nodes\n");
    return ret;
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
    pwm_t pwm = 0;
    unsigned int lower, upper, hrange;

    // adjust altitude if specified
    if (chnl_flags & VCM_AXIS_ALT)
    {
        pwm = g_channels[PWM_ALT].handle;
        pwm_get_range(pwm, &lower, &upper);
        hrange = (upper - lower) >> 1;
        pwm_set_compare(pwm, lower + hrange + (int)(hrange * sigs->alt));
    }

    // adjust pitch if specified
    if (chnl_flags & VCM_AXIS_PITCH)
    {
        pwm = g_channels[PWM_PITCH].handle;
        pwm_get_range(pwm, &lower, &upper);
        hrange = (upper - lower) >> 1;
        pwm_set_compare(pwm, lower + hrange + (int)(hrange * sigs->pitch));
    }

    // adjust roll if specified
    if (chnl_flags & VCM_AXIS_ROLL)
    {
        pwm = g_channels[PWM_ROLL].handle;
        pwm_get_range(pwm, &lower, &upper);
        hrange = (upper - lower) >> 1;
        pwm_set_compare(pwm, lower + hrange + (int)(hrange * sigs->roll));
    }

    // adjust yaw if specified
    if (chnl_flags & VCM_AXIS_YAW)
    {
        pwm = g_channels[PWM_YAW].handle;
        pwm_get_range(pwm, &lower, &upper);
        hrange = (upper - lower) >> 1;
        pwm_set_compare(pwm, lower + hrange + (int)(hrange * sigs->yaw));
    }
}

