#ifndef FLIGHT_CONTROL__H_
#define FLIGHT_CONTROL__H_

#include "uav_protocol.h"
#include "../pwm-lib/pwm_lib.h"

#define PWM_ALT     0
#define PWM_PITCH   1
#define PWM_ROLL    2
#define PWM_YAW     3

// PWM channel definitions
#define PWM_DEV_FIRST   8
#define PWM_DEV_LAST    11

#define PWM_DEV_ALT     PWM_DEV_FIRST + 0
#define PWM_DEV_PITCH   PWM_DEV_FIRST + 1
#define PWM_DEV_ROLL    PWM_DEV_FIRST + 2
#define PWM_DEV_YAW     PWM_DEV_FIRST + 3

// Data structure containing control signal values
typedef struct ctl_sigs
{
    float alt, pitch, roll, yaw;
} ctl_sigs_t;

typedef struct pwm_channel
{
    pwm_t handle;
    unsigned int rng_min, rng_max;
} pwm_channel_t;

void assign_duty(pwm_channel_t *pwm, float duty);
void assign_value(pwm_channel_t *pwm, float value);
int open_controls();
void close_controls();
void flight_control(ctl_sigs_t *sigs, int chnl_flags);

#endif // FLIGHT_CONTROL__H_

