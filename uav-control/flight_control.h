#ifndef FLIGHT_CONTROL__H_
#define FLIGHT_CONTROL__H_

#include "uav_protocol.h"
#include "../pwm-lib/pwm_lib.h"

// PWM channel definitions
#define PWM_ALT     8
#define PWM_PITCH   9
#define PWM_ROLL    10
#define PWM_YAW     11

// Data structure containing control signal values
typedef struct _control_signals
{
    float alt, pitch, roll, yaw;
} ctl_sigs;

// Data structure containing the helicopter's pwm channels
typedef struct _pwm_channels
{
    pwm_t alt, pitch, roll, yaw;
} pwm_chnls;

int open_controls();
void close_controls();
void check_signal_bounds(pwm_t pwm, int *duty);
void flight_control(ctl_sigs *sigs, int chnl_flags);

#endif // FLIGHT_CONTROL__H_

