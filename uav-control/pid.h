// -----------------------------------------------------------------------------
// File:    pid.h
// Authors: Kevin Macksamie, Garrett Smith
// Created: 09-30-2010
//
// Definitions for proportional-integral-derivative controller algorithm.
// -----------------------------------------------------------------------------

#ifndef _UAV_PID__H_
#define _UAV_PID__H_

typedef struct pid_ctrl
{
    float setpoint;     // desired value of controller
    float kp;           // proportion constant
    float ki;           // integral constant
    float kd;           // derivative constant
    float e_min;        // minimum range of error
    float e_max;        // maximum range of error
    float e_prev;       // error from previous sample
    float e_curr;       // latest error collected
    float e_sum;        // error sum
} pid_ctrl_t;

// initialize the pid controller with specified setpoint/min/max
void pid_init(pid_ctrl_t *pid, float sp, float e_min, float e_max);

// reset the error state within the pid controller
void pid_reset_error(pid_ctrl_t *pid);

// compute the pid for the specified position
float pid_update(pid_ctrl_t *pid, float position);

// compute the pid for the specified position with integral reset
float pid_update_with_reset(pid_ctrl_t *pid, float position);

#endif // _UAV_PID__H_

