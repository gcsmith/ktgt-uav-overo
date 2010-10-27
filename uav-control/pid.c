// -----------------------------------------------------------------------------
// File:    pid.c
// Authors: Kevin Macksamie, Garrett Smith
// Created: 09-30-2010
//
// Definitions for proportional-integral-derivative controller algorithm.
// -----------------------------------------------------------------------------

#include <string.h>
#include "pid.h"
#include "utility.h"

#define PID_MAX_ERROR 12
#define PID_RESET     30

// -----------------------------------------------------------------------------
void pid_init(pid_ctrl_t *pid, float sp, float e_min, float e_max)
{
    memset(pid, 0, sizeof(pid_t));
    pid->setpoint = sp;
    pid->e_min = e_min;
    pid->e_max = e_max;
}

// -----------------------------------------------------------------------------
void pid_reset_error(pid_ctrl_t *pid)
{
    pid->e_prev = 0;
    pid->e_curr = 0;
    pid->e_sum = 0;
}

// -----------------------------------------------------------------------------
float pid_update(pid_ctrl_t *pid, float position)
{
    float error  = pid->setpoint - position;
    float p_term = pid->kp * error;
    float d_term = pid->kd * (error - pid->e_prev);

    // shift error to e_curr, shift previous e_curr to e_prev
    pid->e_prev = pid->e_curr;
    pid->e_curr = error;
    pid->e_sum += error;

    // clamp the integral component within a specified error range
    pid->e_sum = CLAMP(pid->e_sum, pid->e_min, pid->e_max);

    // return the sum of the proportional, integral, and derivative terms
    return p_term + (pid->e_sum * pid->ki) + d_term;
}

// -----------------------------------------------------------------------------
float pid_update_with_reset(pid_ctrl_t *pid, float position)
{
    float error  = pid->setpoint - position;
    float p_term = pid->kp * error;
    float d_term = pid->kd * (error - pid->e_prev);

    // shift error to e_curr, shift previous e_curr to e_prev
    pid->e_prev = pid->e_curr;
    pid->e_curr = error;
    pid->e_sum += error;

    // clamp the integral component within a specified error range
    pid->e_sum = CLAMP(pid->e_sum, pid->e_min, pid->e_max);
    if ((error > PID_RESET) || (error < -PID_RESET))
        pid->e_sum = 0;

    // return the sum of the proportional, integral, and derivative terms
    return p_term + (pid->e_sum * pid->ki) + d_term;
}

