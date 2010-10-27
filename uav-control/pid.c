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
float pid_update(pid_ctrl_t *pid, float error)
{
    float p_term = pid->kp * error;
    float d_term = pid->kd * (error - pid->e_prev);

    // shift error to e_curr, shift previous e_curr to e_prev
    pid->e_prev = pid->e_curr;
    pid->e_curr = error;
    pid->e_sum += error;

    // clamp the integral component within a specified range
    pid->e_sum = CLAMP(pid->e_sum, pid->e_min, pid->e_max);

    return p_term + d_term + (pid->e_sum * pid->ki);
}

// -----------------------------------------------------------------------------
void pid_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u)
{
    float curr_diff_error = 0.0f;
    float reset = 1.0f;

    *curr_error = controller->setpoint - input;

    controller->e_sum += *curr_error;
    controller->e_prev = controller->e_curr;
    controller->e_curr = *curr_error;

    curr_diff_error = controller->e_curr - controller->e_prev;

    controller->e_sum = CLAMP(controller->e_sum, -(PID_MAX_ERROR), PID_MAX_ERROR);

    if ((*curr_error > PID_RESET) || (*curr_error < -(PID_RESET)))
        reset = 0.0f;

    *u = (float)((controller->kp * *curr_error) + 
            (controller->ki * controller->e_sum * reset) + 
            (controller->kd * curr_diff_error));

    *u = CLAMP(*u, -1.0f, 1.0f);
}

