// -----------------------------------------------------------------------------
// File:    pid.c
// Authors: Kevin Macksamie, Garrett Smith
// Created: 09-30-2010
//
// Definitions for proportional-integral-derivative controller algorithm.
// -----------------------------------------------------------------------------

#include "pid.h"
#include "utility.h"

#define PID_MAX_ERROR 12
#define PID_RESET     30

void pid_init(pid_ctrl_t *controller, float sp, float kp, float ki, float kd)
{
    controller->setpoint = sp;
    controller->Kp = kp;
    controller->Ki = ki;
    controller->Kd = kd;
    controller->prev_error = 0;
    controller->last_error = 0;
    controller->total_error = 0;
}

void pid_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u)
{
    float curr_diff_error = 0.0f;
    float reset = 1.0f;

    *curr_error = controller->setpoint - input;

    controller->total_error += *curr_error;
    controller->prev_error = controller->last_error;
    controller->last_error = *curr_error;

    curr_diff_error = controller->last_error - controller->prev_error;

    controller->total_error = CLAMP(controller->total_error, -(PID_MAX_ERROR), PID_MAX_ERROR);

    if ((*curr_error > PID_RESET) || (*curr_error < -(PID_RESET)))
        reset = 0.0f;

    *u = (float)((controller->Kp * *curr_error) + 
            (controller->Ki * controller->total_error * reset) + 
            (controller->Kd * curr_diff_error));

    *u = CLAMP(*u, -1.0f, 1.0f);
}

void p_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u)
{
    float curr_diff_error = 0.0f;

    *curr_error = controller->setpoint - input;

    controller->total_error += *curr_error;
    controller->prev_error = controller->last_error;
    controller->last_error = *curr_error;

    curr_diff_error = controller->last_error - controller->prev_error;

    controller->total_error = CLAMP(controller->total_error, -(PID_MAX_ERROR), PID_MAX_ERROR);

    *u = (float)((controller->Kp * *curr_error)); 

    *u = CLAMP(*u, -1.0f, 1.0f);
}

void pd_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u)
{
    float curr_diff_error = 0.0f;

    *curr_error = controller->setpoint - input;

    controller->total_error += *curr_error;
    controller->prev_error = controller->last_error;
    controller->last_error = *curr_error;

    curr_diff_error = controller->last_error - controller->prev_error;

    controller->total_error = CLAMP(controller->total_error, -(PID_MAX_ERROR), PID_MAX_ERROR);

    *u = (float)((controller->Kp * *curr_error) + (controller->Kd * curr_diff_error));

    *u = CLAMP(*u, -1.0f, 1.0f);
}

void pi_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u)
{
    float curr_diff_error = 0.0f;
    float reset = 1.0f;

    *curr_error = controller->setpoint - input;

    controller->total_error += *curr_error;
    controller->prev_error = controller->last_error;
    controller->last_error = *curr_error;

    curr_diff_error = controller->last_error - controller->prev_error;

    controller->total_error = CLAMP(controller->total_error, -(PID_MAX_ERROR), PID_MAX_ERROR);

    if (*curr_error > PID_RESET)
        reset = 0.0f;

    *u = (float)((controller->Kp * *curr_error) + 
            (controller->Ki * controller->total_error * reset));

    *u = CLAMP(*u, -1.0f, 1.0f);
}

