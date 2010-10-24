#include "pid.h"
#include "utility.h"

#define PID_RESET 30

/*****************************************************************************
 * u(t) = Kp *e(t) + Ki * integral(e(t), 0, t) + Kd, * d[e(t)]/dt
 * 
 * where:
 * u(t) is the controller output
 * e(t) is the error input into the controller
 * Kp is the proportional parameter
 * Ki is the integral parameter
 * Kd is the derivative parameter
 *****************************************************************************/
void pid_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u)
{
    float curr_diff_error = 0.0f;
    float reset = 1.0f;

    *curr_error = controller->setpoint - input;

    controller->total_error += *curr_error;
    controller->prev_error = controller->last_error;
    controller->last_error = *curr_error;

    curr_diff_error = controller->last_error - controller->prev_error;

    if (*curr_error > PID_RESET)
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

    if (*curr_error > PID_RESET)
        reset = 0.0f;

    *u = (float)((controller->Kp * *curr_error) + 
            (controller->Ki * controller->total_error * reset));

    *u = CLAMP(*u, -1.0f, 1.0f);
}

