// -----------------------------------------------------------------------------
// File:    pid.h
// Authors: Kevin Macksamie
// Created: 09-30-2010
//
// Definitions for proportional-integral-derivative controller algorithm.
// -----------------------------------------------------------------------------

#ifndef _UAV_PID__H_
#define _UAV_PID__H_

typedef struct _pid pid_ctrl_t;
struct _pid
{
    float setpoint;     /* Desired value of controller */
    float Kp;           /* Proportion constant */
    float Ki;           /* Integral constant */
    float Kd;           /* Derivative constant */
    float prev_error;   /* Error from previous sample */
    float last_error;   /* Latest error collected */
    float total_error;  /* Error sum */
};

// -----------------------------------------------------------------------------
// Function: pid_compute
// Inputs:
//   + controller - the PID controller being used
//   + input - the sensor output the controller is monitoring
//   + curr_error - the error between the controller setpoint and the sensor
//     output
//   + u - the output of the controller
// Outputs:
//   + curr_error
//   + u
// -----------------------------------------------------------------------------
void pid_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u);

void p_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u);

void pd_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u);

void pi_compute(pid_ctrl_t *controller, float input, float *curr_error, float *u);

#endif // _UAV_PID__H_

