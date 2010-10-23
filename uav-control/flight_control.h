// -----------------------------------------------------------------------------
// File:    flight_control.h
// Authors: Kevin Macksamie, Garrett Smith
// Created: 09-23-2010
//
// Algorithms and state management for computer controlled flight.
// -----------------------------------------------------------------------------

#ifndef FLIGHT_CONTROL__H_
#define FLIGHT_CONTROL__H_

#include <pthread.h>
#include "gpio_event.h"
#include "pwm_lib.h"
#include "razor_imu.h"
#include "uav_protocol.h"
#include "utility.h"

#define PWM_ALT     0
#define PWM_YAW     1
#define PWM_PITCH   2
#define PWM_ROLL    3

// PWM channel definitions

#define PWM_DEV_FIRST   8
#define PWM_DEV_LAST    11

#define PWM_DEV_ALT     (PWM_DEV_FIRST + PWM_ALT)
#define PWM_DEV_YAW     (PWM_DEV_FIRST + PWM_YAW)
#define PWM_DEV_PITCH   (PWM_DEV_FIRST + PWM_PITCH)
#define PWM_DEV_ROLL    (PWM_DEV_FIRST + PWM_ROLL)

#define RECORD_BUCKET_SIZE 1024

typedef struct ctl_sigs
{
    float alt, pitch, roll, yaw;
} ctl_sigs_t;

typedef struct input_record
{
    timespec_t delta;
    ctl_sigs_t signals;
} input_record_t;

typedef struct record_bucket
{
    input_record_t records[RECORD_BUCKET_SIZE];
    unsigned int count;
    struct record_bucket *next;
} record_bucket_t;

typedef struct pwm_channel
{
    pwm_t handle;
    unsigned int rng_min, rng_max;
    unsigned int cmp;
    unsigned int trim;
} pwm_channel_t;

// initialize the flight control subsystem
int fc_init(gpio_event_t *pwm_usrf, imu_data_t *ypr_imu);

// shutdown the flight control subsystem
void fc_shutdown();

// set capture path. all input signals will be logged and stored for mixed mode
int fc_set_capture(const char *path);

// set replay path. all autonomous flight will be replayed from stored signals
int fc_set_replay(const char *path);

// request autonomous takeoff
int fc_takeoff();

// request autonomous landing
int fc_land();

// update vehicle control mode info (control type and enabled axes)
void fc_set_vcm(int axes, int type);

// get the current control mode type and enabled axes
void fc_get_vcm(int *axes, int *type);

// inject manual input control signals
void fc_set_ctl(ctl_sigs_t *sigs);

void fc_set_trim(int channel, int value);

int fc_get_trim(int channel);

#endif // FLIGHT_CONTROL__H_

