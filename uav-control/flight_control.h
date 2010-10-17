#ifndef FLIGHT_CONTROL__H_
#define FLIGHT_CONTROL__H_

#include <pthread.h>
#include "gpio_event.h"
#include "pwm_lib.h"
#include "razor_imu.h"
#include "uav_protocol.h"

#define PWM_ALT     0
#define PWM_YAW     1
#define PWM_ROLL    2
#define PWM_PITCH   3

// PWM channel definitions
#define PWM_DEV_FIRST   8
#define PWM_DEV_LAST    11

#define PWM_DEV_ALT     PWM_DEV_FIRST + 0
#define PWM_DEV_YAW     PWM_DEV_FIRST + 1
#define PWM_DEV_PITCH   PWM_DEV_FIRST + 2
#define PWM_DEV_ROLL    PWM_DEV_FIRST + 3

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

int fc_init(gpio_event_t *pwm_usrf, imu_data_t *ypr_imu);
void fc_shutdown();
void fc_takeoff();
void fc_land();
void fc_update_vcm(int axes, int type);
void fc_update_ctl(ctl_sigs_t *sigs);
void fc_get_vcm(int *axes, int *type);

#endif // FLIGHT_CONTROL__H_

