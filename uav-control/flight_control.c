#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include "flight_control.h"
#include "pid.h"

#define PWM_DUTY_MIN   0.05f
#define PWM_DUTY_MAX   0.09f
#define PWM_DUTY_IDLE  0.07f

#define ALT_DUTY_LO 0.046f
#define ALT_DUTY_HI 0.09f

#define PITCH_DUTY_LO 0.057f
#define PITCH_DUTY_HI 0.09f

#define ROLL_DUTY_LO 0.052f
#define ROLL_DUTY_HI 0.087f

#define YAW_DUTY_LO 0.047f
#define YAW_DUTY_HI 0.094f

typedef struct fc_globals {
    pthread_t takeoff_thrd;
    pthread_t land_thrd;
    pthread_t auto_thrd;

    pthread_mutex_t alive_lock;
    pthread_mutex_t vcm_lock;

    pwm_channel_t channels[4];
    gpio_event_t *usrf; // ultrasonic range finder pwm
    imu_data_t *imu;    // imu sensor

    int vcm_axes;
    int vcm_type;
    int alive;

    float thro_last_value;
    int thro_last_cmp;
    int thro_first;
} fc_globals_t;

static fc_globals_t globals;

// relative throttle control variables

// -----------------------------------------------------------------------------
void assign_duty(pwm_channel_t *pwm, float fmin, float fmax, float duty)
{
    int cmp_val = 0;
    
    // check bounds
    if (duty < fmin)
        duty = fmin;
    else if (duty > fmax)
        duty = fmax;

    cmp_val = (int)(pwm->rng_min + (int)((pwm->rng_max - pwm->rng_min) * duty));
    pwm_set_compare(pwm->handle, cmp_val);
}

// -----------------------------------------------------------------------------
void assign_value(pwm_channel_t *pwm, float fmin, float fmax, float value, 
        int kill)
{
    int cmp, max, min;
    unsigned int range, hrange;

    range = pwm->rng_max - pwm->rng_min;
    min = pwm->rng_min + (int)(range * fmin);
    max = pwm->rng_min + (int)(range * fmax);
    hrange = (max - min) >> 1;

    // thro_last_cmp should be set to the current PWM cmp here to avoid
    // checking cmp == 0 at the end of this function.

    if (kill)
    {
        // reset signals -
        // throttle is set to minimum
        // yaw, pitch, roll are set to their idle positions
        if (globals.channels[PWM_ALT].handle == pwm->handle)
            cmp = min;
        else
            cmp = min + hrange;
    }
    else if (globals.channels[PWM_ALT].handle == pwm->handle)
    {
        // on the first pass set we need to modify the current PWM signal
        if (globals.thro_first == 0)
        {
            globals.thro_first = 1;
            cmp = min + (int)(hrange * value);
            fprintf(stderr, "flight_control alt: value = %f, lv = %f\n",
                    value, globals.thro_last_value);
            globals.thro_last_cmp = cmp;
        }
        else
        {
            cmp = globals.thro_last_cmp + (int)(hrange * value * 0.05f);

            // keep signal within range
            if (cmp > max)
                cmp = max;
            if (cmp < min)
                cmp = min;

            fprintf(stderr, "flight_control alt: value = %f, lv = %f, cmp = %d, lc = %d\n", 
                    value, globals.thro_last_value, cmp, globals.thro_last_cmp);
            globals.thro_last_cmp = cmp;
         }

         globals.thro_last_value = value;
    }
    else
    {
        cmp = min + hrange + (int)(hrange * value);
    }

    if (cmp == 0)
        return;

    fprintf(stderr, "cmp = %u\n", cmp);
    pwm_set_compare(pwm->handle, cmp);
}

// -----------------------------------------------------------------------------
void flight_control(ctl_sigs_t *sigs, int chnl_flags)
{
    int fc_on;

    pthread_mutex_lock(&globals.alive_lock);
    fc_on = globals.alive;
    pthread_mutex_unlock(&globals.alive_lock);

    if (fc_on)
    {
        // adjust altitude if specified
        if (chnl_flags & VCM_AXIS_ALT)
        {
            assign_value(&globals.channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, sigs->alt, 0);
        }
       
        // adjust pitch if specified
        if (chnl_flags & VCM_AXIS_PITCH)
        {
            assign_value(&globals.channels[PWM_PITCH], PITCH_DUTY_LO, PITCH_DUTY_HI, sigs->pitch, 0);
        }
       
        // adjust roll if specified
        if (chnl_flags & VCM_AXIS_ROLL)
        {
            assign_value(&globals.channels[PWM_ROLL], ROLL_DUTY_LO, ROLL_DUTY_HI, sigs->roll, 0);
        }
       
        // adjust yaw if specified
        if (chnl_flags & VCM_AXIS_YAW)
        {
            assign_value(&globals.channels[PWM_YAW], YAW_DUTY_LO, YAW_DUTY_HI, sigs->yaw, 0);
        }
    }
}

// -----------------------------------------------------------------------------
void *autopilot_thread(void *arg)
{
    int kill_reached = 0;
    int axes, type;
    float pid_result, curr_error;

    // flight dynamics
    float fd_pitch, fd_alt; // , fd_yaw, fd_roll;

    // flight control's signal structure
    ctl_sigs_t fc_sigs;
    
    // flight control's PID controllers
    pid_ctrl_t pid_alt_ctlr, pid_pitch_ctlr;

    fc_sigs.alt   = 0.0f;
    fc_sigs.pitch = 0.0f;
    fc_sigs.roll  = 0.0f;
    fc_sigs.yaw   = 0.0f;

    // altitude controller
    pid_alt_ctlr.setpoint    = 42.0f;
    pid_alt_ctlr.Kp          = 0.01f;
    pid_alt_ctlr.Ki          = 0.01f;
    pid_alt_ctlr.Kd          = 0.001f;
    pid_alt_ctlr.prev_error  = 0.0f;
    pid_alt_ctlr.last_error  = 0.0f;
    pid_alt_ctlr.total_error = 0.0f;
    
    // pitch controller
    pid_pitch_ctlr.setpoint    = 5.0f;
    pid_pitch_ctlr.Kp          = 0.01f;
    pid_pitch_ctlr.Ki          = 0.01f;
    pid_pitch_ctlr.Kd          = 0.001f;
    pid_pitch_ctlr.prev_error  = 0.0f;
    pid_pitch_ctlr.last_error  = 0.0f;
    pid_pitch_ctlr.total_error = 0.0f;

    //pthread_mutex_lock(&globals.alive_lock);
    while (globals.alive && !kill_reached)
    {
        //pthread_mutex_unlock(&globals.alive_lock);

        // capture pitch
        pthread_mutex_lock(&globals.imu->lock);
        //fd_yaw   = globals.imu->angles[IMU_DATA_YAW];
        fd_pitch = globals.imu->angles[IMU_DATA_PITCH];
        //fd_roll  = globals.imu->angles[IMU_DATA_ROLL];
        pthread_mutex_unlock(&globals.imu->lock);

        // capture altitude
        fd_alt = gpio_event_read(globals.usrf) / 147;

        // capture flight control's mode and vcm axes
        pthread_mutex_lock(&globals.vcm_lock);
        axes = globals.vcm_axes;
        type = globals.vcm_type;
        pthread_mutex_unlock(&globals.vcm_lock);

        // reset all signals if flight control is of type kill
        if (type == VCM_TYPE_KILL)
        {
            assign_value(&globals.channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, 0, 1);
            assign_value(&globals.channels[PWM_PITCH], PITCH_DUTY_LO, PITCH_DUTY_HI, 0, 1);
            assign_value(&globals.channels[PWM_ROLL], ROLL_DUTY_LO, ROLL_DUTY_HI, 0, 1);
            assign_value(&globals.channels[PWM_YAW], YAW_DUTY_LO, YAW_DUTY_HI, 0, 1);

            kill_reached = 1;
        }
        else if ((type == VCM_TYPE_AUTO) || (type == VCM_TYPE_MIXED))
        {
            // check pitch bit is 0 for autonomous control
            if (!(axes & VCM_AXIS_PITCH))
            {
                pid_compute(&pid_pitch_ctlr, fd_pitch, &curr_error, &pid_result);
                fc_sigs.pitch = pid_result;
                flight_control(&fc_sigs, VCM_AXIS_PITCH);
            }
            
            // check altitude bit is 0 for autonomous control
            if (!(axes & VCM_AXIS_ALT))
            {
                pid_compute(&pid_alt_ctlr, fd_alt, &curr_error, &pid_result);
                fc_sigs.alt = pid_result;
                //flight_control(&fc_sigs, VCM_AXIS_ALT);
            }
        }
    }

    //pthread_mutex_unlock(&globals.alive_lock);
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
void *takeoff_thread(void *arg)
{
    int error, input, last_input = 0, dx_dt, setpoint;
    float last_control = 0.0f;
    char stable = 0, timer_set = 0;
    ctl_sigs_t control;
    clock_t timer = 0;;

    fprintf(stderr, "FLIGHT CONTROL: Helicopter, permission granted to take off\n");

    // We'll need the pitch controller for takeoff to account for the
    // helicopter being front or back heavy, this will ensure the helicopter
    // goes straight up
    // 
    // We'll set the autopilot to control only the pitch - altitude will be
    // controlled through dead reckoning. Eventually the helicopter will
    // be high enough where we can turn on the altitude controller.
    
    // vcm_axes bits - 
    // 0 is autonomous controlled
    // 1 is manual/mixed controlled

    pthread_mutex_lock(&globals.vcm_lock);
    globals.vcm_axes |= VCM_AXIS_ALT;
    //vcm_axes = vcm_axes & ~(VCM_AXIS_PITCH);
    pthread_mutex_unlock(&globals.vcm_lock);

    // spawn autopilot thread
    // autopilot should get be controlling the pitch
    pthread_create(&globals.auto_thrd, NULL, autopilot_thread, NULL);

    setpoint = 42; // 42 inches ~= 1 meter

    while (!stable)
    {
        while ((input = (gpio_event_read(globals.usrf) / 147)) != setpoint)
        {
            fprintf(stderr, "gpio pw = %d\n", input);

            // dead reckoning variables
            error = setpoint - input;
            fprintf(stderr, "error = %d", error);
            if (last_input == 0)
                last_input = input;
            dx_dt = input - last_input; // rate of climb [inches per second]
            last_input = input;

            // if the helicopter is 20 inches from the setpoint, switch to PID control
            if ((error <= (setpoint - 20)) && (error >= (setpoint + 20)))
            {
                // turn the altitude VCM bit off to indicate autonomous control
                pthread_mutex_lock(&globals.vcm_lock);
                globals.vcm_axes &= ~(VCM_AXIS_ALT);
                pthread_mutex_unlock(&globals.vcm_lock);
            }
            else if (error > 0)
            {
                // need to climb
                if (dx_dt < 1)
                {
                    fprintf(stderr, "need to climb\n");
                    control.alt = 0.35;
                    flight_control(&control, VCM_AXIS_ALT);
                }

                // need to slow the rate of climb
                else if (dx_dt > 3)
                {
                    fprintf(stderr, "need to slow down\n");
                    control.alt = last_control * 0.5f;
                    flight_control(&control, VCM_AXIS_ALT);
                }
            }
            else
            {
                control.alt = -0.1f;
                flight_control(&control, VCM_AXIS_ALT);
            }

            last_control = control.alt;

            // sleep for a second to allow the helicopter to lift
            usleep(50000);

            // helicopter has strayed away from stability - stop timing 
            // previous stability time
            if (timer_set)
                timer_set = 0;
        }

        if ((input == setpoint) && (!timer_set))
        {
            // get the time from now that the helicopter should still be at 
            // the setpoint to be considered stable
            timer = clock() + 5 * CLOCKS_PER_SEC;
            timer_set = 1;
        }
        else if ((input == setpoint) && timer_set)
        {
            // helicopter has been at the setpoint for 5 seconds
            // alititude requirement has been achieved
            if (clock() >= timer)
                stable = 1;
        }
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
void *landing_thread(void *arg)
{
    int i;
    int alt, prev_alt = 0, dx_dt;
    ctl_sigs_t landing_sigs;

    landing_sigs.alt = 0.0f;

    // procedure:
    // turn autonomous pitch on
    // turn autonomous altitude off
    // slowly kill throttle
    // once throttle is at minimum exit function

    pthread_mutex_lock(&globals.vcm_lock);
    globals.vcm_axes &= ~(VCM_AXIS_PITCH);
    globals.vcm_axes |= VCM_AXIS_ALT;
    pthread_mutex_unlock(&globals.vcm_lock);

    // monitor altitude
    while ((alt = gpio_event_read(globals.usrf) / 147) > 6)
    {
        if (prev_alt == 0)
            prev_alt = alt;
        dx_dt = alt - prev_alt;
        prev_alt = alt;

        if (dx_dt == 0)
        {
            landing_sigs.alt = -0.1f;
            flight_control(&landing_sigs, VCM_AXIS_ALT);
        }
        else if (dx_dt <= -3) 
        {
            landing_sigs.alt = 0.05f;
            flight_control(&landing_sigs, VCM_AXIS_ALT);
        }

        usleep(50000);
    }
    
    landing_sigs.alt = -0.05;
    for (i = 0; i < 5; i++)
    {
        flight_control(&landing_sigs, VCM_AXIS_ALT);
        usleep(100000);
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
int init_channel(int index, int duty_lo, int duty_hi, int duty_idle)
{
    pwm_channel_t *pwm = &globals.channels[index];
    if (0 > (pwm->handle = pwm_open_device(PWM_DEV_FIRST + index))) {
        syslog(LOG_ERR, "error opening pwm %d device\n", index);
        return 0; 
    }

    // keep throttle signal at the current value it is
    pwm_set_freq_x100(pwm->handle, 4580);
    pwm_get_range(pwm->handle, &pwm->rng_min, &pwm->rng_max);
    assign_duty(pwm, duty_lo, duty_hi, duty_idle);
    return 1;
}

// -----------------------------------------------------------------------------
int fc_init(gpio_event_t *pwm_usrf, imu_data_t *ypr_imu)
{
    globals.vcm_type = VCM_TYPE_AUTO;
    globals.vcm_axes = VCM_AXIS_ALL;

    globals.thro_last_value = 0.0f;
    globals.thro_last_cmp = 0;
    globals.thro_first = 0;

    // direct flight control to ultrasonic sensor
    globals.usrf = pwm_usrf;

    // direct flight control to IMU data array
    globals.imu = ypr_imu;

    pthread_mutex_init(&globals.alive_lock, NULL);
    pthread_mutex_init(&globals.vcm_lock, NULL);

    if (!init_channel(PWM_ALT, ALT_DUTY_LO, ALT_DUTY_HI, ALT_DUTY_LO))
        return 0;
    syslog(LOG_INFO, "flight control: altitude channel opened\n");

    if (!init_channel(PWM_PITCH, PITCH_DUTY_LO, PITCH_DUTY_HI, PWM_DUTY_IDLE))
        return 0; 
    syslog(LOG_INFO, "flight control: pitch channel opened\n");

    if (!init_channel(PWM_ROLL, ROLL_DUTY_LO, ROLL_DUTY_HI, PWM_DUTY_IDLE))
        return 0;
    syslog(LOG_INFO, "flight control: roll channel opened\n");

    if (!init_channel(PWM_YAW, YAW_DUTY_LO, YAW_DUTY_HI, PWM_DUTY_IDLE))
        return 0;
    syslog(LOG_INFO, "flight control: yaw channel opened\n");

    globals.alive = 1;
    syslog(LOG_INFO, "opened pwm device nodes\n");
    return 1;
}

// -----------------------------------------------------------------------------
void fc_shutdown()
{
    int i;

    // set flag for all controllers to see that flight control is shutting down
    pthread_mutex_lock(&globals.alive_lock);
    globals.alive = 0;
    pthread_mutex_unlock(&globals.alive_lock);

    // TODO: Wait for controller threads to exit

    for (i = 0; i < 4; ++i)
        pwm_close_device(globals.channels[i].handle);

    globals.usrf = NULL;

    pthread_mutex_destroy(&globals.alive_lock);
    pthread_mutex_destroy(&globals.vcm_lock);
}

// -----------------------------------------------------------------------------
void fc_takeoff()
{
    if (globals.alive)
        pthread_create(&globals.takeoff_thrd, NULL, takeoff_thread, NULL);
}

// -----------------------------------------------------------------------------
void fc_land()
{
    if (globals.alive)
        pthread_create(&globals.land_thrd, NULL, landing_thread, NULL);
}

// -----------------------------------------------------------------------------
void fc_update_vcm(int axes, int type)
{
    pthread_mutex_lock(&globals.vcm_lock);
    globals.vcm_axes = axes;
    globals.vcm_type = type;
    pthread_mutex_unlock(&globals.vcm_lock);

    if ((type == VCM_TYPE_LOCKOUT) || (type == VCM_TYPE_KILL))
    {
        globals.thro_last_value = 0.0f;
        globals.thro_last_cmp = 0;
        globals.thro_first = 0;
        assign_duty(&globals.channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, ALT_DUTY_LO);
        pthread_mutex_lock(&globals.alive_lock);
        globals.alive = 0;
        pthread_mutex_unlock(&globals.alive_lock);
    }
}

// -----------------------------------------------------------------------------
void fc_update_ctl(ctl_sigs_t *sigs)
{
    int axes;

    pthread_mutex_lock(&globals.vcm_lock);
    axes = globals.vcm_axes;
    pthread_mutex_unlock(&globals.vcm_lock);

    // if the incoming signal is a throttle event from the client, then we
    // target just the altitude control signal so we don't disturb the other
    // control signals
    fprintf(stderr, "flight control's axes: %d\n", axes);
    flight_control(sigs, axes);
}

// -----------------------------------------------------------------------------
void fc_get_vcm(int *axes, int *type)
{
    *axes = globals.vcm_axes;
    *type = globals.vcm_type;
}

