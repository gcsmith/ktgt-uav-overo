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

// threads
pthread_t takeoff_thrd;
pthread_t land_thrd;
pthread_t auto_thrd;

// mutexes
pthread_mutex_t fc_alive_event;
pthread_mutex_t fc_vcm_event;
pthread_mutex_t fc_cond_ap_mutex;

// pwm channels to helicopter's controls
pwm_channel_t g_channels[4];

gpio_event_t *usrf; // ultrasonic range finder pwm
imu_data_t *imu;    // imu sensor

// axes flags
// if an axis is set in vcm_axes, then this means that axis is not to be 
// autonomously controlled
int vcm_axes;

// vcm type
// flight control should know how the helicopter is operating
int vcm_type;

// flight control's alive flag
char fc_alive;

// flight control's autonomous control flag
char fc_allow_autonomous;

// relative throttle control variables
float thro_last_value = 0.0f;
int thro_last_cmp = 0, thro_first = 0;

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
        if (g_channels[PWM_ALT].handle == pwm->handle)
            cmp = min;
        else
            cmp = min + hrange;
    }
    else if (g_channels[PWM_ALT].handle == pwm->handle)
    {
        // on the first pass set we need to modify the current PWM signal
        if (thro_first == 0)
        {
            thro_first = 1;
            cmp = min + (int)(hrange * value);
            fprintf(stderr, "flight_control alt: value = %f, lv = %f\n", value, thro_last_value);
            thro_last_cmp = cmp;
        }
        else
        {
            cmp = thro_last_cmp + (int)(hrange * value * 0.15f);

            // keep signal within range
            if (cmp > max)
                cmp = max;
            if (cmp < min)
                cmp = min;

            fprintf(stderr, "flight_control alt: value = %f, lv = %f, cmp = %d, lc = %d\n", 
                    value, thro_last_value, cmp, thro_last_cmp);
            thro_last_cmp = cmp;
         }

         thro_last_value = value;
    }
    else
    {
        cmp = min + hrange + (int)(hrange * value);
    }

    if (cmp == 0)
        return;

    fprintf(stderr, "cmp = %d\n", cmp);
    pwm_set_compare(pwm->handle, cmp);
}

// -----------------------------------------------------------------------------
void *autopilot()
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

    pthread_mutex_lock(&fc_alive_event);
    while (fc_alive && !kill_reached)
    {
        pthread_mutex_unlock(&fc_alive_event);

        // capture pitch
        pthread_mutex_lock(&imu->lock);
        //fd_yaw   = imu->angles[IMU_DATA_YAW];
        fd_pitch = imu->angles[IMU_DATA_PITCH];
        //fd_roll  = imu->angles[IMU_DATA_ROLL];
        pthread_mutex_unlock(&imu->lock);

        // capture altitude
        pthread_mutex_lock(&usrf->lock);
        fd_alt = usrf->pulsewidth / 147;
        pthread_mutex_unlock(&usrf->lock);

        // capture flight control's mode and vcm axes
        pthread_mutex_lock(&fc_vcm_event);
        axes = vcm_axes;
        type = vcm_type;
        pthread_mutex_unlock(&fc_vcm_event);

        // reset all signals if flight control is of type kill
        if (type == VCM_TYPE_KILL)
        {
            assign_value(&g_channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, 0, 1);
            assign_value(&g_channels[PWM_PITCH], PITCH_DUTY_LO, PITCH_DUTY_HI, 0, 1);
            assign_value(&g_channels[PWM_ROLL], ROLL_DUTY_LO, ROLL_DUTY_HI, 0, 1);
            assign_value(&g_channels[PWM_YAW], YAW_DUTY_LO, YAW_DUTY_HI, 0, 1);

            kill_reached = 1;
        }
        else if ((vcm_type == VCM_TYPE_AUTO) || (vcm_type == VCM_TYPE_MIXED))
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
                flight_control(&fc_sigs, VCM_AXIS_ALT);
            }
        }
    }

    pthread_mutex_unlock(&fc_alive_event);
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
void *takeoff()
{
    int error, input, last_input = 0, dx_dt, setpoint;
    float last_control;
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

    pthread_mutex_lock(&fc_vcm_event);
    vcm_axes = vcm_axes | VCM_AXIS_ALT;
    vcm_axes = vcm_axes & ~(VCM_AXIS_PITCH);
    pthread_mutex_unlock(&fc_vcm_event);

    // spawn autopilot thread
    // autopilot should get be controlling the pitch
    pthread_create(&auto_thrd, NULL, autopilot, NULL);

    setpoint = 42; // 42 inches ~= 1 meter

    while (!stable)
    {
        pthread_mutex_lock(&usrf->lock);
        while ((input = (usrf->pulsewidth / 147)) != setpoint)
        {
            pthread_mutex_unlock(&usrf->lock);

            // dead reckoning variables
            error = setpoint - input;
            if (last_input == 0)
                last_input = input;
            dx_dt = input - last_input; // rate of climb [inches per second]
            last_input = input;

            // if the helicopter is 20 inches from the setpoint, switch to PID control
            if ((error <= (setpoint - 20)) && (error >= (setpoint + 20)))
            {
                // turn the altitude VCM bit off to indicate autonomous control
                pthread_mutex_lock(&fc_vcm_event);
                vcm_axes = vcm_axes & ~(VCM_AXIS_ALT);
                pthread_mutex_unlock(&fc_vcm_event);
            }
            else if (error > 0)
            {
                // need to climb
                if (dx_dt < 1)
                {
                    control.alt = 0.1f;
                    //flight_control(&control, VCM_AXIS_ALT);
                }

                // need to slow the rate of climb
                else if (dx_dt > 3)
                {
                    control.alt = last_control * 0.5f;
                    //flight_control(&control, VCM_AXIS_ALT);
                }
            }
            else
            {
                control.alt = -0.1f;
                //flight_control(&control, VCM_AXIS_ALT);
            }

            last_control = control.alt;

            // sleep for a second to allow the helicopter to lift
            usleep(1000000);

            // helicopter has strayed away from stability - stop timing 
            // previous stability time
            if (timer_set)
                timer_set = 0;
        }

        pthread_mutex_unlock(&usrf->lock);

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
void *land()
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

    pthread_mutex_lock(&fc_vcm_event);
    vcm_axes = vcm_axes & ~(VCM_AXIS_PITCH);
    vcm_axes = vcm_axes | VCM_AXIS_ALT;
    pthread_mutex_unlock(&fc_vcm_event);

    // monitor altitude
    pthread_mutex_lock(&usrf->lock);
    while ((alt = usrf->pulsewidth / 147) > 6)
    {
        pthread_mutex_unlock(&usrf->lock);

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

        usleep(1000000);
    }
    
    pthread_mutex_unlock(&usrf->lock);

    landing_sigs.alt = -0.05;
    for (i = 0; i < 5; i++)
    {
        flight_control(&landing_sigs, VCM_AXIS_ALT);
        usleep(500000);
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
int fc_open_controls(gpio_event_t *pwm_usrf, imu_data_t *ypr_imu)
{
    thro_last_value = 0.0f;
    thro_last_cmp = 0, thro_first = 0;

    // direct flight control to ultrasonic sensor
    usrf = pwm_usrf;

    // direct flight control to IMU data array
    imu = ypr_imu;

    pthread_mutex_init(&fc_alive_event, NULL);
    pthread_mutex_init(&fc_vcm_event, NULL);
    pthread_mutex_init(&fc_cond_ap_mutex, NULL);

    if (0 > (g_channels[PWM_ALT].handle = pwm_open_device(PWM_DEV_ALT)))
    {
        syslog(LOG_ERR, "Error opening pwm device %d\n", PWM_DEV_ALT);
        return 0; 
    }
    syslog(LOG_INFO, "flight control: altitude channel opened\n");

    // keep throttle signal at the current value it is
    pwm_set_freq_x100(g_channels[PWM_ALT].handle, 4580);
    pwm_get_range(g_channels[PWM_ALT].handle, &g_channels[PWM_ALT].rng_min,
        &g_channels[PWM_ALT].rng_max);
    assign_duty(&g_channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, ALT_DUTY_LO);

    if (0 > (g_channels[PWM_PITCH].handle = pwm_open_device(PWM_DEV_PITCH)))
    {
        syslog(LOG_ERR, "Error opening pwm device %d\n", PWM_DEV_PITCH);
        return 0; 
    }
    syslog(LOG_INFO, "flight control: pitch channel opened\n");

    pwm_set_freq_x100(g_channels[PWM_PITCH].handle, 4580);
    pwm_get_range(g_channels[PWM_PITCH].handle, &g_channels[PWM_PITCH].rng_min,
            &g_channels[PWM_PITCH].rng_max);
    assign_duty(&g_channels[PWM_PITCH], PITCH_DUTY_LO, PITCH_DUTY_HI, PWM_DUTY_IDLE);

    if (0 > (g_channels[PWM_ROLL].handle = pwm_open_device(PWM_DEV_ROLL)))
    {
        syslog(LOG_ERR, "Error opening pwm device %d\n", PWM_DEV_ROLL);
        return 0;
    }
    syslog(LOG_INFO, "flight control: roll channel opened\n");

    pwm_set_freq_x100(g_channels[PWM_ROLL].handle, 4580);
    pwm_get_range(g_channels[PWM_ROLL].handle, &g_channels[PWM_ROLL].rng_min,
        &g_channels[PWM_ROLL].rng_max);
    assign_duty(&g_channels[PWM_ROLL], ROLL_DUTY_LO, ROLL_DUTY_HI, PWM_DUTY_IDLE);

    if (0 > (g_channels[PWM_YAW].handle = pwm_open_device(PWM_DEV_YAW)))
    {
        syslog(LOG_ERR, "Error opening pwm device %d\n", PWM_DEV_YAW);
        return 0;
    }
    syslog(LOG_INFO, "flight control: yaw channel opened\n");

    pwm_set_freq_x100(g_channels[PWM_YAW].handle, 4580);
    pwm_get_range(g_channels[PWM_YAW].handle, &g_channels[PWM_YAW].rng_min,
            &g_channels[PWM_YAW].rng_max);
    assign_duty(&g_channels[PWM_YAW], YAW_DUTY_LO, YAW_DUTY_HI, PWM_DUTY_IDLE);

    fc_alive = 1;
    syslog(LOG_INFO, "opened pwm device nodes\n");
    return 1;
}

// -----------------------------------------------------------------------------
void fc_close_controls()
{
    int i;

    // set flag for all controllers to see that flight control is shutting down
    pthread_mutex_lock(&fc_alive_event);
    fc_alive = 0;
    pthread_mutex_unlock(&fc_alive_event);

    // TODO: Wait for controller threads to exit

    for (i = 0; i < 4; ++i)
        pwm_close_device(g_channels[i].handle);

    usrf = NULL;

    pthread_mutex_destroy(&fc_alive_event);
    pthread_mutex_destroy(&fc_vcm_event);
    pthread_mutex_destroy(&fc_cond_ap_mutex);
}

// -----------------------------------------------------------------------------
void fc_takeoff()
{
    if (fc_alive)
        pthread_create(&takeoff_thrd, NULL, takeoff, NULL);
}

// -----------------------------------------------------------------------------
void fc_land()
{
    if (fc_alive)
        pthread_create(&land_thrd, NULL, land, NULL);
}

// -----------------------------------------------------------------------------
void fc_update_vcm(int axes, int type)
{
    pthread_mutex_lock(&fc_vcm_event);
    vcm_axes = axes;
    vcm_type = type;
    pthread_mutex_unlock(&fc_vcm_event);
}

// -----------------------------------------------------------------------------
void flight_control(ctl_sigs_t *sigs, int chnl_flags)
{
    int fc_on;

    pthread_mutex_lock(&fc_alive_event);
    fc_on = fc_alive;
    pthread_mutex_unlock(&fc_alive_event);

    if (fc_on)
    {
        // adjust altitude if specified
        if (chnl_flags & VCM_AXIS_ALT)
        {
            assign_value(&g_channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, sigs->alt, 0);
        }
       
        // adjust pitch if specified
        if (chnl_flags & VCM_AXIS_PITCH)
        {
            assign_value(&g_channels[PWM_PITCH], PITCH_DUTY_LO, PITCH_DUTY_HI, sigs->pitch, 0);
        }
       
        // adjust roll if specified
        if (chnl_flags & VCM_AXIS_ROLL)
        {
            assign_value(&g_channels[PWM_ROLL], ROLL_DUTY_LO, ROLL_DUTY_HI, sigs->roll, 0);
        }
       
        // adjust yaw if specified
        if (chnl_flags & VCM_AXIS_YAW)
        {
            assign_value(&g_channels[PWM_YAW], YAW_DUTY_LO, YAW_DUTY_HI, sigs->yaw, 0);
        }
    }
}

