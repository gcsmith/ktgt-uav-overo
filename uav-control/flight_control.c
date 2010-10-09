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

#define THRO_SENS 10

// threads
pthread_t takeoff_thrd;
pthread_t land_thrd;

// mutexes
pthread_mutex_t fc_alive_event;
#if 0
pthread_mutex_t fc_signals_event;
#endif
pthread_mutex_t fc_axes_event;

// pwm channels to helicopter's controls
pwm_channel_t g_channels[4];

gpio_event_t *usrf; // ultrasonic range finder pwm

// axes flags
int vcm_axes;

// flight control's master signal carrier
#if 0
ctl_sigs_t fc_mr_ctl;
#endif

// flight control's alive flag
char fc_alive;

// flight control's autonomous control flag
char fc_allow_autonomous;

// relative throttle control variables
float thro_last_value = 0.0f;
int thro_last_cmp = 0, thro_first = 0;

// -----------------------------------------------------------------------------
void *takeoff()
{
    int error, input, last_input, dx_dt;
    float last_control;
    char stable = 0, timer_set = 0;
    ctl_sigs_t control;
    clock_t timer = 0;;

    pid_ctrl_t pid;
    pid.setpoint = 42;      // 42 inches ~= 1 meter
    pid.Kp = 0.01;
    pid.Ki = 0.1;
    pid.Kd = 0.001;
    pid.total_error = 0.0;

    control.alt = 0.0f;

    fprintf(stderr, "FLIGHT CONTROL: Helicopter, permission granted to take off\n");

    // We'll need the pitch controller for takeoff as well to account for the
    // helicopter being front or back heavy, this will ensure the helicopter
    // goes straight up
    while (!stable)
    {
        pthread_mutex_lock(&usrf->lock);
        while ((input = (usrf->pulsewidth / 147)) != pid.setpoint)
        {
            pthread_mutex_unlock(&usrf->lock);

            // dead reckoning variables
            error = pid.setpoint - input;
            last_input = input;
            dx_dt = input - last_input; // rate of climb [inches per second]

            // if the helicopter is 20 inches from the setpoint, switch to PID control
            if ((error <= 22) && (error >= -22))
            {
                pid_compute(&pid, (float)input, (float *)&error, &control.alt);
                // not sure how the output will look from pid controller
                fprintf(stderr, "pid output = %f\n", control.alt);
                //flight_control(&control, VCM_AXIS_ALT);
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
            // previous stability
            if (timer_set)
                timer_set = 0;
        }

        if ((input == pid.setpoint) && (!timer_set))
        {
            // get the time from now that the helicopter should still be at 
            // the setpoint to be considered stable
            timer = clock() + 5 * CLOCKS_PER_SEC;
            timer_set = 1;
        }
        else if ((input == pid.setpoint) && timer_set)
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
    pthread_exit(NULL);
}

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
    fprintf(stderr, "rng_max = %u\n", pwm->rng_max);
    fprintf(stderr, "rng_min = %u\n", pwm->rng_min);
    fprintf(stderr, "cmp_val = %u\n", cmp_val);
    pwm_set_compare(pwm->handle, cmp_val);
}

// -----------------------------------------------------------------------------
void assign_value(pwm_channel_t *pwm, float fmin, float fmax, float value)
{
    int cmp, max, min;
    unsigned int range, hrange;

    range = pwm->rng_max - pwm->rng_min;
    min = pwm->rng_min + (int)(range * fmin);
    max = pwm->rng_min + (int)(range * fmax);
    hrange = (max - min) >> 1;

    // thro_last_cmp should be set to the current PWM cmp here to avoid
    // checking cmp == 0 at the end of this function.

    fprintf(stderr, "pwm channel = %d\n", pwm->handle);
    fprintf(stderr, "alt chan = %d\n", g_channels[PWM_ALT].handle);
    fprintf(stderr, "pitch chan = %d\n", g_channels[PWM_PITCH].handle);
    fprintf(stderr, "roll chan = %d\n", g_channels[PWM_ROLL].handle);
    fprintf(stderr, "yaw chan = %d\n", g_channels[PWM_YAW].handle);
    if (g_channels[PWM_ALT].handle == pwm->handle)
    {
#if 0
        // joystick is scrolling from up to down
        if ((thro_last_value > value) && (thro_last_value > 0) && (value > 0))
        {
            cmp = thro_last_cmp;
        }

        // joystick is scrolling from down to up
        else if ((thro_last_value < value) && (thro_last_value < 0) && (value < 0))
        {
            cmp = thro_last_cmp;
        }

        // no delta
        else if (thro_last_value == value)
        {
            cmp = thro_last_cmp;
        }
#endif
        //else 
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
                // scale the value for sensitivity
#if 0
                if (value < 0.0f)
                    value = -0.10f;
                else
                    value = 0.10f;
#endif
                cmp = thro_last_cmp + (int)(hrange * value * 0.15f);
                if (cmp > max)
                    cmp = max;
                if (cmp < min)
                    cmp = min;
                fprintf(stderr, "flight_control alt: value = %f, lv = %f, cmp = %d, lc = %d\n", 
                    value, thro_last_value, cmp, thro_last_cmp);
                thro_last_cmp = cmp;
            }
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
int fc_open_controls(gpio_event_t *pwm_usrf)
{
    thro_last_value = 0.0f;
    thro_last_cmp = 0, thro_first = 0;

    // direct flight control to ultrasonic sensor
    usrf = pwm_usrf;

    // initial flight controls
#if 0
    fc_mr_ctl.alt   = 0.0f;
    fc_mr_ctl.pitch = 0.0f;
    fc_mr_ctl.roll  = 0.0f;
    fc_mr_ctl.yaw   = 0.0f;
#endif

    pthread_mutex_init(&fc_alive_event, NULL);
#if 0
    pthread_mutex_init(&fc_signals_event, NULL);
#endif
    pthread_mutex_init(&fc_axes_event, NULL);

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
#if 0
    pthread_mutex_destroy(&fc_signals_event);
#endif
    pthread_mutex_destroy(&fc_axes_event);
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
void alt_controller()
{
    char thread_done = 0;

    while (!thread_done)
    {
        pthread_mutex_lock(&fc_alive_event);

        if (fc_alive)
        {
            pthread_mutex_unlock(&fc_alive_event);

            if (!(vcm_axes & VCM_AXIS_ALT))
            {
                // altitude is controlled autonomously
            }
        }
        else
        {
            pthread_mutex_unlock(&fc_alive_event);
            thread_done = 1;
        }
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
void pitch_controller()
{
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
void fc_update_axes(int chnl_flags)
{
    pthread_mutex_lock(&fc_axes_event);
    vcm_axes = chnl_flags;
    pthread_mutex_unlock(&fc_axes_event);
}

// -----------------------------------------------------------------------------
void flight_control(ctl_sigs_t *sigs, int chnl_flags)
{
#if 0
    // assign incoming signals to fight control's signal carrier
    pthread_mutex_lock(&fc_signals_event);
    fc_mr_ctl.alt = sigs->alt;
    fc_mr_ctl.pitch = sigs->pitch;
    fc_mr_ctl.roll = sigs->roll;
    fc_mr_ctl.yaw = sigs->yaw;
    pthread_mutex_unlock(&fc_signals_event);
#endif

    if (fc_alive)
    {
        // adjust altitude if specified
        if (chnl_flags & VCM_AXIS_ALT)
        {
            assign_value(&g_channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, sigs->alt);
        }
       
        // adjust pitch if specified
        if (chnl_flags & VCM_AXIS_PITCH)
        {
            assign_value(&g_channels[PWM_PITCH], PITCH_DUTY_LO, PITCH_DUTY_HI, sigs->pitch);
        }
       
        // adjust roll if specified
        if (chnl_flags & VCM_AXIS_ROLL)
        {
            assign_value(&g_channels[PWM_ROLL], ROLL_DUTY_LO, ROLL_DUTY_HI, sigs->roll);
        }
       
        // adjust yaw if specified
        if (chnl_flags & VCM_AXIS_YAW)
        {
            assign_value(&g_channels[PWM_YAW], YAW_DUTY_LO, YAW_DUTY_HI, sigs->yaw);
        }
    }
}

