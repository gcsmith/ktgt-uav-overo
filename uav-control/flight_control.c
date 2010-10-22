#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include "flight_control.h"
#include "pid.h"

#define ALT_DUTY_LO 0.046f
#define ALT_DUTY_HI 0.09f
#define ALT_DUTY_IDLE 0.07f

#define PITCH_DUTY_LO 0.057f
#define PITCH_DUTY_HI 0.09f
#define PITCH_DUTY_IDLE 0.07f

#define ROLL_DUTY_LO 0.052f
#define ROLL_DUTY_HI 0.087f
#define ROLL_DUTY_IDLE 0.07f

#define YAW_DUTY_LO 0.0465f
#define YAW_DUTY_HI 0.0948f
#define YAW_DUTY_IDLE 0.07f

typedef struct fc_globals {
    pthread_t replay_thrd;
    pthread_t takeoff_thrd;
    pthread_t land_thrd;
    pthread_t auto_thrd;

    pthread_cond_t takeoff_cond;

    pthread_mutex_t alive_lock;
    pthread_mutex_t vcm_lock;
    pthread_mutex_t takeoff_cond_lock;

    pwm_channel_t channels[4];
    gpio_event_t *usrf; // ultrasonic range finder pwm
    imu_data_t *imu;    // imu sensor

    int vcm_axes;
    int vcm_type;
    int alive;

    float thro_last_value;
    int thro_last_cmp;
    int thro_first;

    const char *capture_path;
    const char *replay_path;

    record_bucket_t *record_head, *record_tail;
    timespec_t last_time;
} fc_globals_t;

static fc_globals_t globals = { 0 };

// -----------------------------------------------------------------------------
record_bucket_t *record_create_bucket()
{
    record_bucket_t *bkt = (record_bucket_t *)malloc(sizeof(record_bucket_t));
    bkt->count = 0;
    bkt->next = NULL;
    return bkt;
}

// -----------------------------------------------------------------------------
int fc_get_alive(void)
{
    int alive = 0;
    pthread_mutex_lock(&globals.alive_lock);
    alive = globals.alive;
    pthread_mutex_unlock(&globals.alive_lock);
    return alive;
}

// -----------------------------------------------------------------------------
void fc_set_alive(int alive)
{
    pthread_mutex_lock(&globals.alive_lock);
    globals.alive = alive;
    pthread_mutex_unlock(&globals.alive_lock);
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

    // fprintf(stderr, "called assign_duty with %f %f %f\n", fmin, fmax, duty);
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
            //fprintf(stderr, "flight_control alt: value = %f, lv = %f\n",
              //      value, globals.thro_last_value);
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

            //fprintf(stderr, "flight_control alt: value = %f, lv = %f, cmp = %d, lc = %d\n", 
             //       value, globals.thro_last_value, cmp, globals.thro_last_cmp);
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

    pwm_set_compare(pwm->handle, cmp);
}

// -----------------------------------------------------------------------------
void flight_control(ctl_sigs_t *sigs, int chnl_flags)
{
    if (fc_get_alive())
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

    fprintf(stderr, "autopilot waiting...\n");
    
    pthread_mutex_lock(&globals.takeoff_cond_lock);
    pthread_cond_wait(&globals.takeoff_cond, &globals.takeoff_cond_lock);
    pthread_mutex_unlock(&globals.takeoff_cond_lock);

    fprintf(stderr, "autopilot starting...\n");

    //pthread_mutex_lock(&globals.alive_lock);
    while (fc_get_alive() && !kill_reached)
    {
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

    fprintf(stderr, "autopilot thread exiting\n");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
void *replay_thread(void *arg)
{
    record_bucket_t *bucket;
    input_record_t *record;
    int i;

    fprintf(stderr, "FLIGHT CONTROL: executing replay_thread\n");

    bucket = globals.record_head;
    while (NULL != bucket) {

        for (i = 0; i < bucket->count; i++) {
            record = &bucket->records[i];
            clock_nanosleep(CLOCK_REALTIME, 0, &record->delta, 0);
            flight_control(&record->signals, VCM_AXIS_ALL);
        }

        bucket = bucket->next;
    }

    fprintf(stderr, "completed replay_thread\n");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
void *takeoff_thread(void *arg)
{
    int error, input, last_input = 0, dx_dt, setpoint;
    float last_control = 0.0f;
    ctl_sigs_t control;

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
    pthread_create(&globals.auto_thrd, NULL, autopilot_thread, NULL);

    setpoint = 42; // 42 inches ~= 1 meter

    while (fc_get_alive() && ((input = (gpio_event_read(globals.usrf) / 147)) != setpoint))
    {
        fprintf(stderr, "gpio pw = %d\n", input);

        // dead reckoning variables
        error = setpoint - input;
        fprintf(stderr, "error = %d", error);
        if (last_input == 0)
            last_input = input;
        dx_dt = input - last_input; // rate of climb [inches per second]
        last_input = input;
#if 0
        // if the helicopter is 20 inches from the setpoint, switch to PID control
        if ((error <= (setpoint - 20)) && (error >= (setpoint + 20)))
        {
            // turn the altitude VCM bit off to indicate autonomous control
            pthread_mutex_lock(&globals.vcm_lock);
            globals.vcm_axes &= ~(VCM_AXIS_ALT);
            pthread_mutex_unlock(&globals.vcm_lock);
        }
#endif
        if (error > 0)
        {
            // need to climb
            if (dx_dt < 1)
            {
                fprintf(stderr, "need to climb\n");
                control.alt = 0.20;
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

        // sleep to allow the helicopter to lift
        usleep(50000);
    }

    pthread_mutex_lock(&globals.takeoff_cond_lock);
    pthread_cond_broadcast(&globals.takeoff_cond);
    pthread_mutex_unlock(&globals.takeoff_cond_lock);

    fprintf(stderr, "takeoff thread exiting\n");
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
int init_channel(int index, int freq, float duty_lo, float duty_hi, float duty_idle)
{
    pwm_channel_t *pwm = &globals.channels[index];
    if (0 > (pwm->handle = pwm_open_device(PWM_DEV_FIRST + index))) {
        syslog(LOG_ERR, "error opening pwm %d device\n", index);
        return 0; 
    }

    // keep throttle signal at the current value it is
    pwm_set_freq_x100(pwm->handle, freq);
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
    pthread_cond_init(&globals.takeoff_cond, NULL);

    if (!init_channel(PWM_ALT, 4580, ALT_DUTY_LO, ALT_DUTY_HI, ALT_DUTY_LO))
        return 0;
    syslog(LOG_INFO, "flight control: altitude channel opened\n");

    if (!init_channel(PWM_PITCH, 4580, PITCH_DUTY_LO, PITCH_DUTY_HI, PITCH_DUTY_IDLE))
        return 0; 
    syslog(LOG_INFO, "flight control: pitch channel opened\n");

    if (!init_channel(PWM_ROLL, 4580, ROLL_DUTY_LO, ROLL_DUTY_HI, ROLL_DUTY_IDLE))
        return 0;
    syslog(LOG_INFO, "flight control: roll channel opened\n");

    if (!init_channel(PWM_YAW, 4581, YAW_DUTY_LO, YAW_DUTY_HI, YAW_DUTY_IDLE))
        return 0;
    syslog(LOG_INFO, "flight control: yaw channel opened\n");

    fc_set_alive(1);
    syslog(LOG_INFO, "opened pwm device nodes\n");
    return 1;
}

// -----------------------------------------------------------------------------
void fc_shutdown()
{
    int i;

    // set flag for all controllers to see that flight control is shutting down
    fc_set_alive(0);

    // TODO: Wait for controller threads to exit

    for (i = 0; i < 4; ++i)
        pwm_close_device(globals.channels[i].handle);

    globals.usrf = NULL;

    pthread_mutex_destroy(&globals.alive_lock);
    pthread_mutex_destroy(&globals.vcm_lock);
    pthread_cond_destroy(&globals.takeoff_cond);

    if (globals.capture_path) {
        record_bucket_t *bucket, *curr;
        FILE *fout;

        // attempt to open the output file for writing
        fout = fopen(globals.capture_path, "w");
        if (!fout) {
            syslog(LOG_ERR, "failed to open %s\n", globals.capture_path);
            return;
        }
        syslog(LOG_INFO, "dumping capture to %s\n", globals.capture_path);

        // iterate over and dump out each record bucket
        bucket = globals.record_head;
        while (NULL != bucket) {
            curr = bucket;

            // write the current bucket out to the file
            for (i = 0; i < curr->count; i++) {
                input_record_t *record = &curr->records[i];
                fprintf(fout, "%ld %ld %f %f %f %f\n",
                        record->delta.tv_sec, record->delta.tv_nsec,
                        record->signals.alt, record->signals.yaw,
                        record->signals.pitch, record->signals.roll);
            }

            bucket = bucket->next;
            free(curr);
        }
        fclose(fout);
    }
}

// -----------------------------------------------------------------------------
int fc_set_capture(const char *path)
{
    // do some basic checking for erroneous requests
    if (globals.capture_path) {
        syslog(LOG_ERR, "already set capture path in flight_control");
        return 0;
    }
    if (globals.replay_path) {
        syslog(LOG_ERR, "cannot specify both capture and replay paths");
        return 0;
    }

    globals.capture_path = path;

    // allocate the first record bucket in our linked list
    globals.record_head = globals.record_tail = record_create_bucket();
    return 1;
}

// -----------------------------------------------------------------------------
int fc_set_replay(const char *path)
{
    FILE *fin;
    int entries_read = 0, buckets_filled = 1;
    record_bucket_t *bucket;
    input_record_t *record;
    timespec_t delta;

    // do some basic checking for erroneous requests
    if (globals.replay_path) {
        syslog(LOG_ERR, "already set replay path in flight_control");
        return 0;
    }
    if (globals.capture_path) {
        syslog(LOG_ERR, "cannot specify both replay and capture paths");
        return 0;
    }

    fin = fopen(path, "r");
    if (!fin) {
        syslog(LOG_ERR, "failed to open %s for reading", path);
        return 0;
    }

    globals.record_head = globals.record_tail = bucket = record_create_bucket();
    while (!feof(fin)) {
        float alt, yaw, pitch, roll;
        fscanf(fin, "%ld %ld %f %f %f %f\n", &delta.tv_sec, &delta.tv_nsec,
               &alt, &yaw, &pitch, &roll);

        if (bucket->count >= RECORD_BUCKET_SIZE) {
            // add a new bucket to the linked list if we're out of space
            bucket = record_create_bucket();
            globals.record_tail->next = bucket;
            globals.record_tail = bucket;
            ++buckets_filled;
        }

        // store the loaded input record
        record = &bucket->records[bucket->count++];
        record->delta.tv_sec = delta.tv_sec;
        record->delta.tv_nsec = delta.tv_nsec;
        record->signals.alt = alt;
        record->signals.yaw = yaw;
        record->signals.pitch = pitch;
        record->signals.roll = roll;
        ++entries_read;
    }
    fclose(fin);

    syslog(LOG_INFO, "successfully loaded %d input records into %d buckets\n",
           entries_read, buckets_filled);
    globals.replay_path = path;
    return 1;
}

// -----------------------------------------------------------------------------
int fc_takeoff()
{
    if (!fc_get_alive()) {
        syslog(LOG_ERR, "flight_control cannot takeoff in dead state");
        return 0;
    }

    if (globals.replay_path) {
        pthread_create(&globals.replay_thrd, NULL, replay_thread, NULL);
    }
    else {
        pthread_create(&globals.takeoff_thrd, NULL, takeoff_thread, NULL);
    }
    return 1;
}

// -----------------------------------------------------------------------------
int fc_land()
{
    if (!fc_get_alive()) {
        syslog(LOG_ERR, "flight_controll cannot land in dead state");
        return 0;
    }

    pthread_create(&globals.land_thrd, NULL, landing_thread, NULL);
    return 1;
}

// -----------------------------------------------------------------------------
void fc_reset_channels(void)
{
    globals.thro_last_value = 0.0f;
    globals.thro_last_cmp = 0;
    globals.thro_first = 0;
    assign_duty(&globals.channels[PWM_ALT], ALT_DUTY_LO, ALT_DUTY_HI, ALT_DUTY_LO);
    assign_duty(&globals.channels[PWM_YAW], YAW_DUTY_LO, YAW_DUTY_HI, YAW_DUTY_IDLE);
    assign_duty(&globals.channels[PWM_PITCH], PITCH_DUTY_LO, PITCH_DUTY_HI, PITCH_DUTY_IDLE);
    assign_duty(&globals.channels[PWM_ROLL], ROLL_DUTY_LO, ROLL_DUTY_HI, ROLL_DUTY_IDLE);
}

// -----------------------------------------------------------------------------
void fc_update_vcm(int axes, int type)
{
    int curr_type;

    pthread_mutex_lock(&globals.vcm_lock);
    curr_type = globals.vcm_type;
    pthread_mutex_unlock(&globals.vcm_lock);

    if (VCM_TYPE_KILL == curr_type)
    {
        // if we're killed, don't allow any more state transitions
        fprintf(stderr, "not alive. ignoring fc_update_vcm\n");
        return;
    }

    // otherwise, set the new type and axes and continue
    pthread_mutex_lock(&globals.vcm_lock);
    globals.vcm_type = type;
    globals.vcm_axes = axes;
    pthread_mutex_unlock(&globals.vcm_lock);

    fc_reset_channels();
    if (VCM_TYPE_KILL == type || VCM_TYPE_LOCKOUT == type)
        fc_set_alive(0);
    else
        fc_set_alive(1);

    // save the last timer tick every time we switch state
    clock_gettime(CLOCK_REALTIME, &globals.last_time);
}

// -----------------------------------------------------------------------------
void fc_update_ctl(ctl_sigs_t *sigs)
{
    int axes;
    timespec_t curr_time, time_delta;

    pthread_mutex_lock(&globals.vcm_lock);
    axes = globals.vcm_axes;
    pthread_mutex_unlock(&globals.vcm_lock);

    // if the incoming signal is a throttle event from the client, then we
    // target just the altitude control signal so we don't disturb the other
    // control signals
    fprintf(stderr, "flight control's axes: %d\n", axes);
    flight_control(sigs, axes);

    // if --capture is enabled, store this control signal
    if (globals.capture_path) {
        record_bucket_t *bucket = globals.record_tail;
        if (bucket->count >= RECORD_BUCKET_SIZE) {
            // attach this bucket to the current tail of our linked list
            bucket = record_create_bucket();
            globals.record_tail->next = bucket;
            globals.record_tail = bucket;
        }

        // record the delta time between controller inputs
        clock_gettime(CLOCK_REALTIME, &curr_time);
        timespec_diff(&globals.last_time, &curr_time, &time_delta);
        globals.last_time = curr_time;

        // insert the record into our bucket
        bucket->records[bucket->count].delta = time_delta;
        bucket->records[bucket->count].signals = *sigs;
        bucket->count++;
    }
}

// -----------------------------------------------------------------------------
void fc_get_vcm(int *axes, int *type)
{
    *axes = globals.vcm_axes;
    *type = globals.vcm_type;
}

