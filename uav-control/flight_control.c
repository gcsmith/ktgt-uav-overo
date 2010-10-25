#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

// #define DBG_DISABLE_PID
#define DBG_USE_RELATIVE

typedef struct fc_globals {
    pthread_t replay_thrd;
    pthread_t takeoff_thrd;
    pthread_t land_thrd;
    pthread_t auto_thrd;

    pthread_cond_t  takeoff_cond;
    pthread_mutex_t alive_lock;
    pthread_mutex_t vcm_lock;
    pthread_mutex_t takeoff_cond_lock;

    pwm_channel_t channels[4];
    gpio_event_t *usrf; // ultrasonic range finder pwm
    imu_data_t *imu;    // imu sensor

    int vcm_axes;
    int vcm_type;
    int alive;

    int thro_last_cmp;
    int thro_first;

    const char *capture_path;
    const char *replay_path;

    record_bucket_t *record_head, *record_tail;
    timespec_t last_time;
} fc_globals_t;

static fc_globals_t globals = { 0 };

// -----------------------------------------------------------------------------
static record_bucket_t *record_create_bucket(void)
{
    record_bucket_t *bkt = (record_bucket_t *)malloc(sizeof(record_bucket_t));
    bkt->count = 0;
    bkt->next = NULL;
    return bkt;
}

// -----------------------------------------------------------------------------
static void record_insert_sigs(const ctl_sigs_t *sigs)
{
    timespec_t curr_time, time_delta;
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

// -----------------------------------------------------------------------------
static void record_write_buckets(void)
{
    record_bucket_t *bucket, *curr;
    FILE *fout;
    int i;

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

// -----------------------------------------------------------------------------
static int fc_get_alive(void)
{
    int alive = 0;
    pthread_mutex_lock(&globals.alive_lock);
    alive = globals.alive;
    pthread_mutex_unlock(&globals.alive_lock);
    return alive;
}

// -----------------------------------------------------------------------------
static void fc_set_alive(int alive)
{
    pthread_mutex_lock(&globals.alive_lock);
    globals.alive = alive;
    pthread_mutex_unlock(&globals.alive_lock);
}

// -----------------------------------------------------------------------------
static void assign_duty(pwm_channel_t *pwm, float duty)
{
    int cmp_val = 0;
    
    duty = CLAMP(duty, pwm->duty_lo, pwm->duty_hi);
    cmp_val = (int)(pwm->rng_min + (int)((pwm->rng_max - pwm->rng_min) * duty));
    pwm_set_compare(pwm->handle, cmp_val);
}

// -----------------------------------------------------------------------------
static void assign_value_rel(pwm_channel_t *pwm, float val)
{
    int cmp, max, min;
    unsigned int range, hrange;

    range = pwm->rng_max - pwm->rng_min;
    min = pwm->rng_min + (int)(range * pwm->duty_lo);
    max = pwm->rng_min + (int)(range * pwm->duty_hi);
    hrange = (max - min) >> 1;

    // thro_last_cmp should be set to the current PWM cmp here to avoid
    // checking cmp == 0 at the end of this function.

    if (globals.channels[PWM_ALT].handle == pwm->handle) {
        // on the first pass set we need to modify the current PWM signal
        if (globals.thro_first == 0) {
            globals.thro_first = 1;
            cmp = min + (int)(hrange * val);
        }
        else {
            cmp = globals.thro_last_cmp + (int)(hrange * val); // * 0.05f);
         }
        cmp = CLAMP(cmp, min, max);
        globals.thro_last_cmp = cmp;
    }
    else {
        cmp = min + hrange + (int)(hrange * val);
    }

    // adjust the trim, but keep within the absolute limits of this pwm channel
    pwm->cmp = CLAMP(cmp, pwm->rng_min, pwm->rng_max);
    pwm_set_compare(pwm->handle, pwm->cmp + pwm->trim);
}

// -----------------------------------------------------------------------------
static void assign_value_abs(pwm_channel_t *pwm, float val)
{
    int cmp, max, min;
    unsigned int range, hrange;

    range = pwm->rng_max - pwm->rng_min;
    min = pwm->rng_min + (int)(range * pwm->duty_lo);
    max = pwm->rng_min + (int)(range * pwm->duty_hi);
    hrange = (max - min) >> 1;

    // thro_last_cmp should be set to the current PWM cmp here to avoid
    // checking cmp == 0 at the end of this function.

    if (globals.channels[PWM_ALT].handle == pwm->handle)
        cmp = min + (int)(range * val);
    else
        cmp = min + hrange + (int)(hrange * val);

    // adjust the trim, but keep within the absolute limits of this pwm channel
    pwm->cmp = CLAMP(cmp, pwm->rng_min, pwm->rng_max);
    pwm_set_compare(pwm->handle, pwm->cmp + pwm->trim);
}

// -----------------------------------------------------------------------------
static int fc_control_rel(const ctl_sigs_t *sigs, int chnl_flags)
{
    if (!fc_get_alive())
        return 0;

    if (chnl_flags & VCM_AXIS_ALT)
        assign_value_rel(&globals.channels[PWM_ALT], sigs->alt);
    if (chnl_flags & VCM_AXIS_PITCH)
        assign_value_rel(&globals.channels[PWM_PITCH], sigs->pitch);
    if (chnl_flags & VCM_AXIS_ROLL)
        assign_value_rel(&globals.channels[PWM_ROLL], sigs->roll);
    if (chnl_flags & VCM_AXIS_YAW)
        assign_value_rel(&globals.channels[PWM_YAW], sigs->yaw);
    return 1;
}

// -----------------------------------------------------------------------------
static int fc_control_abs(const ctl_sigs_t *sigs, int chnl_flags)
{
    if (!fc_get_alive())
        return 0;

    if (chnl_flags & VCM_AXIS_ALT)
        assign_value_abs(&globals.channels[PWM_ALT], sigs->alt);
    if (chnl_flags & VCM_AXIS_PITCH)
        assign_value_abs(&globals.channels[PWM_PITCH], sigs->pitch);
    if (chnl_flags & VCM_AXIS_ROLL)
        assign_value_abs(&globals.channels[PWM_ROLL], sigs->roll);
    if (chnl_flags & VCM_AXIS_YAW)
        assign_value_abs(&globals.channels[PWM_YAW], sigs->yaw);
    return 1;
}

// -----------------------------------------------------------------------------
static void *autopilot_thread(void *arg)
{
    int vcm_axes, vcm_type;
    float pid_result, curr_error;

    // flight dynamics
    //float fd_pitch, fd_alt; // , fd_yaw, fd_roll;
    float fd_alt;

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
    pid_alt_ctlr.Kp          = 0.00025;
    pid_alt_ctlr.Ki          = 0.000f;
    pid_alt_ctlr.Kd          = 0.000f;
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
    while (fc_get_alive()) {
        int axes = VCM_AXIS_ALL;

#if 0
        // capture pitch
        pthread_mutex_lock(&globals.imu->lock);
        //fd_yaw   = globals.imu->angles[IMU_DATA_YAW];
        fd_pitch = globals.imu->angles[IMU_DATA_PITCH];
        //fd_roll  = globals.imu->angles[IMU_DATA_ROLL];
        pthread_mutex_unlock(&globals.imu->lock);
#endif

        // capture altitude
        fd_alt = gpio_event_read(globals.usrf, ACCESS_SYNC) / 147;

        // capture flight control's mode and vcm axes
        pthread_mutex_lock(&globals.vcm_lock);
        vcm_axes = globals.vcm_axes;
        vcm_type = globals.vcm_type;
        pthread_mutex_unlock(&globals.vcm_lock);

        // reset all signals if flight control is of type kill
        switch (vcm_type) {
        case VCM_TYPE_KILL:
            fprintf(stderr, "vcm_type = kill\n");
            // assign_value_abs(&globals.channels[PWM_ALT], 0);
            // assign_value_abs(&globals.channels[PWM_PITCH], 0);
            // assign_value_abs(&globals.channels[PWM_ROLL], 0);
            // assign_value_abs(&globals.channels[PWM_YAW], 0);
            break;
        case VCM_TYPE_AUTO:
            fprintf(stderr, "vcm_type = auto\n");
            axes = 0;
            break;
        case VCM_TYPE_MIXED:
            fprintf(stderr, "vcm_type = mixed\n");
            axes = vcm_axes;
            break;
        }
            
#if 0
        // check pitch bit is 0 for autonomous control
        if (!(axes & VCM_AXIS_PITCH)) {
            pid_compute(&pid_pitch_ctlr, fd_pitch, &curr_error, &pid_result);
            fc_sigs.pitch = pid_result;
            fc_control_abs(&fc_sigs, VCM_AXIS_PITCH);
        }
#endif

        // check altitude bit is 0 for autonomous control
        //if (!(axes & VCM_AXIS_ALT)) {
            pid_compute(&pid_alt_ctlr, fd_alt, &curr_error, &pid_result);
            fc_sigs.alt = pid_result;
            fprintf(stderr, "pid set alt to %f\n", fc_sigs.alt);
#ifndef DBG_USE_RELATIVE
            fc_control_abs(&fc_sigs, VCM_AXIS_ALT);
#else
            fc_control_rel(&fc_sigs, VCM_AXIS_ALT);
#endif
        //}
    }

    fprintf(stderr, "autopilot thread exiting\n");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
static void *takeoff_thread(void *arg)
{
#if 0
    int error, input, last_input = 0, dx_dt, setpoint;
    float last_control = 0.0f;
#endif
    int i, setpoint;
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

    // spawn autopilot thread
    pthread_create(&globals.auto_thrd, NULL, autopilot_thread, NULL);
    pthread_mutex_unlock(&globals.vcm_lock);

    memset(&control, 0, sizeof(control));
    setpoint = 42; // 42 inches ~= 1 meter

#ifdef DBG_USE_RELATIVE
    for (i = 0; i < 575; i++) {
        control.alt = 0.04 * .05f;
        fc_control_rel(&control, VCM_AXIS_ALT);
        usleep(10000);
    }
    printf("done ramping (rel) -- switching to pid autopilot thread\n");
#else
    control.alt = 0.0f;
    for (i = 0; i < 575; i++) {
        control.alt += 0.00005;
        fc_control_abs(&control, VCM_AXIS_ALT);
        usleep(10000);
    }
    printf("done ramping (abs) -- switching to pid autopilot thread\n");
#endif

#ifndef DBG_DISABLE_PID
    pthread_mutex_lock(&globals.takeoff_cond_lock);
    pthread_cond_broadcast(&globals.takeoff_cond);
    pthread_mutex_unlock(&globals.takeoff_cond_lock);
#endif

    fprintf(stderr, "takeoff thread exiting\n");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
static void *replay_thread(void *arg)
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
            fc_control_rel(&record->signals, VCM_AXIS_ALL);
        }

        bucket = bucket->next;
    }

    fprintf(stderr, "completed replay_thread\n");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
static void *landing_thread(void *arg)
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
    while ((alt = gpio_event_read(globals.usrf, ACCESS_SYNC) / 147) > 6) {
        if (prev_alt == 0)
            prev_alt = alt;
        dx_dt = alt - prev_alt;
        prev_alt = alt;

        if (dx_dt == 0) {
            landing_sigs.alt = -0.1f;
            fc_control_abs(&landing_sigs, VCM_AXIS_ALT);
        }
        else if (dx_dt <= -3) {
            landing_sigs.alt = 0.05f;
            fc_control_abs(&landing_sigs, VCM_AXIS_ALT);
        }

        usleep(50000);
    }
    
    landing_sigs.alt = -0.05;
    for (i = 0; i < 5; i++) {
        fc_control_abs(&landing_sigs, VCM_AXIS_ALT);
        usleep(100000);
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
static int init_channel(int index, int freq, float lo, float hi, float idle)
{
    pwm_channel_t *pwm = &globals.channels[index];
    if (0 > (pwm->handle = pwm_open_device(PWM_DEV_FIRST + index))) {
        syslog(LOG_ERR, "error opening pwm %d device\n", index);
        return 0; 
    }

    // reset the trim to the dead center of the duty cycle range
    pwm->trim = 0;
    pwm->duty_lo = lo;
    pwm->duty_hi = hi;

    // keep throttle signal at the current value it is
    pwm_set_freq_x100(pwm->handle, freq);
    pwm_get_range(pwm->handle, &pwm->rng_min, &pwm->rng_max);
    assign_duty(pwm, idle);
    return 1;
}

// -----------------------------------------------------------------------------
int fc_init(gpio_event_t *pwm_usrf, imu_data_t *ypr_imu)
{
    globals.vcm_type = VCM_TYPE_AUTO;
    globals.vcm_axes = VCM_AXIS_ALL;

    globals.thro_last_cmp = 0;
    globals.thro_first = 0;

    // direct flight control to ultrasonic sensor
    globals.usrf = pwm_usrf;

    // direct flight control to IMU data array
    globals.imu = ypr_imu;

    pthread_mutex_init(&globals.alive_lock, NULL);
    pthread_mutex_init(&globals.vcm_lock, NULL);
    pthread_cond_init(&globals.takeoff_cond, NULL);

    if (!init_channel(PWM_ALT, 4582, ALT_DUTY_LO, ALT_DUTY_HI, ALT_DUTY_LO))
        return 0;
    syslog(LOG_INFO, "flight control: altitude channel opened\n");

    if (!init_channel(PWM_PITCH, 4582, PITCH_DUTY_LO, PITCH_DUTY_HI, PITCH_DUTY_IDLE))
        return 0; 
    syslog(LOG_INFO, "flight control: pitch channel opened\n");

    if (!init_channel(PWM_ROLL, 4582, ROLL_DUTY_LO, ROLL_DUTY_HI, ROLL_DUTY_IDLE))
        return 0;
    syslog(LOG_INFO, "flight control: roll channel opened\n");

    if (!init_channel(PWM_YAW, 4582, YAW_DUTY_LO, YAW_DUTY_HI, YAW_DUTY_IDLE))
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
        // save and destroy the record buckets
        record_write_buckets();
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
        // attempt to parse the current line
        float alt, yaw, pitch, roll;
        if (6 != fscanf(fin, "%ld %ld %f %f %f %f\n", &delta.tv_sec,
                    &delta.tv_nsec, &alt, &yaw, &pitch, &roll)) {
            // skip this line and continue on if incorrectly formatted
            syslog(LOG_ERR, "invalid line detected in replay file\n");
            continue;
        }

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
    pwm_channel_t *ch = &globals.channels[0];
    globals.thro_last_cmp = 0;
    globals.thro_first = 0;

    assign_duty(&ch[PWM_ALT], ALT_DUTY_LO);
    assign_duty(&ch[PWM_YAW], YAW_DUTY_IDLE);
    assign_duty(&ch[PWM_PITCH], PITCH_DUTY_IDLE);
    assign_duty(&ch[PWM_ROLL], ROLL_DUTY_IDLE);
}

// -----------------------------------------------------------------------------
void fc_set_vcm(int axes, int type)
{
    int curr_type;

    pthread_mutex_lock(&globals.vcm_lock);
    curr_type = globals.vcm_type;
    pthread_mutex_unlock(&globals.vcm_lock);

    if (VCM_TYPE_KILL == curr_type) {
        // if we're killed, don't allow any more state transitions
        fprintf(stderr, "not alive. ignoring fc_set_vcm\n");
        return;
    }

    // otherwise, set the new type and axes and continue
    pthread_mutex_lock(&globals.vcm_lock);
    globals.vcm_type = type;
    globals.vcm_axes = axes;
    pthread_mutex_unlock(&globals.vcm_lock);

    fc_reset_channels();
    fc_set_alive((VCM_TYPE_KILL != type) && (VCM_TYPE_LOCKOUT != type));

    // save the last timer tick every time we switch state
    clock_gettime(CLOCK_REALTIME, &globals.last_time);
}

// -----------------------------------------------------------------------------
void fc_set_ctl(ctl_sigs_t *sigs)
{
    int axes;

    pthread_mutex_lock(&globals.vcm_lock);
    axes = globals.vcm_axes;

    // scale down the throttle signal, as the default [-1, 1] is too sensitive
    sigs->alt *= 0.05f;

    fc_control_rel(sigs, axes);
    pthread_mutex_unlock(&globals.vcm_lock);

    if (globals.capture_path) {
        // if --capture is enabled, store this control signal
        record_insert_sigs(sigs);
    }
}

// -----------------------------------------------------------------------------
void fc_get_vcm(int *axes, int *type)
{
    *axes = globals.vcm_axes;
    *type = globals.vcm_type;
}

// -----------------------------------------------------------------------------
void fc_set_trims(int axes, int value)
{
    if (axes & VCM_AXIS_ALT) {
        pwm_channel_t *alt = &globals.channels[PWM_ALT];
        alt->trim = value;
        pwm_set_compare(alt->handle, alt->cmp + alt->trim);
        syslog(LOG_INFO, "setting ALT trim to %d\n", value);
    }
    if (axes & VCM_AXIS_YAW) {
        pwm_channel_t *yaw = &globals.channels[PWM_YAW];
        yaw->trim = value;
        pwm_set_compare(yaw->handle, yaw->cmp + yaw->trim);
        syslog(LOG_INFO, "setting YAW trim to %d\n", value);
    }
    if (axes & VCM_AXIS_PITCH) {
        pwm_channel_t *pitch = &globals.channels[PWM_PITCH];
        pitch->trim = value;
        pwm_set_compare(pitch->handle, pitch->cmp + pitch->trim);
        syslog(LOG_INFO, "setting PITCH trim to %d\n", value);
    }
    if (axes & VCM_AXIS_ROLL) {
        pwm_channel_t *roll = &globals.channels[PWM_ROLL];
        roll->trim = value;
        pwm_set_compare(roll->handle, roll->cmp + roll->trim);
        syslog(LOG_INFO, "setting ROLL trim to %d\n", value);
    }
}

// -----------------------------------------------------------------------------
int fc_get_trim(int axes)
{
    switch (axes) {
    case VCM_AXIS_ALT:
        return globals.channels[PWM_ALT].trim;
    case VCM_AXIS_YAW:
        return globals.channels[PWM_YAW].trim;
    case VCM_AXIS_PITCH:
        return globals.channels[PWM_PITCH].trim;
    case VCM_AXIS_ROLL:
        return globals.channels[PWM_ROLL].trim;
    default:
        return -1;
    }
}

