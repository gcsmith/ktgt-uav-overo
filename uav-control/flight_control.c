// -----------------------------------------------------------------------------
// File:    flight_control.c
// Authors: Kevin Macksamie, Garrett Smith, Tyler Thierolf, Timothy Miller
// Created: 09-23-2010
//
// Algorithms and state management for computer controlled flight.
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <syslog.h>
#include "tracking.h"
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
    pthread_t landing_thrd;
    pthread_t auto_alt_thrd;
    pthread_t auto_imu_thrd;

    pthread_mutex_t state_lock;
    pthread_mutex_t takeoff_lock;
    pthread_mutex_t landing_lock;
    pthread_mutex_t vcm_lock;
    pthread_mutex_t pid_lock;

    pthread_cond_t state_cond;
    pthread_cond_t takeoff_cond;
    pthread_cond_t landing_cond;

    pwm_channel_t channels[4];
    gpio_event_t *usrf;         // ultrasonic range finder pwm
    imu_data_t *imu;            // imu sensor

    int enabled;
    int capture_enabled;
    int state;
    int vcm_axes;
    int vcm_type;

    const char *capture_path;
    const char *replay_path;
    float curr_alt;
    float carry_over;

    record_bucket_t *record_head, *record_tail;
    timespec_t last_time;

    pid_ctrl_t pid_alt;
    pid_ctrl_t pid_yaw;
    pid_ctrl_t pid_pitch;
    pid_ctrl_t pid_roll;
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
        syslog(LOG_ERR, "failed to open %s", globals.capture_path);
        return;
    }
    syslog(LOG_INFO, "dumping capture to %s", globals.capture_path);

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
    }
    fclose(fout);
}

// -----------------------------------------------------------------------------
static void record_delete_buckets(void)
{
    record_bucket_t *bucket, *curr;

    bucket = globals.record_head;
    while (NULL != bucket) {
        curr = bucket;
        bucket = bucket->next;
        free(curr);
    }

    globals.record_head = NULL;
    globals.record_tail = NULL;
}

// -----------------------------------------------------------------------------
static void fc_set_state(int state)
{
    pthread_mutex_lock(&globals.state_lock);

    if (state != globals.state) {
        // set the new state and signal the state transition
        globals.state = state;
        pthread_cond_broadcast(&globals.state_cond);
    }

    pthread_mutex_unlock(&globals.state_lock);
}

// -----------------------------------------------------------------------------
static void assign_duty(pwm_channel_t *pwm, float duty)
{
    duty = CLAMP(duty, pwm->duty_lo, pwm->duty_hi);
    pwm->cmp = (int)(pwm->rng_min + (int)((pwm->rng_max - pwm->rng_min) * duty));
    pwm_set_compare(pwm->handle, pwm->cmp);
}

// -----------------------------------------------------------------------------
static void assign_value(pwm_channel_t *pwm, float val)
{
    int cmp, max, min;
    unsigned int cmp_range, range, hrange;

    cmp_range = pwm->rng_max - pwm->rng_min;
    min = pwm->rng_min + (int)(cmp_range * pwm->duty_lo);
    max = pwm->rng_min + (int)(cmp_range * pwm->duty_hi);
    range = max - min;
    hrange = range >> 1;

    if (globals.channels[PWM_ALT].handle == pwm->handle) {
        cmp = min + (int)(range * val);
        cmp = CLAMP(cmp, min, max);
    }
    else {
        cmp = min + hrange + (int)(hrange * val);
    }

    // adjust the trim, but keep within the absolute limits of this pwm channel
    pwm->cmp = CLAMP(cmp, pwm->rng_min, pwm->rng_max);
    pwm_set_compare(pwm->handle, pwm->cmp + pwm->trim);
}

// -----------------------------------------------------------------------------
static int fc_control(const ctl_sigs_t *sigs, int chnl_flags)
{
    if (!globals.enabled) {
        syslog(LOG_ERR, "fc_control: flight control is not enabled!");
        return 0;
    }

    if (chnl_flags & VCM_AXIS_ALT)
        assign_value(&globals.channels[PWM_ALT], sigs->alt);
    if (chnl_flags & VCM_AXIS_PITCH)
        assign_value(&globals.channels[PWM_PITCH], sigs->pitch);
    if (chnl_flags & VCM_AXIS_ROLL)
        assign_value(&globals.channels[PWM_ROLL], sigs->roll);
    if (chnl_flags & VCM_AXIS_YAW)
        assign_value(&globals.channels[PWM_YAW], sigs->yaw);
    return 1;
}

// -----------------------------------------------------------------------------
// Thread for autonomous altitude/throttle control (hovering).
static void *auto_alt_thread(void *arg)
{
    int vcm_axes, vcm_type, timeout = 0;
    float p_rad, r_rad, angles[3], altitude, pid_result = 0.0f;
    ctl_sigs_t signal;

    // flight control's signal structure
    memset(&signal, 0, sizeof(ctl_sigs_t));
    memset(&angles, 0, sizeof(float) * 3);
    pid_reset_error(&globals.pid_alt);
    
    // wait for the dr takeoff process to complete and signal pid altitude
    syslog(LOG_INFO, "auto_alt_thread: waiting...");
    pthread_mutex_lock(&globals.takeoff_lock);
    pthread_cond_wait(&globals.takeoff_cond, &globals.takeoff_lock);
    pthread_mutex_unlock(&globals.takeoff_lock);
    syslog(LOG_INFO, "auto_alt_thread: starting...");

    while (FCS_STATE_HOVERING == globals.state ||
            FCS_STATE_LANDING == globals.state) {
        // capture altitude and flight control's mode and vcm axes
        altitude = gpio_event_read(globals.usrf, ACCESS_SYNC) / (real_t)147;
        fc_get_vcm(&vcm_axes, &vcm_type);

        if (!imu_read_angles(globals.imu, angles, ACCESS_ASYNC)) {
            // shouldn't happen, but if it does just use the altitude by itself
            syslog(LOG_ERR, "auto_alt_thread: failed to read imu angles");
            angles[IMU_PITCH] = 0.0f;
            angles[IMU_ROLL]  = 0.0f;
        }
        p_rad = DEG_TO_RAD(angles[IMU_PITCH]);
        r_rad = DEG_TO_RAD(angles[IMU_ROLL]);

        // only control altitude with PID if axis is enabled (mixed mode)
        pthread_mutex_lock(&globals.pid_lock);
        if (!(vcm_axes & VCM_AXIS_ALT)) {
            // compensate for the pitch and roll angles (method 1)
            // altitude *= (cos(p_rad) * cos(r_rad));

            pid_result = pid_update(&globals.pid_alt, altitude);
            signal.alt = .58f + pid_result;

            // compensate for the pitch and roll angles (method 2)
            // float tpr = tan(p_rad);
            // float trr = tan(r_rad);
            // signal.alt /= (1.0f + atan(sqrt(tpr * tpr + trr * trr)));

            fc_control(&signal, VCM_AXIS_ALT);
        }
        pthread_mutex_unlock(&globals.pid_lock);

        // if landing requested, try not to release while still accelerating
        if (FCS_STATE_LANDING == globals.state) {
            globals.carry_over = signal.alt;
            if (++timeout > 100 || pid_result <= 0.0f) {
                break;
            }
        }
    }

    // signal the landing dr thread to take over
    pthread_mutex_lock(&globals.landing_lock);
    pthread_cond_broadcast(&globals.landing_cond);
    pthread_mutex_unlock(&globals.landing_lock);

    syslog(LOG_INFO, "auto_alt_thread: exiting");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
// Thread for autonomous orientation/servo control. Control rate is locked to
// the sampling razor of the 9DOF Razor IMU.
void *auto_imu_thread(void *arg)
{
    int axes, vcm_type;
    float angles[3] = { 0 };
    ctl_sigs_t signal;
    track_coords_t tc;

    // flight control's signal structure
    memset(&signal, 0, sizeof(ctl_sigs_t));

    // wait for the takeoff process to complete
    syslog(LOG_INFO, "auto_imu_thread waiting...");
    pthread_mutex_lock(&globals.takeoff_lock);
    pthread_cond_wait(&globals.takeoff_cond, &globals.takeoff_lock);
    pthread_mutex_unlock(&globals.takeoff_lock);
    syslog(LOG_INFO, "auto_imu_thread starting...");

    // reset controllers' error collections
    pid_reset_error(&globals.pid_pitch);
    pid_reset_error(&globals.pid_roll);
    pid_reset_error(&globals.pid_yaw);
    
    while (FCS_STATE_HOVERING == globals.state) {
        // blocks on IMU data to capture current angles
        if (!imu_read_angles(globals.imu, angles, ACCESS_SYNC)) {
            // error occurred while trying to read IMU data
            // is this an appropriate way of handling an error?
            continue;
        }
        
        // capture axes and type
        fc_get_vcm(&axes, &vcm_type);

        pthread_mutex_lock(&globals.pid_lock);
        if (!(axes & VCM_AXIS_PITCH)) {
            // compute PID result for pitch
            signal.pitch = -pid_update(&globals.pid_pitch, angles[IMU_PITCH]);
            signal.pitch = CLAMP(signal.pitch, -1.0f, 1.0f);
            fprintf(stderr, "signal.pitch = %f\n", signal.pitch);
            fc_control(&signal, VCM_AXIS_PITCH);
        }
        
        if (!(axes & VCM_AXIS_ROLL)) {
            // compute PID result for roll
            signal.roll = pid_update(&globals.pid_roll, angles[IMU_ROLL]);
            signal.roll = CLAMP(signal.roll, -1.0f, 1.0f);
            fprintf(stderr, "signal.roll = %f\n", signal.roll);
            fc_control(&signal, VCM_AXIS_ROLL);
        }
        
        if (!(axes & VCM_AXIS_YAW) && tracking_read_state(&tc, ACCESS_ASYNC)) {
            signal.yaw = 0.0f;
            if (tc.detected) {
                signal.yaw = (tc.yc < 120) ? -0.4f : 0.2f;
                fprintf(stderr, "detected: move %f\n", signal.yaw);
            }
            fc_control(&signal, VCM_AXIS_YAW);
            // compute PID result for yaw -- disabled for now due to EMF
            //float pid_result = pid_update(&globals.pid_yaw, angles[IMU_YAW]);
            //signal.yaw = pid_result;
            //fc_control(&signal, VCM_AXIS_YAW);
        }
        pthread_mutex_unlock(&globals.pid_lock);
    }

    syslog(LOG_INFO, "auto_imu_thread exiting");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
static void *dr_takeoff_thread(void *arg)
{
    ctl_sigs_t control;
    int vcm_axes, vcm_type;

    syslog(LOG_INFO, "dr_takeoff_thread: starting...");
    memset(&control, 0, sizeof(control));

    control.alt = 0.0f;
    while ((FCS_STATE_TAKEOFF == globals.state) && (control.alt < .65)) {
        fc_get_vcm(&vcm_axes, &vcm_type);

        // this yields a ramp up time a little under 5 seconds: (.65/.0014)*10ms
        if (!(vcm_axes & VCM_AXIS_ALT)) {
            control.alt += 0.0014f;
            fc_control(&control, VCM_AXIS_ALT);
            usleep(10000); // 10 ms (100 Hz)
        }
    }
    syslog(LOG_INFO, "dr_takeoff_thread: done ramping -- switching to pid");

    // is the state still takeoff? -- if so, switch to hovering now
    if (FCS_STATE_TAKEOFF == globals.state)
        fc_set_state(FCS_STATE_HOVERING);

    // signal to the blocked altitude and orientation threads to start
    pthread_mutex_lock(&globals.takeoff_lock);
    pthread_cond_broadcast(&globals.takeoff_cond);
    pthread_mutex_unlock(&globals.takeoff_lock);

    syslog(LOG_INFO, "dr_takeoff_thread: exiting");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
static void *dr_landing_thread(void *arg)
{
    ctl_sigs_t control;

    // wait until the altitude hover thread signals us to continue
    syslog(LOG_INFO, "dr_landing_thread: waiting...");
    pthread_mutex_lock(&globals.landing_lock);
    pthread_cond_wait(&globals.landing_cond, &globals.landing_lock);
    pthread_mutex_unlock(&globals.landing_lock);
    syslog(LOG_INFO, "dr_landing_thread: starting...");

    // start at the last throttle value asserted by the altitude hover thread
    control.alt = globals.carry_over;

    // back off the throttle in a decaying manner, until treshold reached
    while (FCS_STATE_LANDING == globals.state) {
        // capture altitude
        float altitude = gpio_event_read(globals.usrf, ACCESS_SYNC) / (real_t)147;
        if (altitude < 10.0f)
            break;

        control.alt -= 0.0004f;
        fc_control(&control, VCM_AXIS_ALT);
    }
    syslog(LOG_INFO, "dr_landing_thread: done dropping throttle");

    // by this point the helicopter will hopefully be on the ground with 
    // minimal throttle so we can shut off motors completely
    usleep(700);
    control.alt = 0.0f;
    fc_control(&control, VCM_AXIS_ALT);

    fc_set_state(FCS_STATE_GROUNDED);
    globals.capture_enabled = 1;
    syslog(LOG_INFO, "dr_landing_thread: exiting");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
static void *dr_replay_thread(void *arg)
{
    record_bucket_t *bucket;
    input_record_t *record;
    int i, vcm_axes, vcm_type;

    syslog(LOG_INFO, "dr_replay_thread: starting...");

    bucket = globals.record_head;
    while (NULL != bucket) {
        for (i = 0; i < bucket->count; i++) {
            fprintf(stderr, "inject\n");
            // control all enabled autonomous axes, but never throttle
            fc_get_vcm(&vcm_axes, &vcm_type);
            vcm_axes = ~(vcm_axes | VCM_AXIS_ALT);

            // inject the input then sleep for the specified period
            record = &bucket->records[i];
            clock_nanosleep(CLOCK_REALTIME, 0, &record->delta, 0);
            fc_control(&record->signals, vcm_axes);

            if (FCS_STATE_GROUNDED == globals.state) {
                bucket = NULL;
                break;
            }
        }

        bucket = bucket->next;
    }

    syslog(LOG_INFO, "dr_replay_thread: exiting");
    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
static int init_channel(int index, float lo, float hi, float idle)
{
    pwm_channel_t *pwm = &globals.channels[index];
    if (0 > (pwm->handle = pwm_open_device(PWM_DEV_FIRST + index))) {
        syslog(LOG_ERR, "error opening pwm %d device", index);
        return 0; 
    }

    // reset the trim to the dead center of the duty cycle range
    pwm->trim = 0;
    pwm->duty_lo = lo;
    pwm->duty_hi = hi;

    // keep throttle signal at the current value it is
    pwm_set_freq_x100(pwm->handle, 4582);
    pwm_get_range(pwm->handle, &pwm->rng_min, &pwm->rng_max);
    assign_duty(pwm, idle);
    return 1;
}

// -----------------------------------------------------------------------------
static void fc_reset_internals(void)
{
    pwm_channel_t *ch = &globals.channels[0];
    globals.curr_alt = 0.0f;
    globals.capture_enabled = 0;
    fc_set_state(FCS_STATE_GROUNDED);

    // wake up any threads that may be blocked so that they can exit cleanly
    pthread_mutex_lock(&globals.takeoff_lock);
    pthread_cond_broadcast(&globals.takeoff_cond);
    pthread_mutex_unlock(&globals.takeoff_lock);

    pthread_mutex_lock(&globals.landing_lock);
    pthread_cond_broadcast(&globals.landing_cond);
    pthread_mutex_unlock(&globals.landing_lock);

    // reset the pwm outputs to their idle values
    assign_duty(&ch[PWM_ALT], ALT_DUTY_LO);
    assign_duty(&ch[PWM_YAW], YAW_DUTY_IDLE);
    assign_duty(&ch[PWM_PITCH], PITCH_DUTY_IDLE);
    assign_duty(&ch[PWM_ROLL], ROLL_DUTY_IDLE);
}

// -----------------------------------------------------------------------------
int fc_init(gpio_event_t *pwm_usrf, imu_data_t *ypr_imu)
{
    // at initialization, start in autonomous control with all axes enabled
    memset(&globals, 0, sizeof(fc_globals_t));

    pid_init(&globals.pid_yaw,   0.0f, -12.0f, 12.0f);
    pid_init(&globals.pid_pitch, 0.0f, -12.0f, 12.0f);
    pid_init(&globals.pid_roll,  0.0f, -12.0f, 12.0f);
    pid_init(&globals.pid_alt,   0.0f, -12.0f, 12.0f);

    // save gpio event handles so we can fetch altitude and orgientation
    globals.usrf = pwm_usrf;
    globals.imu  = ypr_imu;

    // initialize all of our thread synchronization primitives
    pthread_mutex_init(&globals.state_lock, NULL);
    pthread_mutex_init(&globals.takeoff_lock, NULL);
    pthread_mutex_init(&globals.landing_lock, NULL);
    pthread_mutex_init(&globals.vcm_lock, NULL);
    pthread_mutex_init(&globals.pid_lock, NULL);

    pthread_cond_init(&globals.state_cond, NULL);
    pthread_cond_init(&globals.takeoff_cond, NULL);
    pthread_cond_init(&globals.landing_cond, NULL);
    
    // initialize the pwm channels for throttle, yaw, pitch, and roll
    if (!init_channel(PWM_ALT, ALT_DUTY_LO, ALT_DUTY_HI, ALT_DUTY_LO))
        return 0;
    syslog(LOG_INFO, "flight control: altitude channel opened");

    if (!init_channel(PWM_PITCH, PITCH_DUTY_LO, PITCH_DUTY_HI, PITCH_DUTY_IDLE))
        return 0; 
    syslog(LOG_INFO, "flight control: pitch channel opened");

    if (!init_channel(PWM_ROLL, ROLL_DUTY_LO, ROLL_DUTY_HI, ROLL_DUTY_IDLE))
        return 0;
    syslog(LOG_INFO, "flight control: roll channel opened");

    if (!init_channel(PWM_YAW, YAW_DUTY_LO, YAW_DUTY_HI, YAW_DUTY_IDLE))
        return 0;
    syslog(LOG_INFO, "flight control: yaw channel opened");

    globals.enabled = 1;
    fc_set_state(FCS_STATE_GROUNDED);
    fc_set_vcm(VCM_AXIS_ALL, VCM_TYPE_AUTO);
    syslog(LOG_INFO, "opened pwm device nodes");
    return 1;
}

// -----------------------------------------------------------------------------
void fc_shutdown(void)
{
    int i;

    // set flag for all controllers to see that flight control is shutting down
    globals.enabled = 0;

    // TODO: Wait for controller threads to exit

    for (i = 0; i < 4; ++i)
        pwm_close_device(globals.channels[i].handle);

    globals.usrf = NULL;

    pthread_mutex_destroy(&globals.takeoff_lock);
    pthread_mutex_destroy(&globals.landing_lock);
    pthread_mutex_destroy(&globals.vcm_lock);
    pthread_mutex_destroy(&globals.pid_lock);

    pthread_cond_destroy(&globals.state_cond);
    pthread_cond_destroy(&globals.takeoff_cond);
    pthread_cond_destroy(&globals.landing_cond);

    if (globals.capture_path) {
        // save and destroy the record buckets
        record_write_buckets();
        record_delete_buckets();
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
            syslog(LOG_ERR, "invalid line detected in replay file");
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

    syslog(LOG_INFO, "successfully loaded %d input records into %d buckets",
           entries_read, buckets_filled);
    globals.replay_path = path;
    return 1;
}

// -----------------------------------------------------------------------------
int fc_request_takeoff(void)
{
    // don't do anything if this subsystem is disabled
    if (!globals.enabled) {
        syslog(LOG_ERR, "fc_request_takeoff: flight control not enabled");
        return 0;
    }

    // only permit takeoff from the grounded state
    if (FCS_STATE_GROUNDED != globals.state) {
        syslog(LOG_ERR, "fc_request_takeoff: takeoff denied, not grounded");
        return 0;
    }

    fc_set_state(FCS_STATE_TAKEOFF);
    pthread_create(&globals.takeoff_thrd,  NULL, dr_takeoff_thread, 0);
    pthread_create(&globals.landing_thrd,  NULL, dr_landing_thread, 0);
    pthread_create(&globals.auto_alt_thrd, NULL, auto_alt_thread, 0);

    if (globals.replay_path) {
        syslog(LOG_INFO, "fc_request_takeoff: beginning replay execution");
        pthread_create(&globals.replay_thrd, NULL, dr_replay_thread, 0);
    }
    else {
        syslog(LOG_INFO, "fc_request_takeoff: beginning takeoff process");
        pthread_create(&globals.auto_imu_thrd, NULL, auto_imu_thread, 0);
        
        if (globals.capture_path) {
            syslog(LOG_INFO, "fc_request_takeoff: record capture enabled");
            record_delete_buckets();
            clock_gettime(CLOCK_REALTIME, &globals.last_time);
            globals.record_head = globals.record_tail = record_create_bucket();
            globals.capture_enabled = 1;
        }
    }

    return 1;
}

// -----------------------------------------------------------------------------
int fc_request_landing(void)
{
    // don't do anything if this subsystem is disabled
    if (!globals.enabled) {
        syslog(LOG_ERR, "fc_request_landing: flight control not enabled");
        return 0;
    }

    // only permit landing from the hovering state
    if (FCS_STATE_HOVERING != globals.state) {
        syslog(LOG_ERR, "fc_request_landing: landing denied, not hovering");
        return 0;
    }

    syslog(LOG_INFO, "fc_request_landing: beginning landing process");
    fc_set_state(FCS_STATE_LANDING);
    return 1;
}

// -----------------------------------------------------------------------------
int fc_set_vcm(int axes, int type)
{
    pthread_mutex_lock(&globals.vcm_lock);

    if (VCM_TYPE_KILL == globals.vcm_type) {
        // if we're killed, don't allow any more state transitions
        pthread_mutex_unlock(&globals.vcm_lock);
        syslog(LOG_ERR, "not alive. ignoring fc_set_vcm");
        return 0;
    }

    switch (type) {
    case VCM_TYPE_RADIO:
    case VCM_TYPE_LOCKOUT:
    case VCM_TYPE_KILL:
        // for any non-autonomous mode, reset axes
        globals.vcm_axes = VCM_AXIS_ALL;
        break;
    case VCM_TYPE_AUTO:
        // all manual axes disabled in full autonomous mode
        globals.vcm_axes = 0;
        break;
    case VCM_TYPE_MIXED:
        // set user specified axes in mixed mode
        globals.vcm_axes = axes;
        break;
    default:
        syslog(LOG_ERR, "unknown vcm type (%d) in fc_set_vcm", type);
        break;
    }

    if (type != globals.vcm_type) {
        // reset the channels to their idle values every time we switch state
        globals.vcm_type = type;
        fc_reset_internals();
    }

    // save the last timer tick every time we switch state
    clock_gettime(CLOCK_REALTIME, &globals.last_time);
    pthread_mutex_unlock(&globals.vcm_lock);
    return 1;
}

// -----------------------------------------------------------------------------
void fc_get_vcm(int *axes, int *type)
{
    pthread_mutex_lock(&globals.vcm_lock);
    *axes = globals.vcm_axes;
    *type = globals.vcm_type;
    pthread_mutex_unlock(&globals.vcm_lock);
}

// -----------------------------------------------------------------------------
int fc_get_state(access_mode_t mode)
{
    int state;
    pthread_mutex_lock(&globals.state_lock);

    switch (mode) {
    case ACCESS_ASYNC:
        // access in an asynchronous (non-blocking) fashion
        break;
    case ACCESS_SYNC:
        // access in a synchronous (blocking) fashion
        pthread_cond_wait(&globals.state_cond, &globals.state_lock);
        break;
    default:
        syslog(LOG_ERR, "fc_get_state: invalid access mode");
        break;
    }

    state = globals.state;
    pthread_mutex_unlock(&globals.state_lock);
    return state;
}

// -----------------------------------------------------------------------------
void fc_set_ctl(ctl_sigs_t *sigs)
{
    int axes;

    pthread_mutex_lock(&globals.vcm_lock);
    axes = globals.vcm_axes;

    // scale down the throttle signal, as the default [-1, 1] is too sensitive
    globals.curr_alt = CLAMP(globals.curr_alt + sigs->alt * 0.01, 0, 1);
    sigs->alt = globals.curr_alt;
    if (sigs->yaw < 0.0f)
        sigs->yaw *= 0.50;
    else
        sigs->yaw *= 0.25;

    fc_control(sigs, axes);
    pthread_mutex_unlock(&globals.vcm_lock);

    if (globals.capture_enabled) {
        // if --capture is enabled, store this control signal
        record_insert_sigs(sigs);
    }
}

// -----------------------------------------------------------------------------
void fc_set_trims(int axes, int value)
{
    if (axes & VCM_AXIS_ALT) {
        pwm_channel_t *alt = &globals.channels[PWM_ALT];
        alt->trim = value;
        pwm_set_compare(alt->handle, alt->cmp + alt->trim);
        syslog(LOG_INFO, "setting ALT trim to %d", value);
    }
    if (axes & VCM_AXIS_YAW) {
        pwm_channel_t *yaw = &globals.channels[PWM_YAW];
        yaw->trim = value;
        pwm_set_compare(yaw->handle, yaw->cmp + yaw->trim);
        syslog(LOG_INFO, "setting YAW trim to %d", value);
    }
    if (axes & VCM_AXIS_PITCH) {
        pwm_channel_t *pitch = &globals.channels[PWM_PITCH];
        pitch->trim = value;
        pwm_set_compare(pitch->handle, pitch->cmp + pitch->trim);
        syslog(LOG_INFO, "setting PITCH trim to %d", value);
    }
    if (axes & VCM_AXIS_ROLL) {
        pwm_channel_t *roll = &globals.channels[PWM_ROLL];
        roll->trim = value;
        pwm_set_compare(roll->handle, roll->cmp + roll->trim);
        syslog(LOG_INFO, "setting ROLL trim to %d", value);
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

// -----------------------------------------------------------------------------
int fc_set_pid_param(int axis, int param, float value)
{
    pthread_mutex_lock(&globals.pid_lock);
    pid_ctrl_t *pid = NULL;

    switch (axis) {
    case VCM_AXIS_YAW:
        fprintf(stderr, "setting yaw pid, ");
        pid = &globals.pid_yaw; break;
    case VCM_AXIS_PITCH:
        fprintf(stderr, "setting pitch pid, ");
        pid = &globals.pid_pitch; break;
    case VCM_AXIS_ROLL:
        fprintf(stderr, "setting roll pid, ");
        pid = &globals.pid_roll; break;
    case VCM_AXIS_ALT:
        fprintf(stderr, "setting alt pid, ");
        pid = &globals.pid_alt; break;
    default:
        pthread_mutex_unlock(&globals.pid_lock);
        syslog(LOG_ERR, "invalid axis specified for fc_set_pid_param");
        return 0;
    }

    switch (param) {
    case PID_PARAM_KP:
        fprintf(stderr, "kp = %f\n", value);
        pid->kp = value; break;
    case PID_PARAM_KI:
        fprintf(stderr, "ki = %f\n", value);
        pid->ki = value; break;
    case PID_PARAM_KD:
        fprintf(stderr, "kd = %f\n", value);
        pid->kd = value; break;
    case PID_PARAM_SP:
        fprintf(stderr, "setpoint = %f\n", value);
        pid->setpoint = value; break;
    default:
        pthread_mutex_unlock(&globals.pid_lock);
        syslog(LOG_ERR, "invalid parameter specified for fc_set_pid_param");
        return 0;
    }

    pid_reset_error(pid);
    pthread_mutex_unlock(&globals.pid_lock);
    return 1;
}

// -----------------------------------------------------------------------------
void fc_get_pid_params(int axis, float *params)
{
    pthread_mutex_lock(&globals.pid_lock);
    switch (axis) { 
    case VCM_AXIS_YAW:
        params[PID_PARAM_KP] = globals.pid_yaw.kp;
        params[PID_PARAM_KI] = globals.pid_yaw.ki;
        params[PID_PARAM_KD] = globals.pid_yaw.kd;
        params[PID_PARAM_SP] = globals.pid_yaw.setpoint;
        break;
    case VCM_AXIS_PITCH:
        params[PID_PARAM_KP] = globals.pid_pitch.kp;
        params[PID_PARAM_KI] = globals.pid_pitch.ki;
        params[PID_PARAM_KD] = globals.pid_pitch.kd;
        params[PID_PARAM_SP] = globals.pid_pitch.setpoint;
        break;
    case VCM_AXIS_ROLL:
        params[PID_PARAM_KP] = globals.pid_roll.kp;
        params[PID_PARAM_KI] = globals.pid_roll.ki;
        params[PID_PARAM_KD] = globals.pid_roll.kd;
        params[PID_PARAM_SP] = globals.pid_roll.setpoint;
        break;
    case VCM_AXIS_ALT:
        params[PID_PARAM_KP] = globals.pid_alt.kp;
        params[PID_PARAM_KI] = globals.pid_alt.ki;
        params[PID_PARAM_KD] = globals.pid_alt.kd;
        params[PID_PARAM_SP] = globals.pid_alt.setpoint;
        break;
    default:
        memset(params, 0, sizeof(float) * 3);
        syslog(LOG_ERR, "unknown axis specified in fc_get_pid_params");
        break;
    }
    pthread_mutex_unlock(&globals.pid_lock);
}

