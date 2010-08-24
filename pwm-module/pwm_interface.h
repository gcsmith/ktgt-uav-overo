// pwm_interface.h
// Garrett Smith 2010

#ifndef PWM_INTERFACE__H_
#define PWM_INTERFACE__H_

#define PWM_IOC_MAGIC 'G'
#define PWM_IOCX_RAW_READ   _IO(PWM_IOC_MAGIC, 0)
#define PWM_IOCX_RAW_WRITE  _IO(PWM_IOC_MAGIC, 1)

struct pwm_raw_user_parms {
    int frequency;
    int precision;
};

#endif // PWM_INTERFACE__H_

