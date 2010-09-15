// pwm_interface.h
// Garrett Smith 2010

#ifndef PWM_INTERFACE__H_
#define PWM_INTERFACE__H_

#define PWM_IOC_MAGIC 'G'

#define PWM_IOC_ENABLE      _IO(PWM_IOC_MAGIC, 0)
#define PWM_IOC_DISABLE     _IO(PWM_IOC_MAGIC, 1)
#define PWM_IOCT_FREQ       _IOW(PWM_IOC_MAGIC, 2, int)
#define PWM_IOCQ_FREQ       _IOR(PWM_IOC_MAGIC, 3, int)
#define PWM_IOCT_DUTY       _IOW(PWM_IOC_MAGIC, 4, int)
#define PWM_IOCQ_DUTY       _IOR(PWM_IOC_MAGIC, 5, int)
#define PWM_IOCT_COMPARE    _IOW(PWM_IOC_MAGIC, 6, int)
#define PWM_IOCQ_MINRANGE   _IOR(PWM_IOC_MAGIC, 7, int)
#define PWM_IOCQ_MAXRANGE   _IOR(PWM_IOC_MAGIC, 8, int)

struct pwm_raw_user_parms {
    int frequency;
    int precision;
};

#endif // PWM_INTERFACE__H_

