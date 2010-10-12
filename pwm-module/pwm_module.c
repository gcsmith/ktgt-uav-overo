// pwm_module.c
// Garrett Smith 2010

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "pwm_module.h"
#include "pwm_interface.h"

// user-specified module parameters
static int pwm_en[4] = { 8, 9, 10, 11 };
static int pwm_en_count = 0;
static int pwm_freq[4] = { 50, 50, 50, 50 };
static int pwm_freq_count = 0;
static int pwm_duty[4] = { 50, 50, 50, 50 };
static int pwm_duty_count = 0;

module_param_array(pwm_en, int, &pwm_en_count, 0000);
MODULE_PARM_DESC(pwm_en, "Specify PWM outputs to enable (8-11)");

module_param_array(pwm_freq, int, &pwm_freq_count, 0000);
MODULE_PARM_DESC(pwm_freq, "Specify frequency of each enabled PWM");

module_param_array(pwm_duty, int, &pwm_duty_count, 0000);
MODULE_PARM_DESC(pwm_duty, "Specify duty cycle of each enabled PWM");

// holds per-device settings for each PWM
struct pwm_dev {
    struct cdev cdev;               // character device info
    int en;                         // is this pwm device enabled?
    int active;                     // started/stopped state
    int dev;                        // device number
    int maj;                        // major device number
    int min;                        // minor device number
    char buff[1024];                // general purpose buffer for file IO
    unsigned int gpt_base;
    unsigned int mux_base;
    uint16_t mux_restore;
    uint32_t tldr;
    uint32_t tmar;
    uint32_t tmar_min;
    uint32_t tmar_max;
};

// holds global data for this driver module
struct pwm_module {
    dev_t chrdev;                   // first character device number
    struct class *class;            // device class structure
    struct pwm_dev dev[PWM_COUNT];  // pwm device node data
    uint32_t gpio171_mux_restore;
    uint32_t gpio172_mux_restore;
    uint32_t gpio173_mux_restore;
    uint32_t gpio174_mux_restore;
};

static struct pwm_module pwm;

// general purpose timer addresses for PWM8-PWM11
static const unsigned int gptimer_addrs[] = {
    GPTIMER8_ADDR,
    GPTIMER9_ADDR,
    GPTIMER10_ADDR,
    GPTIMER11_ADDR,
};

// pad multiplexing register offsets for pwm_evt modes
static const unsigned int padconf_offsets[] = {
    CONTROL_PADCONF_UART2_RX_OFFSET,    // mode 2: gpt8_pwm_evt
    CONTROL_PADCONF_UART2_CTS_OFFSET,   // mode 2: gpt9_pwm_evt
    CONTROL_PADCONF_UART2_RTS_OFFSET,   // mode 2: gpt10_pwm_evt
    CONTROL_PADCONF_UART2_TX_OFFSET,    // mode 2: gpt11_pwm_evt
};

// -----------------------------------------------------------------------------
void pwm_set_active(struct pwm_dev *dev, int active)
{
    // enable or disable the timer
    void __iomem *base = ioremap(dev->gpt_base, GPTIMER_SIZE);
    if (!base) {
        printk(KERN_ALERT "failed to map GPTIMER address range\n");
        return;
    }

    iowrite32(active ? 0x00001843 : 0, base + GPT_TCLR);
    iounmap(base);
}

// -----------------------------------------------------------------------------
void pwm_set_freq(struct pwm_dev *dev, uint32_t freq)
{
    void __iomem *base = ioremap(dev->gpt_base, GPTIMER_SIZE);

    if (!base) {
        printk(KERN_ALERT "failed to map GPTIMER address range\n");
        return;
    }

    freq = MIN(freq, 1000);
    dev->tldr = 0xFFFFFFFF - 13000000 / freq + 1;
    dev->tmar_min = dev->tldr;
    dev->tmar_max = (0xFFFFFFFF - dev->tldr) - 1;

    iowrite32(dev->tldr, base + GPT_TLDR); // timer load 20ms period
    iounmap(base);
}

// -----------------------------------------------------------------------------
inline int pwm_get_freq(struct pwm_dev *dev)
{
    return 13000000 / (0xFFFFFFFF - dev->tldr + 1);
}

// -----------------------------------------------------------------------------
void pwm_set_freq_x100(struct pwm_dev *dev, uint32_t freq)
{
    void __iomem *base = ioremap(dev->gpt_base, GPTIMER_SIZE);

    if (!base) {
        printk(KERN_ALERT "failed to map GPTIMER address range\n");
        return;
    }

    freq = MIN(freq, 100000);
    dev->tldr = 0xFFFFFFFF - 1300000000 / freq + 1;
    dev->tmar_min = dev->tldr;
    dev->tmar_max = (0xFFFFFFFF - dev->tldr) - 1;

    iowrite32(dev->tldr, base + GPT_TLDR); // timer load 20ms period
    iounmap(base);
}

// -----------------------------------------------------------------------------
inline int pwm_get_freq_x100(struct pwm_dev *dev)
{
    return 1300000000 / (0xFFFFFFFF - dev->tldr + 1);
}

// -----------------------------------------------------------------------------
void pwm_set_duty(struct pwm_dev *dev, uint32_t duty)
{
    void __iomem *base;

    duty = MIN(duty, 100);
    dev->tmar = dev->tmar_min + (duty * dev->tmar_max) / 100;

    base = ioremap(dev->gpt_base, GPTIMER_SIZE);
    if (!base) {
        printk(KERN_ALERT "failed to map GPTIMER address range\n");
        return;
    }

    iowrite32(dev->tmar, base + GPT_TMAR);
    iounmap(base);
}

// -----------------------------------------------------------------------------
inline int pwm_get_duty(struct pwm_dev *dev)
{
    return 100 * (dev->tmar - dev->tldr) / (0xFFFFFFFF - dev->tldr - 1);
}

// -----------------------------------------------------------------------------
void pwm_set_compare(struct pwm_dev *dev, uint32_t compare)
{
    void __iomem *base;

    // compare = MAX(MIN(compare, dev->tmar_max), dev->tmar_min);
    dev->tmar = compare;

    base = ioremap(dev->gpt_base, GPTIMER_SIZE);
    if (!base) {
        printk(KERN_ALERT "failed to map GPTIMER address range\n");
        return;
    }

    iowrite32(dev->tmar, base + GPT_TMAR);
    iounmap(base);
}

// -----------------------------------------------------------------------------
inline uint32_t pwm_get_minrange(struct pwm_dev *dev)
{
    return dev->tmar_min;
}

// -----------------------------------------------------------------------------
inline uint32_t pwm_get_maxrange(struct pwm_dev *dev)
{
    return dev->tmar_min + dev->tmar_max;
}

// -----------------------------------------------------------------------------
static int pwm_open(struct inode *inode, struct file *fp)
{
    printk("executing pwm_open()\n");
    fp->private_data = container_of(inode->i_cdev, struct pwm_dev, cdev);

    return 0;
}

// -----------------------------------------------------------------------------
static int pwm_release(struct inode *inode, struct file *fp)
{
    printk("executing pwm_release()\n");
    return 0;
}

// -----------------------------------------------------------------------------
static int pwm_ioctl(struct inode *inode, struct file *fp,
                     unsigned int cmd, unsigned long arg)
{
    struct pwm_dev *dev = fp->private_data;
    int retval = 0;

    switch (cmd)
    {
    case PWM_IOC_ENABLE:
        // set pwm active state to on
        pwm_set_active(dev, 1);
        break;
    case PWM_IOC_DISABLE:
        // set pwm active state to off
        pwm_set_active(dev, 0);
        break;
    case PWM_IOCT_FREQ:
        // specify the pwm frequency
        pwm_set_freq(dev, (uint32_t)arg);
        break;
    case PWM_IOCQ_FREQ:
        // return the pwm frequency
        retval = pwm_get_freq(dev);
        break;
    case PWM_IOCT_FREQ_X100:
        // specify the pwm frequency scaled by 100
        pwm_set_freq_x100(dev, (uint32_t)arg);
        break;
    case PWM_IOCQ_FREQ_X100:
        // return the pwm frequency scaled by 100
        retval = pwm_get_freq_x100(dev);
        break;
    case PWM_IOCT_DUTY:
        // specify the pwm duty cycle
        pwm_set_duty(dev, (uint32_t)arg);
        break;
    case PWM_IOCQ_DUTY:
        // return the pwm duty cycle
        retval = pwm_get_duty(dev);
        break;
    case PWM_IOCT_COMPARE:
        // set the compare value
        pwm_set_compare(dev, arg);
        break;
    case PWM_IOCQ_MINRANGE:
        // return the pwm compare minimum value
        retval = pwm_get_minrange(dev);
        break;
    case PWM_IOCQ_MAXRANGE:
        // return the pwm compare maximum value
        retval = pwm_get_maxrange(dev);
        break;
    }

    return retval;
}

// -----------------------------------------------------------------------------
static ssize_t pwm_read(struct file *fp, char __user *buff,
                        size_t count, loff_t *off)
{
    struct pwm_dev *dev = fp->private_data;
    size_t len, copy_len;
    int freq, duty;

    if (*off > 0) {
        return 0;
    }

    freq = 13000000 / (0xFFFFFFFF - dev->tldr + 1);
    duty = 100 * (dev->tmar - dev->tldr) / (0xFFFFFFFF - dev->tldr - 1);

    printk("executing pwm_read()\n");
    len = snprintf(dev->buff, 1024, "PWM%d freq=%d duty=%d\n",
                   dev->min, freq, duty) + 1;

    copy_len = MIN(len, count);
    if (copy_to_user(buff, dev->buff, copy_len)) {
        // IO error
        return -EIO;
    }

    *off = copy_len;
    return copy_len;
}

// -----------------------------------------------------------------------------
int tokenize_str(char *ptr, char **ptok, char **pnext)
{
    if (!ptr || !ptok || !pnext)
        return 0;

    *ptok = NULL;
    *pnext = NULL;

    // seek to first non-delimeter character
    for (;;) {
        if (*ptr == '\0')
            return 0;
        if ((*ptr != ' ') && (*ptr != '\t') && (*ptr != '\n'))
            break;
        ptr++;
    }

    *ptok = ptr;

    // seek to next delimeter or end of string
    for (;;) {
        if (*ptr == '\0')
            return 1;
        if ((*ptr == ' ') || (*ptr == '\t') || (*ptr == '\n'))
            break;
        ptr++;
    }

    // terminate the current token and point to the next
    *ptr = '\0';
    *pnext = ptr + 1;
    return 1;
}

// -----------------------------------------------------------------------------
static ssize_t pwm_write(struct file *fp, const char __user *buff,
                         size_t count, loff_t *off)
{
    struct pwm_dev *dev = fp->private_data;
    int freq = 0, duty = 0;
    size_t copy_len;
    char *ptok, *pstr;
    printk("executing pwm_write()\n");

    copy_len = MIN(1023, count);
    if (copy_from_user(dev->buff, buff, copy_len)) {
        // IO error
        return -EIO;
    }
    dev->buff[copy_len] = '\0';

    // fetch arguments in the format: name [parms] name [parms] ...
    pstr = dev->buff;
    while (pstr && tokenize_str(pstr, &ptok, &pstr)) {
        // check the argument name
        if (!strcmp(ptok, "start")) {
            pwm_set_active(dev, 1);
        }
        else if (!strcmp(ptok, "stop")) {
            pwm_set_active(dev, 0);
        }
        else if (!strcmp(ptok, "freq")) {
            // require a frequency parameter
            if (!tokenize_str(pstr, &ptok, &pstr)) {
                break;
            }
            freq = simple_strtol(ptok, NULL, 10);
            pwm_set_freq(dev, freq);
        }
        else if (!strcmp(ptok, "freq_x100")) {
            // require a frequency parameter
            if (!tokenize_str(pstr, &ptok, &pstr)) {
                break;
            }
            freq = simple_strtol(ptok, NULL, 10);
            pwm_set_freq_x100(dev, freq);
        }
        else if (!strcmp(ptok, "duty")) {
            // require a duty cycle parameter
            if (!tokenize_str(pstr, &ptok, &pstr)) {
                break;
            }
            duty = simple_strtol(ptok, NULL, 10);
            pwm_set_duty(dev, duty);
        }
    }

    *off += copy_len;
    return copy_len;
}

// set relevant file io operation callbacks
static struct file_operations pwm_fops = {
    .owner   = THIS_MODULE,
    .open    = pwm_open,
    .release = pwm_release,
    .ioctl   = pwm_ioctl,
    .read    = pwm_read,
    .write   = pwm_write,
};

// -----------------------------------------------------------------------------
static int pwm_dev_init(struct pwm_dev *dev, int index)
{
    int rc;
    void __iomem *base;
    dev->maj = MAJOR(pwm.chrdev);
    dev->min = MINOR(pwm.chrdev) + index;
    dev->dev = MKDEV(dev->maj, dev->min);
    dev->gpt_base = gptimer_addrs[index];
    dev->mux_base = padconf_offsets[index];
    printk("initializing pwm device %d (%d.%d)\n", dev->dev, dev->maj, dev->min);

    // create the character device for this PWM index
    cdev_init(&dev->cdev, &pwm_fops);
    dev->cdev.owner = THIS_MODULE;
    dev->cdev.ops = &pwm_fops;

    if ((rc = cdev_add(&dev->cdev, dev->dev, 1)) < 0) {
        printk(KERN_ALERT "cdev_add failed (rc = %d)\n", rc);
        return -1;
    }

    if (!device_create(pwm.class, NULL, dev->dev, NULL, "pwm%d", dev->min)) {
        printk(KERN_ALERT "device_create failed\n");
        return -1;
    }

    // initialize the padconf and gp timer registers
    base = ioremap(CONTROL_PADCONF_MUX_PBASE, CONTROL_PADCONF_MUX_SIZE);
    if (!base) {
        printk(KERN_ALERT "failed to map PADCONF address range\n");
        return -1;
    }

    // set mux to mode 2 (pwm_evt)
    dev->mux_restore = ioread16(base + dev->mux_base);
    iowrite16(0x002, base + dev->mux_base);
    iounmap(base);

    // enable the timer
    base = ioremap(dev->gpt_base, GPTIMER_SIZE);
    if (!base) {
        printk(KERN_ALERT "failed to map GPTIMER address range\n");
        return -1;
    }

    dev->tmar = 0xfffc3400;
    dev->tldr = 0xfffc0860;

    iowrite32(0x00000000, base + GPT_TCLR); // stop the timer
    iowrite32(dev->tldr,  base + GPT_TLDR); // timer load 20ms period
    iowrite32(dev->tmar,  base + GPT_TMAR); // set timer to low duty cycle
    iowrite32(0xffffffff, base + GPT_TCRR); // set the timer counter
    iowrite32(0x00001843, base + GPT_TCLR); // start the timer
    iounmap(base);

    pwm_set_freq(dev, 50);

    // select 13 mhz clock for pwm10 and pwm11
    if (index >= 2)
    {
        int value;
        base = ioremap(CM_CORE_PBASE, CM_CORE_SIZE);
        if (!base) {
            printk(KERN_ALERT "failed to map CM CORE register space\n");
            return -1;
        }

        value = ioread32(base + CM_CLKSEL_CORE);
        value |= ((index == 2) ? CLKSEL_GPT10 : CLKSEL_GPT11);
        iowrite32(value, base + CM_CLKSEL_CORE);
        iounmap(base);
    }

    return 1;
}

// -----------------------------------------------------------------------------
static void pwm_dev_cleanup(struct pwm_dev *dev, int index)
{
    void __iomem *base;
    printk("cleaning up pwm device index %d\n", index);

    // reset the multiplexer register to its initial value
    base = ioremap(CONTROL_PADCONF_MUX_PBASE, CONTROL_PADCONF_MUX_SIZE);
    if (base) {
        iowrite16(dev->mux_restore, base + dev->mux_base);
        iounmap(base);
    }

    // destroy the device node and character device data
    device_destroy(pwm.class, dev->dev);
    cdev_del(&dev->cdev);
}

// -----------------------------------------------------------------------------
static void pwm_handle_args(void)
{
    int indices[PWM_COUNT] = { -1 };
    int i;

    // initialize structure and process user specified module arguments
    memset(&pwm, 0, sizeof(pwm));

    if (pwm_en_count > PWM_COUNT) {
        printk(KERN_ALERT "specified too many pwm indices. ignoring args\n");
        return;
    }
    if (pwm_freq_count > pwm_en_count) {
        printk(KERN_ALERT "inconsistent frequency count. ignoring args\n");
        return;
    }
    if (pwm_duty_count > pwm_en_count) {
        printk(KERN_ALERT "inconsistent duty cycle count. ignoring args\n");
        return;
    }

    for (i = 0; i < pwm_en_count; i++) {
        int index = pwm_en[i] - PWM_FIRST;
        if (index < 0 || index >= PWM_COUNT) {
            printk(KERN_ALERT "specified invalid pwm index %d\n", pwm_en[i]);
            continue;
        }
        if (0 != pwm.dev[index].en) {
            printk(KERN_ALERT "specified duplicate pwm index %d\n", pwm_en[i]);
            continue;
        }
        pwm.dev[index].en = 1;
        indices[i] = index;
    }

    // set user specified frequencies
    for (i = 0; i < pwm_freq_count; i++) {
        int index = indices[i];
        if (index >= 0) {
            // pwm.dev[index].freq = pwm_freq[i];
        }
    }

    // set user specified duty cycles
    for (i = 0; i < pwm_duty_count; i++) {
        int index = indices[i];
        if (index >= 0) {
            // pwm.dev[index].duty = pwm_duty[i];
        }
    }

}

// -----------------------------------------------------------------------------
int pwm_disable_spi(void)
{
    // this doesn't really belong in a pwm driver, but it's convenient for now
    void __iomem *base = ioremap(0x480021C8, 8);
    if (!base) {
        return 0;
    }

    pwm.gpio171_mux_restore = ioread16(base + 0);
    pwm.gpio172_mux_restore = ioread16(base + 2);
    pwm.gpio173_mux_restore = ioread16(base + 4);
    pwm.gpio174_mux_restore = ioread16(base + 6);

    iowrite16(0x104, base + 0);
    iowrite16(0x104, base + 2);
    iowrite16(0x104, base + 4);
    iowrite16(0x104, base + 6);

    iounmap(base);
    return 1;
}

// -----------------------------------------------------------------------------
int pwm_restore_spi(void)
{
    // this doesn't really belong in a pwm driver, but it's convenient for now
    void __iomem *base = ioremap(0x480021C8, 8);
    if (!base) {
        return 0;
    }

    iowrite16(pwm.gpio171_mux_restore, base + 0);
    iowrite16(pwm.gpio172_mux_restore, base + 2);
    iowrite16(pwm.gpio173_mux_restore, base + 4);
    iowrite16(pwm.gpio174_mux_restore, base + 6);

    iounmap(base);
    return 1;
}

// -----------------------------------------------------------------------------
static int __init pwm_init(void)
{
    int rc, i;
    printk("executing pwm_init...\n");

    pwm_handle_args();

    // allocate the character device range (4 minor ids from pwm8 to pwm11 )
    if ((rc = alloc_chrdev_region(&pwm.chrdev, PWM_FIRST, PWM_COUNT, "pwm")) < 0) {
        printk(KERN_ALERT "alloc_chrdev_region failed (rc = %d)\n", rc);
        return -1;
    }

    pwm.class = class_create(THIS_MODULE, "pwm");
    if (NULL == pwm.class) {
        printk(KERN_ALERT "create_class failed\n");
        return -1;
    }

    // allocate each individual pwm device
    for (i = 0; i < PWM_COUNT; ++i) {
        if ((0 != pwm.dev[i].en) && (rc = pwm_dev_init(&pwm.dev[i], i)) < 0) {
            printk(KERN_ALERT "pwm_dev_init(%d) failed (rc = %d)\n", i, rc);
            return -1;
        }
    }

    if (!pwm_disable_spi()) {
        printk(KERN_ALERT "unable to disable spi gpio pins");
        return -1;
    }

    return 0;
}

// -----------------------------------------------------------------------------
static void __exit pwm_exit(void)
{
    int i;
    printk("executing pwm_exit...\n");

    for (i = 0; i < PWM_COUNT; ++i) {
        if (0 != pwm.dev[i].en) {
            printk("destroying pwm device %d...\n", i + PWM_FIRST);
            pwm_dev_cleanup(&pwm.dev[i], i);
        }
    }

    printk("destroying pwm class and device ids...\n");
    class_destroy(pwm.class);
    unregister_chrdev_region(pwm.chrdev, PWM_COUNT);

    printk("restoring spi gpio pins...\n");
    pwm_restore_spi();
}

module_init(pwm_init);
module_exit(pwm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Garrett Smith <smith.garrett.c@gmail.com>");
MODULE_DESCRIPTION("PWM driver for OMAP3 devices");

MODULE_SUPPORTED_DEVICE("pwm8");
MODULE_SUPPORTED_DEVICE("pwm9");
MODULE_SUPPORTED_DEVICE("pwm10");
MODULE_SUPPORTED_DEVICE("pwm11");

