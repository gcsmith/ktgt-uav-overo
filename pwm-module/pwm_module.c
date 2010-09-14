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
    int freq;                       // pwm frequency
    int duty;                       // pwm duty cycle
    int dev;                        // device number
    int maj;                        // major device number
    int min;                        // minor device number
    char buff[1024];                // general purpose buffer for file IO
    unsigned int gpt_base;
    unsigned int mux_base;
    uint16_t mux_restore;
};

// holds global data for this driver module
struct pwm_module {
    dev_t chrdev;                   // first character device number
    struct class *class;            // device class structure
    struct pwm_dev dev[PWM_COUNT];  // pwm device node data
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
static int pwm_ioctl(struct inode *inode, struct file *fp, unsigned int cmd, unsigned long arg)
{
    printk("executing pwm_ioctl()\n");
    return 0;
}

// -----------------------------------------------------------------------------
static ssize_t pwm_read(struct file *fp, char __user *buff, size_t count, loff_t *off)
{
    struct pwm_dev *dev = fp->private_data;
    size_t len, copy_len;

    if (*off > 0) {
        return 0;
    }

    printk("executing pwm_read()\n");
    len = snprintf(dev->buff, 1024, "PWM%d freq=??? duty=???\n", dev->min) + 1;

    copy_len = MIN(len, count);
    if (copy_to_user(buff, dev->buff, copy_len)) {
        // IO error
        return -EIO;
    }

    *off = copy_len;
    return copy_len;
}

// -----------------------------------------------------------------------------
static ssize_t pwm_write(struct file *fp, const char __user *buff, size_t count, loff_t *off)
{
    struct pwm_dev *dev = fp->private_data;
    unsigned int value = 0;
    size_t copy_len;
    void __iomem *base;
    printk("executing pwm_write()\n");

    copy_len = MIN(1023, count);
    if (copy_from_user(dev->buff, buff, copy_len)) {
        // IO error
        return -EIO;
    }
    dev->buff[copy_len] = '\0';

    value = simple_strtol(dev->buff, NULL, 10);
    value = MIN(value, 0x4000);

    base = ioremap(dev->gpt_base, GPTIMER_SIZE);
    if (!base) {
        printk(KERN_ALERT "failed to map GPTIMER address range\n");
        return -1;
    }

    iowrite32(0xfffC3400 + value, base + GPT_TMAR);
    iounmap(base);

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

    iowrite32(0x00000000, base + GPT_TCLR); // stop the timer
    iowrite32(0xfffc08F0, base + GPT_TLDR); // timer load 20ms period
    iowrite32(0xfffC3400, base + GPT_TMAR); // set timer to low duty cycle
    iowrite32(0xffffffff, base + GPT_TCRR); // set the timer counter
    iowrite32(0x00001843, base + GPT_TCLR); // start the timer
    iounmap(base);

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

    if (pwm_en_count >= PWM_COUNT) {
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
            pwm.dev[index].freq = pwm_freq[i];
        }
    }

    // set user specified duty cycles
    for (i = 0; i < pwm_duty_count; i++) {
        int index = indices[i];
        if (index >= 0) {
            pwm.dev[index].duty = pwm_duty[i];
        }
    }

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

