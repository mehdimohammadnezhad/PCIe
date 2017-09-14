/*
 * SIS8300 ioctl syscall handling.
 */

#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/uaccess.h>

#include "sis8300.h"
#include "sis8300_defs.h"
#include "sis8300_ioctl.h"
#include "sis8300_reg.h"
#include "sis8300_dma.h"


/**
 * Wait for daq irq user-defined irq (defined in custom firmware).
 * Which irq to wait for is determined by the irq type argument.
 *
 * If the timeout is supplied (with the ioctl call) then wait for the interrupt
 * with a timeout otherwise wait for the interrupt indefinitely.
 */
int sis8300_wait_irq(sis8300_dev *sisdevice, sis8300_irq *irq) {
    long                status;
    unsigned long       jiffies;
    int                 *wait_flag;
    wait_queue_head_t   *wait_queue;
    
    switch (irq->type) {
        case SIS8300_DAQ_IRQ:
            wait_flag = &sisdevice->daq_irq_flag;
            wait_queue = &sisdevice->daq_irq_wait;
            break;
        case SIS8300_USR_IRQ:
            wait_flag = &sisdevice->usr_irq_flag;
            wait_queue = &sisdevice->usr_irq_wait;
            break;
        default:
            printk(KERN_ERR "%s: irq type unknown: 0x%x\n", sisdevice->name, irq->type);
            return -EINVAL;
    }
    
    *wait_flag = SIS8300_IRQ_NONE;

    /* There is a race condition here if we make the availability check before
     * the device the removed and start waiting after the removal. */
    if (!sisdevice->available) {
        printk(KERN_ERR "%s: device not physically present\n", sisdevice->name);
        return -ENODEV;
    }

    if(irq->timeout) {
        /* Wait for interrupt, timeout is specified in milliseconds. */
        jiffies = irq->timeout * HZ / 1000;
        status = wait_event_interruptible_timeout(*wait_queue, *wait_flag != SIS8300_IRQ_NONE, jiffies);
    } else {
        status = wait_event_interruptible(*wait_queue, *wait_flag != SIS8300_IRQ_NONE);
        if (!status) {
            status = 1; /* Fake non-timeout return. */
        }
    }

    switch (status) {
        case 0:             /* Timeout elapsed. */
            irq->status = SIS8300_IRQ_TIMEOUT;
            break;
        case -ERESTARTSYS:  /* Error. */
            irq->status = SIS8300_IRQ_ERROR;
            return status;
        default:            /* Irq happened. */
            if (*wait_flag == SIS8300_IRQ_RELEASED) {
                irq->status = SIS8300_IRQ_RELEASE;
            } else {
                irq->status = SIS8300_IRQ_SUCCESS;
            }
            break;
    }

    return 0;
}


/**
 * Release waiters for daq irq user-defined irq (defined in custom firmware).
 * Which irq to release waiters for is determined by the irq type argument.
 */
int sis8300_release_irq(sis8300_dev *sisdevice, sis8300_irq *irq) {
    int                 *wait_flag;
    wait_queue_head_t   *wait_queue;
    
    switch (irq->type) {
        case SIS8300_DAQ_IRQ:
            wait_flag = &sisdevice->daq_irq_flag;
            wait_queue = &sisdevice->daq_irq_wait;
            break;
        case SIS8300_USR_IRQ:
            wait_flag = &sisdevice->usr_irq_flag;
            wait_queue = &sisdevice->usr_irq_wait;
            break;
        default:
            printk(KERN_ERR "%s: irq type unknown: 0x%x\n", sisdevice->name, irq->type);
            return -EINVAL;
    }
    
    *wait_flag = SIS8300_IRQ_RELEASED;
    wake_up_all(wait_queue);

    return 0;
}


/**
 * Syscall ioctl handler.
 *
 * Branch to the following ioctl calls:
 *  - SIS8300_REG_READ read device register specified by sis8300_reg struct.
 *  - SIS8300_REG_WRITE write to device register specified by sis8300_reg struct.
 *  - SIS8300_WAIT_IRQ block until end-of-daq or user-defined irq happens.
 *  - SIS8300_RELEASE_IRQ release all waiters for end-of-daq or user-defined irq.
 */
long sis8300_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    sis8300_dev    *sisdevice = SIS8300_DEV_FILE(filp);
    uint8_t        data[_IOC_SIZE(cmd)];
    int            status;

    if (_IOC_TYPE(cmd) != SIS8300_MAGIC) {
        printk(KERN_ERR "%s: unknown ioctl command: 0x%x, 0x%x, %c\n", sisdevice->name, cmd, _IOC_TYPE(cmd), _IOC_TYPE(cmd));
        return -EINVAL;
    }

    /* If this is a write ioctl call I need to get data from userspace. */
    if (cmd & IOC_IN) {
        if (copy_from_user(&data, (void *)arg, _IOC_SIZE(cmd))) {
            printk(KERN_ERR "%s: ioctl command nr not implemented: 0x%x\n", sisdevice->name, cmd);
            return -EFAULT;
        }
    }

    switch (cmd) {
        case SIS8300_REG_READ:
            mutex_lock(&sisdevice->lock);
            if (!sisdevice->available) {
                printk(KERN_ERR "%s: device not physically present\n", sisdevice->name);
                mutex_unlock(&sisdevice->lock);
                return -ENODEV;
            }
            status = sis8300_register_read(sisdevice,
                    ((sis8300_reg *)data)->offset, &((sis8300_reg *)data)->data);
            mutex_unlock(&sisdevice->lock);
            break;
        case SIS8300_REG_WRITE:
            mutex_lock(&sisdevice->lock);
            if (!sisdevice->available) {
                printk(KERN_ERR "%s: device not physically present\n", sisdevice->name);
                mutex_unlock(&sisdevice->lock);
                return -ENODEV;
            }
            status = sis8300_register_write(sisdevice,
                    ((sis8300_reg *)data)->offset, ((sis8300_reg *)data)->data);
            mutex_unlock(&sisdevice->lock);
            break;
        case SIS8300_WAIT_IRQ:
            status = sis8300_wait_irq(sisdevice, (sis8300_irq *)data);
            break;
        case SIS8300_RELEASE_IRQ:
            status = sis8300_release_irq(sisdevice, (sis8300_irq *)data);
            break;
        default:
            printk(KERN_ERR "%s: ioctl command nr not implemented: 0x%x\n", sisdevice->name, cmd);
            return -EINVAL;
    }

    /* If this is a write ioctl call we need to get data to userspace. */
    if (cmd & IOC_OUT) {
        if (copy_to_user((void *)arg, &data, _IOC_SIZE(cmd))) {
            return -EFAULT;
        }
    }

    return status;
}
