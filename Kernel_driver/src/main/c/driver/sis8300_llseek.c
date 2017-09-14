/*
 * SIS8300 llseek syscall handler.
 */
#include <linux/fs.h>

#include "sis8300.h"
#include "sis8300_defs.h"
#include "sis8300_reg.h"
#include "sis8300_dma.h"
#include "sis8300_llseek.h"


/**
 * Syscall llseek handler.
 */
loff_t sis8300_llseek(struct file *filp, loff_t offset, int origin) {
    sis8300_dev *sisdevice = SIS8300_DEV_FILE(filp);
    loff_t      status;
    
    dev_dbg(&sisdevice->pdev->dev, "seek: offset=0x%lx\n", 
            (unsigned long)offset);
    
    if (offset % SIS8300_DMA_ALLIGN) {
        printk(KERN_ERR "%s: read/write offset must be a multiple of %u\n", 
                sisdevice->name, SIS8300_DMA_ALLIGN);
        return -EINVAL;
    }

    mutex_lock(&sisdevice->lock);

    if (!sisdevice->available) {
        printk(KERN_ERR "%s: device not physically present\n", sisdevice->name);
        mutex_unlock(&sisdevice->lock);
        return -ENODEV;
    }

    status = generic_file_llseek(filp, offset, origin);

    mutex_unlock(&sisdevice->lock);

    return status;
}
