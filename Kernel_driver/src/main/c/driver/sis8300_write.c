/*
 * SIS8300 write syscall handler.
 */

#include <linux/fs.h>
#include <asm/uaccess.h>

#include "sis8300.h"
#include "sis8300_defs.h"
#include "sis8300_reg.h"
#include "sis8300_dma.h"
#include "sis8300_write.h"


/**
 * Syscall read handler.
 *
 * Write the contents of provided userspace buffer into device memory specified
 * by offset and size. A single dma buffer (belonging to the device context) 
 * is used. The actual dma transfer is done in chunks the size of the buffer 
 * while copying each transfered chunk into provided userspace buffer.
 */
ssize_t sis8300_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp) {
    sis8300_dev     *sisdevice = SIS8300_DEV_FILE(filp);
    sis8300_buf     *sisbuf;
    unsigned int    remaining, length, bufsize;
    int             status;
    
    dev_dbg(&sisdevice->pdev->dev, "write: offset=0x%lx, size=0x%lx\n",
            (unsigned long)*offp, (unsigned long)count);
    
    if (count % SIS8300_DMA_ALLIGN) {
        printk(KERN_ERR "%s: write size must be a multiple of %u\n", 
                sisdevice->name, SIS8300_DMA_ALLIGN);
        return -EINVAL;
    }
    
    mutex_lock(&sisdevice->lock);
    
    if (!sisdevice->available) {
        printk(KERN_ERR "%s: device not physically present\n", sisdevice->name);
        mutex_unlock(&sisdevice->lock);
        return -ENODEV;
    }
        
    sisbuf = &sisdevice->dmabuf;
    remaining = count;
    while (remaining > 0) {
        bufsize = SIS8300_SIZE_ORDER(sisbuf->order);
        length = min(remaining, bufsize);
        
        status = copy_from_user((void *)sisbuf->kaddr, buff, length);
        if (unlikely(status)) {
            printk(KERN_ERR "%s: copy_from_user failed\n", sisdevice->name);
            mutex_unlock(&sisdevice->lock);
            return status;
        }
        
        status = sis8300_dma_write(sisdevice, sisbuf, *offp, length);
        if (unlikely(status)) {
            mutex_unlock(&sisdevice->lock);
            return status;
        }
        
        buff += length;
        remaining -= length;
        *offp += length;
    }

    mutex_unlock(&sisdevice->lock);
    
    return count;
}
