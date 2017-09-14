/*
 * SIS8300 read syscall handler.
 */

#include <linux/fs.h>
#include <asm/uaccess.h>

#include "sis8300.h"
#include "sis8300_defs.h"
#include "sis8300_reg.h"
#include "sis8300_dma.h"
#include "sis8300_read.h"


/**
 * Syscall read handler.
 *
 * Read the contents of device memory specified by offset and size into
 * a buffer available in userspace. This can happen in two different ways 
 * depending on how the buffer was allocated.
 * 
 * If the buffer was allocated in the conventional way (malloc or similar)
 * a single dma buffer (belonging to the device context) is used. The actual 
 * dma transfer is done in chunks the size of the buffer while copying 
 * each transfered chunk into provided userspace buffer.
 * 
 * If the buffer was "allocated" with mmap (which mapped in-kernel allocated 
 * physically contignuous memory to userspace) then a single dma transfer is
 * performed into that memory (avoiding the copy into userspace memory).
 */
ssize_t sis8300_read(struct file *filp, char __user *buff, size_t count, loff_t *offp) {
    sis8300_usr     *sisusr = SIS8300_USR_FILE(filp);
    sis8300_dev     *sisdevice = SIS8300_DEV_FILE(filp);
    sis8300_buf     *sisbuf, *entry;
    unsigned int    remaining, length, bufsize;
    int             status;
    
    dev_dbg(&sisdevice->pdev->dev, "read: offset=0x%lx size=0x%lx\n",
            (unsigned long)*offp, (unsigned long)count);
    
    if (count % SIS8300_DMA_ALLIGN) {
        printk(KERN_ERR "%s: read size must be a multiple of %u\n", 
                sisdevice->name, SIS8300_DMA_ALLIGN);
        return -EINVAL;
    }
    
    mutex_lock(&sisdevice->lock);
    
    if (!sisdevice->available) {
        printk(KERN_ERR "%s: device not physically present\n", sisdevice->name);
        mutex_unlock(&sisdevice->lock);
        return -ENODEV;
    }
    
    /* Check if the buffer is a mmaped one so that we can do zero copy. */
    sisbuf = NULL;
    mutex_lock(&sisusr->buflist_lock);
    list_for_each_entry(entry, &sisusr->buflist, list) {
        if (entry->uaddr == (unsigned long)buff) {
            sisbuf = entry;
        }
    }
    mutex_unlock(&sisusr->buflist_lock);
    
    if (sisbuf) {
        /* Zero-copy read. */
        dev_dbg(&sisdevice->pdev->dev, "read into mmap: uaddr=0x%lx, kaddr=0x%lx\n", 
                sisbuf->uaddr, sisbuf->kaddr);
        
        status = sis8300_dma_read(sisdevice, sisbuf, *offp, count);
        if (unlikely(status)) {
            mutex_unlock(&sisdevice->lock);
            return status;
        }
    } else {
        /* Regular bounce-buffer read. */
        dev_dbg(&sisdevice->pdev->dev, "read into kernel buffer\n");
        
        sisbuf = &sisdevice->dmabuf;
        remaining = count;
        while (remaining > 0) {
            bufsize = SIS8300_SIZE_ORDER(sisbuf->order);
            length = min(remaining, bufsize);

            status = sis8300_dma_read(sisdevice, sisbuf, *offp, length);
            if (unlikely(status)) {
                mutex_unlock(&sisdevice->lock);
                return status;
            }
            
            status = copy_to_user(buff, (void *)sisbuf->kaddr, length);
            if (unlikely(status)) {
                printk(KERN_ERR "%s: copy_to_user failed\n", sisdevice->name);
                mutex_unlock(&sisdevice->lock);
                return status;
            }
            
            buff += length;
            remaining -= length;
            *offp += length;
        }
    }
    
    mutex_unlock(&sisdevice->lock);
    
    return count;
}
