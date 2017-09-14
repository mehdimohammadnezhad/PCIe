/*
 * SIS8300 mmap syscall handling.
 */

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <asm/io.h>

#include "sis8300.h"
#include "sis8300_defs.h"
#include "sis8300_dma.h"
#include "sis8300_mmap.h"


/**
 * mmap open handler.
 *
 * Nothing needs to happen here.
 */
static void sis8300_vm_open(struct vm_area_struct *vma) {}


/**
 * mmap close handler.
 *
 * Delete mapping from list of mappings and free kernel memory allocated
 * for dma transfers. This is the only place where mappings are freed since
 * linux calls unmmap() on all mappings when the file descriptor is closed.
 */
static void sis8300_vm_close(struct vm_area_struct *vma) {
    sis8300_usr *sisusr = vma->vm_private_data;
    sis8300_dev *sisdevice = sisusr->sisdevice;
    sis8300_buf *sisbuf, *entry;
    
    printk(KERN_INFO "%s: close mapping: uaddr=0x%lx\n", 
            sisdevice->name, vma->vm_start);

    mutex_lock(&sisusr->buflist_lock);
    list_for_each_entry_safe(sisbuf, entry, &sisusr->buflist, list) {
        if (sisbuf->uaddr == vma->vm_start) {
            list_del(&sisbuf->list);
            sis8300_dma_free(sisusr->sisdevice, sisbuf);
            kfree(sisbuf);
        }
    }
    mutex_unlock(&sisusr->buflist_lock);
}


/**
 * mmap page fault handler.
 *
 * All pages that were mapped to userspace have already been allocated so this
 * should never happen.
 */
static int sis8300_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmfm) {
    sis8300_usr *sisusr = vma->vm_private_data;
    sis8300_dev *sisdevice = sisusr->sisdevice;
    
    printk(KERN_INFO "%s: fault mapping: uaddr=0x%lx\n", 
            sisdevice->name, vma->vm_start);
    
    /* Disallow faulting in any additional pages; if NULL the kernel would
    allocate 'normal', non-contiguous memory. */
    return VM_FAULT_SIGBUS;
}


/**
 * mmap operations struct.
 */
static struct vm_operations_struct sis8300_vm_ops = {
    .open  = sis8300_vm_open,
    .close = sis8300_vm_close,
    .fault = sis8300_vm_fault,
};


/**
 * Syscall mmap handler.
 *
 * Allocate a physically contignuous memory buffer and map it into userspace.
 * The mappings are kept in a separate list for each user context (open file 
 * descriptor) and their unique id is the userspace address.
 */
int sis8300_mmap(struct file *filp, struct vm_area_struct *vma) {
    sis8300_usr     *sisusr = SIS8300_USR_FILE(filp);
    sis8300_dev     *sisdevice = SIS8300_DEV_FILE(filp);
    sis8300_buf     *sisbuf;
    unsigned long   size, mmap_pfn;
    int             status;

    size = vma->vm_end - vma->vm_start;
    size = (size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

    if (vma->vm_end < vma->vm_start) {
        return -EINVAL;
    }

    if ((vma->vm_flags & VM_EXEC) || !(vma->vm_flags & VM_READ)) {
        return -EACCES;
    }

    sisbuf = kzalloc(sizeof(sis8300_buf), GFP_KERNEL);
    if (!sisbuf) {
        printk(KERN_ERR "%s: can't allocate dma buffer context\n", sisdevice->name);
        status = -ENOMEM;
        goto err_nokernmem;
    }

    /* Allocate memory for mapping. */
    status = sis8300_dma_alloc(sisdevice, sisbuf);
    if (status) {
        printk(KERN_ERR "%s: unable to allocate dma buffer\n", sisdevice->name);
        goto err_nodmamem;
    }

    /* Check if the allocated buffer is big enough. */
    if (size > SIS8300_SIZE_ORDER(sisbuf->order)) {
        printk(KERN_ERR "%s: unable to allocate enough dma memory for mapping\n", sisdevice->name);
        status = -ENOMEM;
        goto err_nomap;
    }

    /* Map kernel memory into address space of a userspace process. */
    mmap_pfn = virt_to_phys((void *)sisbuf->kaddr) >> PAGE_SHIFT;
    status = remap_pfn_range(vma, vma->vm_start, mmap_pfn, size, vma->vm_page_prot);
    if (status) {
        printk(KERN_ERR "%s: mmap can't remap memory to userspace\n", sisdevice->name);
        goto err_nomap;
    }

    sisbuf->uaddr = vma->vm_start;
    
    dev_dbg(&sisdevice->pdev->dev, "setup mapping: uaddr=0x%lx, kaddr=0x%lx\n", 
            sisbuf->uaddr, sisbuf->kaddr);

    mutex_lock(&sisusr->buflist_lock);
    list_add_tail(&sisbuf->list, &sisusr->buflist);
    mutex_unlock(&sisusr->buflist_lock);

    /* VM_IO | VM_DONTEXPAND | VM_DONTDUMP are set by remap_pfn_range() so
     * vma->vm_flags |= VM_RESERVED is not needed anymore. */
    vma->vm_ops = &sis8300_vm_ops;
    vma->vm_private_data = sisusr;

    return 0;

err_nomap:
    sis8300_dma_free(sisdevice, sisbuf);

err_nodmamem:
    kfree(sisbuf);

err_nokernmem:
    return status;
}
