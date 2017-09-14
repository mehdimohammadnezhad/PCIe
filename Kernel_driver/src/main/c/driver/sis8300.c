/*
 * SIS8300 Device driver.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include "sis8300.h"
#include "sis8300_defs.h"
#include "sis8300_reg.h"
#include "sis8300_ioctl.h"
#include "sis8300_read.h"
#include "sis8300_write.h"
#include "sis8300_llseek.h"
#include "sis8300_mmap.h"
#include "sis8300_dma.h"
#include "sis8300_irq.h"


LIST_HEAD(sis8300_devlist);             /**< List of device context structs for available devices. */
DEFINE_MUTEX(sis8300_devlist_lock);     /**< Lock that serializes access to the list of available devices. */


MODULE_AUTHOR("SIS GmbH <info@struck.de>");
MODULE_DESCRIPTION("SIS8300/SIS8300L/SIS8300L2/SIS8300KU uTCA.4 Digitizer");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_SUPPORTED_DEVICE("sis8300");


/**
 * PCI device struct.
 *
 * Driver supports SIS8300L also which has a different PCI ID.
 */
const struct pci_device_id sis8300_id[] = {
    { PCI_DEVICE(PCI_VENDOR_FZJZEL, PCI_PRODUCT_SIS8300)   },
    { PCI_DEVICE(PCI_VENDOR_FZJZEL, PCI_PRODUCT_SIS8300L)  },
    { PCI_DEVICE(PCI_VENDOR_FZJZEL, PCI_PRODUCT_SIS8300KU) },
    { 0 }
};

MODULE_DEVICE_TABLE(pci, sis8300_id);

static struct class *sis8300_drvclass;
static const char *sis8300_drvclass_name = "sis8300";


int sis8300_open(struct inode *, struct file *);
int sis8300_release(struct inode *, struct file *);

static int init_sis8300(struct pci_dev *pdev, const struct pci_device_id *ent);
static void exit_sis8300(struct pci_dev *pdev);


/**
 * PCI driver struct.
 */
static struct pci_driver sis8300_driver = {
    .name = "sis8300",
    .id_table = sis8300_id,
    .probe = init_sis8300,
    .remove = exit_sis8300,
};


/**
 * File operations struct.
 */
static struct file_operations sis8300_fops = {
    .owner = THIS_MODULE,
    .open = sis8300_open,
    .read = sis8300_read,
    .write = sis8300_write,
    .llseek = sis8300_llseek,
    .unlocked_ioctl = sis8300_ioctl,
    .mmap = sis8300_mmap,
    .release = sis8300_release,
};


/**
 * Syscall open handler.
 *
 * Search the list of available devices and if a device is found
 * update the file node size, increment the device reference count,
 * allocate user context and associate it with the file descriptor.
 */
int sis8300_open(struct inode *inode, struct file *filp) {
    sis8300_usr *sisusr;
    sis8300_dev *sisdevice;
    
    sisusr = kzalloc(sizeof(sis8300_usr), GFP_KERNEL);
    if (!sisusr) {
        printk(KERN_ERR "%s: can't allocate user context\n", sis8300_drvclass_name);
        return -ENOMEM;
    }  

    mutex_lock(&sis8300_devlist_lock);
    list_for_each_entry(sisdevice, &sis8300_devlist, list) {
        if (MAJOR(sisdevice->drvnum) == imajor(inode)
                && MINOR(sisdevice->drvnum) == iminor(inode)) {
            dev_dbg(&sisdevice->pdev->dev, "open fd\n");        
            
            sisusr->sisdevice = sisdevice;
            INIT_LIST_HEAD(&sisusr->buflist);
            mutex_init(&sisusr->buflist_lock);
            filp->private_data = sisusr;
            sisdevice->count++;
            mutex_unlock(&sis8300_devlist_lock);
            return 0;
            
        }
    }
    mutex_unlock(&sis8300_devlist_lock);
    
    printk(KERN_ERR "%s: device not found\n", sis8300_drvclass_name);
    kfree(sisusr);

    return -ENODEV;
}


/**
 * Syscall close handler.
 *
 * Decrement reference count for the device context associated with
 * the file descriptor and free the user context. Mappings (via mmap)
 * associated with the user context will all have been freed already
 * since linux calls unmmap (where the kernel memory associated with
 * mapping is freed) when the file descriptor is closed.
 * 
 * If the device is not physically present in the system anymore and
 * if we have the last file descriptor associated with it the device
 * context is freed.
 */
int sis8300_release(struct inode *inode, struct file *filp) {
    sis8300_usr     *sisusr = SIS8300_USR_FILE(filp);
    sis8300_dev     *sisdevice = SIS8300_DEV_FILE(filp);

    mutex_lock(&sis8300_devlist_lock);
    sisdevice->count--;
    if (!sisdevice->available && sisdevice->count < 1) {
        printk(KERN_INFO "%s: close - free device context\n", sisdevice->name);
        kfree(sisdevice);
    }
    mutex_unlock(&sis8300_devlist_lock);
    
    kfree(sisusr);
    filp->private_data = NULL;

    return 0;
}


/**
 * PCI driver probe function.
 *
 * Create and initialize device context and do standard PCI device initialization.
 *
 * The name of the device node is set to "sis8300-x" where x is the physical
 * slot in the crate. If the slot is not PCIe capable (in which case the
 * physical slot can't be accurately determined) then x is the PCI bus signature.
 */
static int init_sis8300(struct pci_dev *pdev, const struct pci_device_id *ent) {
    int         status;
    uint32_t    bar0_size, fw_version, fw_options, slotcap, slotno;
    uint16_t    linkstatus, pcieflags;
    struct      pci_dev *slotdev;
    sis8300_dev *sisdevice;

    /* Allocate kernel memory for the device struct. */
    sisdevice = kzalloc(sizeof(sis8300_dev), GFP_KERNEL);
    if (!sisdevice) {
        printk(KERN_ERR "%s: can't allocate device context\n", sis8300_drvclass_name);
        status = -ENOMEM;
        goto err_kdevice;
    }
    
    pci_set_drvdata(pdev, sisdevice);
    sisdevice->pdev = pdev;

    slotdev = pdev->bus->self;
    pcieflags = 0;

    if (pci_is_pcie(slotdev)) {
        pci_read_config_word(slotdev, pci_pcie_cap(slotdev) + PCI_EXP_FLAGS, &pcieflags);
    }

    if (pcieflags & PCI_EXP_FLAGS_SLOT) {
        /* Construct device name based on slot number. */
        pci_read_config_dword(slotdev, pci_pcie_cap(slotdev) + PCI_EXP_SLTCAP, &slotcap);
        slotno = (slotcap >> 19) & 0x1FFF;
        sprintf(sisdevice->name, "sis8300-%u", slotno);
        dev_dbg(&sisdevice->pdev->dev, "%s\n", sisdevice->name);
        dev_dbg(&sisdevice->pdev->dev, "device in physical PCIe slot #%u\n", slotno);
    } else {
        /* Construct device name based on the PCI signature. */
        sprintf(sisdevice->name, "sis8300-%04x:%02x:%02x.%d",
                pci_domain_nr(pdev->bus),
                pdev->bus->number,
                PCI_SLOT(pdev->devfn),
                PCI_FUNC(pdev->devfn));
        dev_dbg(&sisdevice->pdev->dev, "%s\n", sisdevice->name);
        dev_dbg(&sisdevice->pdev->dev, "device not in a PCIe slot\n");
    }
    
    /* XXX: Fix PCI class until Struck provides fixed firmware (if ever). */
    if (pdev->class == 0x00078000) {
        /* SIS8300 card is 'Signal processing controller' */
        pdev->class = 0x00118000;
    }
    if (pdev->class == 0x00ff0000) {
        /* SIS8300L/SIS8300L2 card is 'Signal processing controller' */
        pdev->class = 0x00118000;
    }
    if (pdev->class == 0x00070000) {
        /* SIS8300KU card is 'Signal processing controller' */
        pdev->class = 0x00118000;
    }
    dev_dbg(&sisdevice->pdev->dev, "vendor/device id %04x/%04x, class %08X\n",
            pdev->vendor, pdev->device, pdev->class);

    /* Enable device. */
    status = pci_enable_device(pdev);
    if (status) {
        printk(KERN_ERR "%s: can't enable pci device: %d\n", sisdevice->name, status);
        goto err_endevice;
    }

    /* Read PCIe link status from the pci config address space. */
    pci_read_config_word(pdev, 0x72, &linkstatus);
    linkstatus = (linkstatus & 0x1F0) >> 4;
    dev_dbg(&sisdevice->pdev->dev, "PCIe link status: %i lane%sat 2.5Gb/s%s\n",
            linkstatus,
            linkstatus > 1 ? "s " : " ",
            linkstatus > 1 ? " each" : "");

    /* Reserve PCI resources for this kernel module. */
    status = pci_request_regions(pdev, "sis8300");
    if (status) {
        printk(KERN_ERR "%s: can't request pci regions: %d\n", 
                sisdevice->name, status);
        goto err_reqregions;
    }

    /* Map bar 0. */
    sisdevice->bar0 = pci_iomap(pdev, 0, 0);
    if (!sisdevice->bar0) {
        printk(KERN_ERR "%s: can't get bar0 address\n", sisdevice->name);
        goto err_iomap;
    }
    bar0_size = pci_resource_len(pdev, 0);
    dev_dbg(&sisdevice->pdev->dev, "bar0 mapped: addr=%p, size=0x%x\n", 
        sisdevice->bar0, bar0_size);

    /* Read firmware revision. */
    status = sis8300_register_read(sisdevice, 
            SIS8300_IDENTIFIER_VERSION_REG, &fw_version);
    if (unlikely(status)) {
        printk(KERN_ERR "%s: can't read device registers\n", sisdevice->name);
        goto err_cdevreg;
    }
    dev_dbg(&sisdevice->pdev->dev, "device firmware/revision: 0x%x\n", fw_version);

    /* Check witch version of the card we have. */
    status = sis8300_register_read(sisdevice, 
            SIS8300_FIRMWARE_OPTIONS_REG, &fw_options);
    if (unlikely(status)) {
        printk(KERN_ERR "%s: can't read device registers\n", sisdevice->name);
        goto err_cdevreg;
    }
    dev_dbg(&sisdevice->pdev->dev, "device firmware options 0x%x\n", fw_options);
    
    printk(KERN_INFO "%s: device info: vid/did=%04x/%04x, version=0x%x, options=0x%x\n", 
            sisdevice->name, pdev->vendor, pdev->device, fw_version, fw_options);

    /* Create char device. */
    status = alloc_chrdev_region(&(sisdevice->drvnum), 0, 1, sisdevice->name);
    if (status) {
        printk(KERN_ERR "%s: can't allocate char device range: %d\n", 
                sisdevice->name, status);
        goto err_cdevreg;
    }
    dev_dbg(&sisdevice->pdev->dev, "char device range allocated\n");

    sisdevice->drvcdev = cdev_alloc();
    if(!sisdevice->drvcdev) {
        printk(KERN_ERR "%s: can't allocate cdev\n", sisdevice->name);
        status = -ENOMEM;
        goto err_cdevadd;
    }

    cdev_init(sisdevice->drvcdev, &sis8300_fops);
    status = cdev_add(sisdevice->drvcdev, sisdevice->drvnum, 1);
    if (status) {
        printk(KERN_ERR "%s: can't add cdev: %d\n", sisdevice->name, status);
        goto err_cdevadd;
    }
    dev_dbg(&sisdevice->pdev->dev, "char device added\n");

    sisdevice->drvdevice = device_create(sis8300_drvclass, &pdev->dev, 
            sisdevice->drvnum, NULL, sisdevice->name);
    if (IS_ERR(sisdevice->drvdevice)) {
        printk(KERN_ERR "%s: can't create device\n", sisdevice->name);
        status = PTR_ERR(sisdevice->drvdevice);
        goto err_devcreate;
    }
    dev_dbg(&sisdevice->pdev->dev, "device created: major=%d, minor=%d\n", 
            MAJOR(sisdevice->drvnum), MINOR(sisdevice->drvnum));

    /* Setup dma operation. */
    if (dma_set_mask(&sisdevice->pdev->dev, DMA_BIT_MASK(64))) {
        printk(KERN_ALERT "%s: unable to set DMA(64) mask\n", sisdevice->name);
        if (dma_set_mask(&sisdevice->pdev->dev, DMA_BIT_MASK(32))) {
            printk(KERN_ERR "%s: unable to set DMA(32) mask\n", sisdevice->name);
            goto err_nodmamem_rw;
        }
    }

    /* Allocate kernel memory for read/write dma operations. */
    status = sis8300_dma_alloc(sisdevice, &sisdevice->dmabuf);
    if (status) {
        printk(KERN_ERR "%s: unable to allocate dma buffer\n", sisdevice->name);
        goto err_nodmamem_rw;
    }
    dev_dbg(&sisdevice->pdev->dev, "read/write dma buffer: kaddr=0x%lx, size=0x%x\n", 
            sisdevice->dmabuf.kaddr, SIS8300_SIZE_ORDER(sisdevice->dmabuf.order));

    /* Claim irq line. */
    dev_dbg(&sisdevice->pdev->dev, "claiming irq line %i\n", pdev->irq);
    status = request_irq(sisdevice->pdev->irq,
            (void *)&sis8300_isr, IRQF_SHARED, sis8300_drvclass_name, sisdevice);
    if (status) {
        printk(KERN_ERR "%s: can't register irq handler\n", sisdevice->name);
        goto err_reqirq;
    }

    /* Intialize irq wait queues and flags. */
    init_waitqueue_head(&sisdevice->dma_irq_wait);
    sisdevice->dma_irq_flag = SIS8300_IRQ_NONE;
    
    init_waitqueue_head(&sisdevice->daq_irq_wait);
    sisdevice->daq_irq_flag = SIS8300_IRQ_NONE;

    init_waitqueue_head(&sisdevice->usr_irq_wait);
    sisdevice->usr_irq_flag = SIS8300_IRQ_NONE;
    
    /* Initialize device lock. */
    mutex_init(&sisdevice->lock);
    
    /* Enable all interrupts. The stock driver always disabled all interrupts
     * and then just enabled the appropriate one during dma read/write.
     * Revert to old behaviour if unexplainable things start happening. */
    sis8300_register_write(sisdevice, IRQ_ENABLE, IRQ_MASTER_ENABLE);

    /* Add device context to the list of available devices. */
    sisdevice->available = 1;
    sisdevice->count = 0;
    mutex_lock(&sis8300_devlist_lock);
    list_add_tail(&sisdevice->list, &sis8300_devlist);
    mutex_unlock(&sis8300_devlist_lock);

    return 0;

/* Properly unwind allocations and mappings on error. */
err_reqirq:
    sis8300_dma_free(sisdevice, &sisdevice->dmabuf);

err_nodmamem_rw:
    device_destroy(sis8300_drvclass, sisdevice->drvnum);

err_devcreate:
    cdev_del(sisdevice->drvcdev);

err_cdevadd:
    unregister_chrdev_region(sisdevice->drvnum, 1);

err_cdevreg:
    pci_iounmap(pdev, sisdevice->bar0);

err_iomap:
    pci_release_regions(pdev);

err_reqregions:
    pci_disable_device(pdev);

err_endevice:
    pci_set_drvdata(pdev, NULL);
    kfree(sisdevice);

err_kdevice:
    return status;
}


#ifndef __devexit
#define __devexit
#endif


/**
 * PCI device remove function.
 *
 * Do standard PCI removal procedure and remove the device context from the
 * list of available devices.
 * 
 * While the device resources are being freed the #sis8300_devlist_lock is
 * taken so that the file desriptor isn't closed simultaneosly and the device
 * context isn't freed twice. The device context lock is taken so that
 * syscalls can rely on the "available" flag to see if the device can be 
 * accessed.
 * 
 * If there are no more file descriptors associated with the device then 
 * the device context is freed. Otherwise that will happen when the last
 * file descriptor is closed.
 */
static void __devexit exit_sis8300(struct pci_dev *pdev) {
    sis8300_dev *sisdevice = pci_get_drvdata(pdev);
    
    dev_dbg(&sisdevice->pdev->dev, "remove device\n");
    
    mutex_lock(&sis8300_devlist_lock);
    
    mutex_lock(&sisdevice->lock);
    
    sisdevice->available = 0;
    list_del(&sisdevice->list);
     
    /* Stop board activity, disable and clear interrupts. */
    sis8300_register_write(sisdevice, SIS8300_ACQUISITION_CONTROL_STATUS_REG, 0x4);
    sis8300_register_write(sisdevice, IRQ_ENABLE, IRQ_MASTER_DISABLE);
    sis8300_register_write(sisdevice, IRQ_CLEAR, IRQ_MASTER_CLEAR);
    
    free_irq(sisdevice->pdev->irq, sisdevice);
    
    /* Wake up anybody that is waiting for interrupts. We know that no
     * dma transfers are active while the device lock is held so no need
     * to wake them up. */ 
    sisdevice->daq_irq_flag = SIS8300_IRQ_RELEASED;
    wake_up_all(&sisdevice->daq_irq_wait);
    
    sisdevice->usr_irq_flag = SIS8300_IRQ_RELEASED;
    wake_up_all(&sisdevice->usr_irq_wait);

    sis8300_dma_free(sisdevice, &sisdevice->dmabuf);

    /* Clean up pci-related things. */
    device_destroy(sis8300_drvclass, sisdevice->drvnum);
    cdev_del(sisdevice->drvcdev);
    unregister_chrdev_region(sisdevice->drvnum, 1);
    pci_disable_device(sisdevice->pdev);
    pci_release_regions(sisdevice->pdev);
    pci_set_drvdata(sisdevice->pdev, NULL);
    pci_iounmap(sisdevice->pdev, sisdevice->bar0);

    mutex_unlock(&sisdevice->lock);
    
    if (sisdevice->count < 1) {
        dev_dbg(&sisdevice->pdev->dev, "remove: free device context\n");
        kfree(sisdevice);
    }
    
    mutex_unlock(&sis8300_devlist_lock);
}


/**
 * Kernel module init function.
 *
 * Called on module load. Creates a class of char devices so that /dev nodes
 * for all devices are created in one folder and minor numbers are handled
 * automatically. Registers the PCI driver with the kernel.
 */
static int __init sis8300_module_init(void) {
    int status;

    printk(KERN_INFO "%s: loading driver\n", sis8300_drvclass_name);
    printk(KERN_INFO "%s: registering for following devices:\n", 
            sis8300_drvclass_name);
    printk(KERN_INFO "%s:     vendor/device: %04x/%04x\n", 
            sis8300_drvclass_name, PCI_VENDOR_FZJZEL, PCI_PRODUCT_SIS8300);
    printk(KERN_INFO "%s:     vendor/device: %04x/%04x\n", 
            sis8300_drvclass_name, PCI_VENDOR_FZJZEL, PCI_PRODUCT_SIS8300L);
    printk(KERN_INFO "%s:     vendor/device: %04x/%04x\n",
            sis8300_drvclass_name, PCI_VENDOR_FZJZEL, PCI_PRODUCT_SIS8300KU);
    printk(KERN_INFO "%s: driver Version: v%d.%d (c)\n", 
            sis8300_drvclass_name, DRIVER_MAJOR, DRIVER_MINOR);

    sis8300_drvclass = class_create(THIS_MODULE, sis8300_drvclass_name);
    if (IS_ERR(sis8300_drvclass)) {
        status = PTR_ERR(sis8300_drvclass);
        printk(KERN_ERR "%s: can't create device class: %d\n", 
                sis8300_drvclass_name, status);
        return status;
    }

    status = pci_register_driver(&sis8300_driver);
    if (status) {
        printk(KERN_ERR "%s: can't register pci driver: %d\n", 
                sis8300_drvclass_name, status);
        return status;
    }

    return 0;
}


/**
 * Kernel module exit function.
 */
static void __exit sis8300_module_exit(void) {
    pci_unregister_driver(&sis8300_driver);
    class_destroy(sis8300_drvclass);

    printk(KERN_INFO "%s: driver unloaded\n", sis8300_drvclass_name);
}


module_init(sis8300_module_init);
module_exit(sis8300_module_exit);
