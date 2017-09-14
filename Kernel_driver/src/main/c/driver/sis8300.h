/*
 * SIS8300 Device driver.
 */

#ifndef SIS8300_H_
#define SIS8300_H_

#include <linux/pci.h>

#include "sis8300_defs.h"


#ifndef PCI_VENDOR_FZJZEL
#define PCI_VENDOR_FZJZEL           0x1796
#endif

#ifndef PCI_PRODUCT_SIS8300
#define PCI_PRODUCT_SIS8300         0x0018
#endif

#ifndef PCI_PRODUCT_SIS8300L
#define PCI_PRODUCT_SIS8300L        0x0019
#endif

#ifndef PCI_PRODUCT_SIS8300KU
#define PCI_PRODUCT_SIS8300KU       0x0024
#endif

#define DRIVER_MAJOR                2
#define DRIVER_MINOR                1

#define SIS8300_USR_FILE(filp)      ((sis8300_usr *)(filp)->private_data)
#define SIS8300_DEV_FILE(filp)      ((sis8300_dev *)SIS8300_USR_FILE(filp)->sisdevice)

#define SIS8300_SIZE_ORDER(order)   (1 << ((order) + PAGE_SHIFT))


/**
 * Buffer struct used for dma transfer memory.
 */
typedef struct t_sis8300_buf {
    struct list_head    list;           /**< List entry struct. */
    unsigned int        order;          /**< Buffer size in units of kernel page order. */
    unsigned long       kaddr;          /**< Kernel virtual address of buffer. */
    unsigned long       uaddr;          /**< Userspace virtual address of mapping. */
} sis8300_buf;


/**
 * Device context struct.
 */
typedef struct t_sis8300_dev {
    char                name[32];       /**< Device name. */
    struct list_head    list;           /**< Device list entry struct. */
    int                 count;          /**< Device users reference count. */
    int                 available;      /**< Device physically available flag. */
    struct mutex        lock;           /**< Lock that serializes access to the device with hotswap. */

    dev_t               drvnum;         /**< Kernel driver minor/major number struct. */
    struct pci_dev      *pdev;          /**< Kernel pci device struct, */
    struct device       *drvdevice;     /**< Kernel device struct. */
    struct cdev         *drvcdev;       /**< Kernel character device struct. */
    uint32_t __iomem    *bar0;          /**< Device register access. */
    sis8300_buf         dmabuf;         /**< Buffer for standard read/write dma transfers. */

    wait_queue_head_t   dma_irq_wait;   /**< Kernel wait queue for dma interrupts. */
    wait_queue_head_t   daq_irq_wait;   /**< Kernel wait queue for end-of-daq interrupts. */
    wait_queue_head_t   usr_irq_wait;   /**< Kernel wait queue for user defined interrupts. */
    int                 dma_irq_flag;   /**< Interrupt flag for dma interrupts. */
    int                 daq_irq_flag;   /**< Interrupt flag for end-of-daq interrupts. */
    int                 usr_irq_flag;   /**< Interrupt flag for user defined interrupts. */
} sis8300_dev;


/**
 * User context struct.
 */
typedef struct t_sis8300_usr {
    sis8300_dev         *sisdevice;     /**< Device context. */
    struct list_head    buflist;        /**< List of buffers obtained via mmap. */
    struct mutex        buflist_lock;   /**< Lock that serializes access to list of mmap buffers. */
} sis8300_usr;


/**
 * Register read helper function.
 *
 * Reads the value of the register and first checks if there is a possibility
 * of a MMIO read failure and if so asserts that by reading a register that
 * if known to contain a value different from ~0 which signifies a pci bus error.
 */
static __inline__ int sis8300_register_read(sis8300_dev *sisdevice, uint32_t offset, uint32_t *data) {
    *data = ioread32(sisdevice->bar0 + offset);;
    if (*data == ~0 && ioread32(sisdevice->bar0) == ~0) {
        return -ENODEV;
    }
    return 0;
}


/**
 * Register write helper function.
 */
static __inline__ int sis8300_register_write(sis8300_dev *sisdevice, uint32_t offset, uint32_t data) {
    iowrite32(data, sisdevice->bar0 + offset);
    return 0;
}


#endif /* SIS8300_H_ */
