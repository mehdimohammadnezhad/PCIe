/*
 * SIS8300 dma memory allocation/deallocation and dma transfer handling.
 */

#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>

#include "sis8300.h"
#include "sis8300_defs.h"
#include "sis8300_reg.h"
#include "sis8300_dma.h"


/**
 * Release memory for dma transfers allocated with #sis8300_dma_alloc.
 */
void sis8300_dma_free(sis8300_dev *sisdevice, sis8300_buf *sisbuf) {
    int    iter;
    
    printk(KERN_INFO "%s: free dma buffer: kaddr=0x%lx, size=0x%x (%d)\n", 
            sisdevice->name, sisbuf->kaddr, 
            SIS8300_SIZE_ORDER(sisbuf->order), sisbuf->order);

    for (iter = 0; iter < SIS8300_SIZE_ORDER(sisbuf->order); iter += PAGE_SIZE) {
        ClearPageReserved(virt_to_page(sisbuf->kaddr + iter));
    }
    free_pages(sisbuf->kaddr, sisbuf->order);
}


/**
 * Allocate memory for dma transfers.
 *
 * Try to allocate a physically contiguous block of memory as large as possible
 * and obtain a dma handle for that memory.
 */
int sis8300_dma_alloc(sis8300_dev *sisdevice, sis8300_buf *sisbuf) {
    unsigned int    order;
    int             iter;

    order = MAX_ORDER;
    while (order-- >= 0) {
        sisbuf->kaddr = __get_free_pages(GFP_KERNEL | GFP_DMA32 | __GFP_COMP, order);
        if (!sisbuf->kaddr) {
            printk(KERN_WARNING "%s: can't get free pages of order %u\n", 
                    sisdevice->name, order);
            continue;
        }

        sisbuf->order = order;
        for (iter = 0; iter < SIS8300_SIZE_ORDER(sisbuf->order); iter += PAGE_SIZE) {
            SetPageReserved(virt_to_page(sisbuf->kaddr + iter));
        }
        
        printk(KERN_INFO "%s: allocated dma buffer: kaddr=0x%lx, size=0x%x (%d)\n", 
                sisdevice->name, sisbuf->kaddr, 
                SIS8300_SIZE_ORDER(sisbuf->order), sisbuf->order);

        return 0;
    }

    return -ENOMEM;
}


/**
 * Transfer a specified region of device memory into a kernel buffer and block 
 * until dma transfer completes.
 */
int sis8300_dma_read(sis8300_dev *sisdevice, sis8300_buf *sisbuf, uint32_t offset, uint32_t count) {
    uint32_t    destination;
    dma_addr_t  dma_handle;
    
    dma_handle = dma_map_single(&sisdevice->pdev->dev, (void *)sisbuf->kaddr, 
            SIS8300_SIZE_ORDER(sisbuf->order), DMA_FROM_DEVICE);
    if (dma_mapping_error(&sisdevice->pdev->dev, dma_handle)) {
        printk(KERN_ERR "%s: dma mapping error: kaddr=0x%lx\n", 
                sisdevice->name, sisbuf->kaddr);
        return -ENOMEM;
    }

    /* Set source and destination address. */
    sis8300_register_write(sisdevice, DMA_READ_SRC_ADR_LO32, offset);

    destination = (uint32_t)dma_handle;
    sis8300_register_write(sisdevice, DMA_READ_DST_ADR_LO32, destination);

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
    destination = (uint32_t)(dma_handle >> 32);
#else
    destination = 0;
#endif
    sis8300_register_write(sisdevice, DMA_READ_DST_ADR_HI32, destination);

    /* Set transfer length. */
    sis8300_register_write(sisdevice, DMA_READ_LEN, count);
    
    sis8300_register_read(sisdevice, DMA_WRITE_SRC_ADR_LO32, &destination);

    /* Stock driver disabled all interrupts and then and reenabled
     * just DMA_READ_DONE interrupt. Now all interrupts are enabled during
     * device init. Revert to old behaviour if unexplainable things start happening. */

    /* Start dma transfer. */
    sisdevice->dma_irq_flag = SIS8300_IRQ_NONE;
    sis8300_register_write(sisdevice, DMA_READ_CTRL, 1 << DMA_READ_START);

    wait_event_interruptible(sisdevice->dma_irq_wait, 
            sisdevice->dma_irq_flag != SIS8300_IRQ_NONE);
            
    dma_unmap_single(&sisdevice->pdev->dev, dma_handle, 
            SIS8300_SIZE_ORDER(sisbuf->order), DMA_FROM_DEVICE);

    return 0;
}


/**
 * Transfer a kernel buffer into a specified region of device memory and block
 * until dma transfer completes.
 */
int sis8300_dma_write(sis8300_dev *sisdevice, sis8300_buf *sisbuf, uint32_t offset, uint32_t count) {
    uint32_t    destination;
    dma_addr_t  dma_handle;
    
    dma_handle = dma_map_single(&sisdevice->pdev->dev, (void *)sisbuf->kaddr, 
            SIS8300_SIZE_ORDER(sisbuf->order), DMA_TO_DEVICE);
    if (dma_mapping_error(&sisdevice->pdev->dev, dma_handle)) {
        printk(KERN_ERR "%s: dma mapping error: kaddr=0x%lx\n", 
                sisdevice->name, sisbuf->kaddr);
        return -ENOMEM;
    }

    /* Set source and destination address. */
    sis8300_register_write(sisdevice, DMA_WRITE_DST_ADR_LO32, offset);

    destination = (uint32_t)dma_handle;
    sis8300_register_write(sisdevice, DMA_WRITE_SRC_ADR_LO32, destination);

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
    destination = (uint32_t)(dma_handle >> 32);
#else
    destination = 0;
#endif
    sis8300_register_write(sisdevice, DMA_WRITE_SRC_ADR_HI32, destination);

    /* Set transfer length. */
    sis8300_register_write(sisdevice, DMA_WRITE_LEN, count);
    
    sis8300_register_read(sisdevice, DMA_WRITE_SRC_ADR_LO32, &destination);

    /* Stock driver disabled all interrupts and then and reenabled
     * just DMA_WRITE_DONE interrupt. Now all interrupts are enabled during
     * device init. Revert to old behaviour if unexplainable things start happening. */
     
    /* Start dma transfer. */
    sisdevice->dma_irq_flag = SIS8300_IRQ_NONE;
    sis8300_register_write(sisdevice, DMA_WRITE_CTRL, 1 << DMA_WRITE_START);

    wait_event_interruptible(sisdevice->dma_irq_wait, 
            sisdevice->dma_irq_flag != SIS8300_IRQ_NONE);
            
    dma_unmap_single(&sisdevice->pdev->dev, dma_handle, 
            SIS8300_SIZE_ORDER(sisbuf->order), DMA_TO_DEVICE);

    return 0;
}
