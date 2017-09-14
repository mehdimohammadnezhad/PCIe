/*
 * SIS8300 interrupt Service Routine.
 */

#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include "sis8300.h"
#include "sis8300_reg.h"


/*
 * Interrupt service routine.
 *
 * Check if our card fired an interrupt, clear it and wake appropriate waiters.
 */
irqreturn_t sis8300_isr(int irq, void *dev_id) {
    sis8300_dev *sisdevice = (sis8300_dev *)dev_id;
    uint32_t    intreg;

    /* Did our card issue an irq? */
    sis8300_register_read(sisdevice, IRQ_STATUS, &intreg);
    if (!intreg) {
        return IRQ_NONE;
    }

    /* Clear irq on card. */
    sis8300_register_write(sisdevice, IRQ_CLEAR, intreg);
    
    if (intreg & ((1 << DMA_READ_DONE) | (1 << DMA_WRITE_DONE))) {
        sisdevice->dma_irq_flag = SIS8300_IRQ_HANDLED;
        wake_up_interruptible(&sisdevice->dma_irq_wait);
    }
    
    if (intreg & (1 << DAQ_IRQ)) {
        sisdevice->daq_irq_flag = SIS8300_IRQ_HANDLED;
        wake_up_all(&sisdevice->daq_irq_wait);
    }

    if (intreg & (1 << USER_IRQ)) {
        sisdevice->usr_irq_flag = SIS8300_IRQ_HANDLED;
        wake_up_all(&sisdevice->usr_irq_wait);
    }

    return IRQ_HANDLED;
}
