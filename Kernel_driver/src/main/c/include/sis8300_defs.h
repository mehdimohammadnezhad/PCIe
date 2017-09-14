#ifndef SIS8300_DEFS_H_
#define SIS8300_DEFS_H_


/* Ioctl structs which will be passed as parameters. */

/* Generic register access. */
typedef struct t_sis8300_reg {
    uint32_t offset;    /**< Offset from pci bar0. */
    uint32_t data;      /**< Data which will be read/written. */
} sis8300_reg;

/* Irq with timeout. */
typedef struct t_sis8300_irq {
    uint32_t timeout;   /**< Timeout in ms when waiting for irq. */
    uint32_t type;      /**< Type of irq to wait for: 0 - daq irq, 1 - user irq. */
    uint32_t status;    /**< Status: 0 - ok, 1 - timeout, 2 - error. */
} sis8300_irq;


/* Irq flag values. */
#define SIS8300_IRQ_NONE            0
#define SIS8300_IRQ_HANDLED         1
#define SIS8300_IRQ_RELEASED        2

/* Irq types. */
#define SIS8300_DAQ_IRQ             0
#define SIS8300_USR_IRQ             1

/* Irq status return values. */
#define SIS8300_IRQ_SUCCESS         0
#define SIS8300_IRQ_RELEASE         1
#define SIS8300_IRQ_TIMEOUT         2
#define SIS8300_IRQ_ERROR           3

/* Calls need to pass an unique identifier as a command to ioctl. */
#define SIS8300_MAGIC 's'

/* Ioctl commands. */
#define SIS8300_REG_READ            _IOWR(SIS8300_MAGIC, 0, sis8300_reg)
#define SIS8300_REG_WRITE           _IOWR(SIS8300_MAGIC, 1, sis8300_reg)
#define SIS8300_WAIT_IRQ            _IOWR(SIS8300_MAGIC, 2, sis8300_irq)
#define SIS8300_RELEASE_IRQ         _IOWR(SIS8300_MAGIC, 3, sis8300_irq)


#endif /* SIS8300_DEFS_H_ */
