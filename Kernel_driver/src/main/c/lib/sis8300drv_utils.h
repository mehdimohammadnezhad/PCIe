/**
 * @file sis8300drv_utils.h
 * @brief Header for device setup functions and constants
 */


#ifndef SIS8300DRV_UTILS_H_
#define SIS8300DRV_UTILS_H_

#include <pthread.h>
#include "sis8300drv_list.h"


/* Channel and sampling constants. */
#define SIS8300DRV_NUM_ADCS             5       /**< Number of ADCs on the device. */
#define SIS8300DRV_SAMPLE_BYTES         2       /**< Length of a sample in bytes. */
#define SIS8300DRV_BLOCK_BYTES          32      /**< Length of a device memory block in bytes, 16 x 16 bits = 32 bytes. */
#define SIS8300DRV_CH_SETUP_FIRST       0x100   /**< Register address of the "Trigger setup register" for the first channel. */
#define SIS8300DRV_CH_THRESHOLD_FIRST   0x110   /**< Register address of the "Trigger threshold register" for the first channel. */
#define SIS8300DRV_CH_ADDRESS_FIRST     0x120   /**< Register address of the "Memory sample start register" for the first channel. */


/* Acquisition control constants. */
#define SIS8300DRV_RESET_ACQ            0x4     /**< Reset sampling logic - "Acquisition control register". */
#define SIS8300DRV_ACQ_ACTIVE           0x3     /**< Check if acquisition has finished - "Acquisition control register". */


/* DAC control constants */
#define SIS8300DRV_DAC_POWER            0x20    /**< Power on DACs. */
#define SIS8300DRV_DAC_TORB             0x10    /**< Two's complement format for DAC input. */
#define SIS8300DRV_DAC_RESET            0x100   /**< Reset DACs. */


/* Clock source constants. */
#define SIS8300DRV_CLKSRC_INTERNAL      0xF     /**< Set internal clock - "Clock distribution mux register". */
#define SIS8300DRV_CLKSRC_RTM2          0x0     /**< Set external clock from RTM - "Clock distribution mux register". */
#define SIS8300DRV_CLKSRC_SMA           0xF0F   /**< Set external clock from frontpanel SMA connector - "Clock distribution mux register". */
#define SIS8300DRV_CLKSRC_HAR           0xA0F   /**< Set external clock from frontpanel Harlink conncetor - "Clock distribution mux register". */
#define SIS8300DRV_CLKSRC_BPA           0xA     /**< Set external clock from first backpanel line - "Clock distribution mux register". */
#define SIS8300DRV_CLKSRC_BPB           0x5     /**< Set external clock from second backpanel line - "Clock distribution mux register". */
#define SIS8300DRV_CLKSRC_AD9510_MUX    0x0     /**< Clock source for the AD9510 chip - from device muxes. */
#define SIS8300DRV_CLKSRC_AD9510_RTM    0x1     /**< Clock source for the AD9510 chip - from RTM clk1 and RTMS clk2*/


/* Clock divisor constants. */
#define SIS8300DRV_CLKDIV_NOBYPASS      0x0     /**< Enable clock divider - "sis8300_ad9510_spi_setup". */
#define SIS8300DRV_CLKDIV_BYPASS        0x8000  /**< Disable clock divider - "sis8300_ad9510_spi_setup". */


/* Trigger related constants. */
#define SIS8300DRV_TRGSRC_SOFT          0x0     /**< Disable external and internal trigger - "Sample control register". */
#define SIS8300DRV_TRGSRC_INT           0x400   /**< Enable internal trigger (on one of the adc channels) - "Sample control register". */
#define SIS8300DRV_TRGSRC_EXT           0x800   /**< Enable external trigger - "Sample control register". */
#define SIS8300DRV_TRG_START            0x1     /**< Start sampling (only when external and internal trigger are disabled) - "Acquisition control register".  */
#define SIS8300DRV_TRG_ARM              0x2     /**< Arm device (wait for external or internal trigger) - "Acquisition control register". */


/* Write to device memory related constants. */
#define SIS8300DRV_DDR2_TEST_DISABLE    0x0     /**< Disable writing to device memory over PCIE. */
#define SIS8300DRV_DDR2_TEST_ENABLE     0x1     /**< Enable writing to device memory over PCIE. */
#define SIS8300DRV_WRITE_CHUNK_SIZE     64      /**< Maximum chunk of data to write in one dma transfer. */


/**
 * @brief Device private struct.
 *
 * Contains objects necessary for device access serialization,
 * end of acquisition signaling, device bookkeeping and status information.
 */
typedef struct t_sis8300drv_dev {
    struct list_head    list;           /**< Entry in the list of opened devices, sis8300drv_devlist. */
    char                *file;          /**< Name of the device file node in /dev */
    int                 handle;         /**< Device file descriptor. */
    int                 count;          /**< Number of times this device has been opened. */
    int                 armed;          /**< Is device armed, 0 - not armed, 1 - armed. */
    unsigned            poll_period;    /**< Poll period in milliseconds used when checking weather acquisition is finished. */
    unsigned            type;           /**< Device type, SIS8300 or SIS8300L. */
    unsigned            mem_size;       /**< Size of device - 0.5gb, 1gb or 2gb. */
    unsigned            adc_mem_size;   /**< Size of device memory dedicated to adc data. */
    pthread_mutex_t     lock;           /**< Lock that serializes access to the device. */
} sis8300drv_dev;


/* Device initialization, cleanup and configuration helper functions. */
void sis8300drv_free(sis8300drv_dev *sisdevice);

int sis8300drv_conf_ch(sis8300drv_dev *sisdevice, 
        unsigned nsamples, 
        unsigned channel_mask, 
        unsigned mem_size);

/* Device register access helper functions. */
int sis8300_reg_read(int handle, 
        uint32_t adr, 
        uint32_t *data);

int sis8300_reg_write(int handle, 
        uint32_t adr, 
        uint32_t data);

/* Read and write to ram helper functions. */
int sis8300drv_read_ram_unlocked(sis8300drv_dev *sisdevice, 
        unsigned offset, 
        unsigned size, 
        void *data);

int sis8300drv_write_ram_unlocked(sis8300drv_dev *sisdevice, 
        unsigned offset, 
        unsigned size, 
        void *data);

/* ADC setup helper functions. */
int sis8300drv_adc_spi_setup(sis8300drv_dev *sisdevice, 
        unsigned adc);


#endif /* SIS8300DRV_UTILS_H_ */
