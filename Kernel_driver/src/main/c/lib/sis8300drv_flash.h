/**
 * @file sis8300drv_flash.h
 * @brief Header for constants/functions related to programming the flash chip
 * over SPI.
 */


#ifndef SIS8300DRV_FLASH_H_
#define SIS8300DRV_FLASH_H_


#define SIS8300DRV_FLASH_PAGESIZE   256     /**< 256B */
#define SIS8300DRV_FLASH_BLOCKSIZE  65536   /**< 64kB */


#include "sis8300drv_utils.h"


int sis8300drv_flash_take(sis8300drv_dev *sisdevice);

int sis8300drv_flash_release(sis8300drv_dev *sisdevice);

int sis8300drv_flash_spi_busy(sis8300drv_dev *sisdevice, 
        unsigned timeout);
        
int sis8300drv_flash_byte_exch(sis8300drv_dev *sisdevice, 
        unsigned in, 
        unsigned *out);
        
int sis8300drv_flash_offset(sis8300drv_dev *sisdevice, 
        unsigned command, 
        unsigned offset);

int sis8300drv_flash_write_enable(sis8300drv_dev *sisdevice);

int sis8300drv_flash_busy(sis8300drv_dev *sisdevice, 
        unsigned timeout);

int sis8300drv_flash_read_block(sis8300drv_dev *sisdevice, 
        unsigned offset, 
        unsigned size, 
        void *data);

int sis8300drv_flash_erase_block(sis8300drv_dev *sisdevice, 
        unsigned offset);

int sis8300drv_flash_write_page(sis8300drv_dev *sisdevice, 
        unsigned offset, 
        unsigned size, 
        void *data);


#endif /* SIS8300DRV_FLASH_H_ */
