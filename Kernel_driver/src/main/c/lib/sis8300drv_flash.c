/**
 * Struck 8300 Linux userspace library.
 * Copyright (C) 2016 - 2017 European Spallation Source ERIC
 * Copyright (C) 2015 Cosylab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * @file sis8300drv_flash.c
 * @brief Implementation of flash chip programming utilities
 * @author kstrnisa
 */


#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include "sis8300_reg.h"
#include "sis8300drv.h"
#include "sis8300drv_utils.h"
#include "sis8300drv_flash.h"


/******************************************************************************/
/*                              PUBLIC FUNCTIONS                              */
/******************************************************************************/


/**
 * @brief Read firmware image from device flash.
 * @param [in] sisuser User context struct.
 * @param [in] size Size of the data to read from flash.
 * @param [out] data User buffer to read into.
 * @param [in] callback Pointer to function that will be called before
 * each page (#SIS8300DRV_FLASH_PAGESIZE bytes) of data is read from flash. Can be NULL.
 *
 * @retval status_no_device Device not opened.
 * @retval status_incompatible Operation not supported on this device.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_device_access Can't access device registers.
 * @retval status_spi_busy SPI bus busy.
 *
 * Reads the contents of device flash into the provided buffer 
 * in #SIS8300DRV_FLASH_PAGESIZE chunks. Before reading each chunk the provided
 * callback function (if not NULL) is called with the offset of the page 
 * about to be read as the argument.
 */
int sis8300drv_read_fw_image(sis8300drv_usr *sisuser, unsigned size, void *data, void (*callback)(unsigned offset)) {
    int             status;
    unsigned        completed, page_size;
    uint8_t         *image;
    sis8300drv_dev  *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    if (sisdevice->type != SIS8300_SIS8300L) {
        return status_incompatible;
    }
    
    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }
    
    completed = 0;
    page_size = SIS8300DRV_FLASH_PAGESIZE;
    image = (uint8_t *)data;

    while (completed < size) {
        if (callback) {
            (callback)(completed);
        }
    
        if (size - completed < SIS8300DRV_FLASH_PAGESIZE) {
            page_size = size - completed;
        }

        status = sis8300drv_flash_read_block(sisdevice, 
                completed, page_size, image + completed);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }
        
        completed += page_size;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);
    
    return status_success;
}


/**
 * @brief Write firmware image to device flash.
 * @param [in] sisuser User context struct.
 * @param [in] size Size of the data to write to flash.
 * @param [in] data User buffer to write.
 * @param [in] callback Pointer to function that will be called before
 * each page (#SIS8300DRV_FLASH_PAGESIZE bytes) of data is written to flash. Can be NULL.
 *
 * @retval status_no_device Device not opened.
 * @retval status_incompatible Operation not supported on this device.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_device_access Can't access device registers.
 * @retval status_spi_busy SPI bus busy.
 * @retval status_flash_busy Flash chip busy.
 * @retval status_flash_failed Data verification failed.
 *
 * Writes the contents of the provided buffer into device flash
 * in #SIS8300DRV_FLASH_PAGESIZE chunks. Before writing each chunk the provided
 * callback function (if not NULL) is called with the offset of the page 
 * about to be read as the argument. After each chunk is written a read at the 
 * same offset is performed and the contents are verified against what was written.
 */
int sis8300drv_write_fw_image(sis8300drv_usr *sisuser, unsigned size, void *data, void (*callback)(unsigned offset)) {
    int             status;
    unsigned        completed, page_size;
    uint8_t         *image, *image_check;
    sis8300drv_dev  *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    if (sisdevice->type != SIS8300_SIS8300L) {
        return status_incompatible;
    }
    
    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }
    
    completed = 0;
    page_size = SIS8300DRV_FLASH_PAGESIZE;
    image = (uint8_t *)data;
    
    image_check = (uint8_t *)malloc(SIS8300DRV_FLASH_PAGESIZE);
    if (!image_check) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_internal;
    }

    while (completed < size) {
        if (callback) {
            (callback)(completed);
        }
        
        /* Erase the block information whenever the address is alligned 
         * to the start of a block. */
        if (!(completed & (SIS8300DRV_FLASH_BLOCKSIZE - 1))) {
            status = sis8300drv_flash_erase_block(sisdevice, completed);
            if (status) {
                free(image_check);
                pthread_mutex_unlock(&sisdevice->lock);
                return status;
            }
        }
    
        if (size - completed < SIS8300DRV_FLASH_PAGESIZE) {
            page_size = size - completed;
        }
        
        status = sis8300drv_flash_write_page(sisdevice, 
                completed, page_size, image + completed);
        if (status) {
            free(image_check);
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }

        status = sis8300drv_flash_read_block(sisdevice, 
                completed, page_size, image_check);
        if (status) {
            free(image_check);
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }

        if (memcmp(image + completed, image_check, page_size)) {
            free(image_check);
            pthread_mutex_unlock(&sisdevice->lock);
            return status_flash_failed;
        }
        
        completed += page_size;
    }
    
    free(image_check);
    pthread_mutex_unlock(&sisdevice->lock);
    
    return status_success;
}


/******************************************************************************/
/*                              PRIVATE FUNCTIONS                             */
/******************************************************************************/


/**
 * @brief Take control of flash chip SPI bus from MMC.
 * @param [in] sisdevice Device context struct.
 *
 * @retval status_success SPI bus control taken successfully.
 * @retval status_device_access Can't access device registers.
 */
int sis8300drv_flash_take(sis8300drv_dev *sisdevice) {
    int status;
    
    /* Take control of spi bus. */
    status = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
            FLASH_SPI_MUX_EN);
    if (status) {
        return status_device_access;
    }
    
    /* Select flash. */
    status = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
            FLASH_SPI_CS);
    if (status) {
        sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                FLASH_SPI_MUX_EN FLASH_SPI_UNWIND);
        return status_device_access;
    }
    
    return status_success;
}


/**
 * @brief Release control of flash chip SPI bus back to the MMC.
 * @param [in] sisdevice Device context struct.
 *
 * @retval status_success SPI bus control released successfully.
 * @retval status_device_access Can't access device registers.
 */
int sis8300drv_flash_release(sis8300drv_dev *sisdevice) {
    int status, status_tmp;
    
    status = status_success;
    
    /* Return flash to mmc. */
    status_tmp = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
            FLASH_SPI_CS FLASH_SPI_UNWIND);
    if (status_tmp) {
        status = status_device_access;
    }

    /* Let go of spi bus. */
    status_tmp = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
            FLASH_SPI_MUX_EN FLASH_SPI_UNWIND);
    if (status_tmp) {
        status = status_device_access;
    }

    return status;
}


/**
 * @brief Poll the flash chip SPI bus until it is not busy anymore.
 * @param [in] sisdevice Device context struct.
 * @param [in] timeout Timeout in milliseconds.
 *
 * @retval status_success SPI bus not busy.
 * @retval status_spi_busy Timeout expired and SPI bus was still busy.
 * @retval status_device_access Can't access device registers.
 */
int sis8300drv_flash_spi_busy(sis8300drv_dev *sisdevice, unsigned timeout) {
    int         status;
    unsigned    iter;
    uint32_t    ui32_reg_val;
    
    iter = timeout * 1000;
    do {
        status = sis8300_reg_read(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                &ui32_reg_val);
        if (status) {
            return status_device_access;
        }
        
        usleep(1);
        iter--;
    } while (iter && (ui32_reg_val & FLASH_SPI_BUSY));
    
    if (!iter) {
        status = status_spi_busy;
    }
    
    return status_success;
}


/**
 * @brief Write a byte to the flash chip SPI bus and read back the response.
 * @param [in] sisdevice Device context struct.
 * @param [in] in Byte to write.
 * @param [out] out Byte that was read back. Can be NULL.
 *
 * @retval status_success Data written and read back successfully.
 * @retval status_spi_busy Timeout expired while waiting for SPI bus to become
 * not busy.
 * @retval status_device_access Can't access device registers.
 *
 * Writes the least significant byte to the the SPI bus, waits for the SPI bus 
 * to become not busy (with a timeout of one millisecond) and 
 * (if the provided argument is not NULL) reads the SPI response into 
 * the least significant byte of the provided argument.
 */
int sis8300drv_flash_byte_exch(sis8300drv_dev *sisdevice, unsigned in, unsigned *out) {
    int         status;
    uint32_t    ui32_reg_val;
    
    status = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
            FLASH_SPI_EXCH | (in & 0xFF));
    if (status) {
        return status_device_access;
    }
    
    status = sis8300drv_flash_spi_busy(sisdevice, 1);
    if (status) {
        return status;
    }

    if (out) {
        status = sis8300_reg_read(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                &ui32_reg_val);
        if (status) {
            return status_device_access;
        }
        *out = (unsigned)(ui32_reg_val & 0xFF);
    }

    return status_success;
}


/**
 * @brief Write a command byte and three bytes of offset to the flash chip SPI bus.
 * @param [in] sisdevice Device context struct.
 * @param [in] command Byte to write first.
 * @param [in] offset Offset to write.
 *
 * @retval status_success Command successful.
 * @retval status_spi_busy Timeout expired while waiting for SPI bus to become
 * not busy.
 * @retval status_device_access Can't access device registers.
 *
 * Writes the least significant byte of command to the the SPI bus and
 * then writes the three least significant bytes of offset to the SPI bus.
 */
int sis8300drv_flash_offset(sis8300drv_dev *sisdevice, unsigned command, unsigned offset) {
    int status, iter;
    
    status = sis8300drv_flash_byte_exch(sisdevice, command, NULL);
    if (status) {
        return status;
    }
    
    for (iter = 2; iter >= 0; iter--) {
        status = sis8300drv_flash_byte_exch(sisdevice, offset >> (8 * iter), NULL);
        if (status) {
            return status;
        }
    }
    
    return status_success;
}


/**
 * @brief Enable writes to the flash chip
 * @param [in] sisdevice Device context struct.
 *
 * @retval status_success Command successful.
 * @retval status_spi_busy Timeout expired while waiting for SPI bus to become
 * not busy.
 * @retval status_device_access Can't access device registers.
 *
 * Takes control of the flash chip SPI bus, send the "ennable write" command
 * and release control. This function has to be called before every interaction
 * (every command) with the flash chip that involves writing.
 */
int sis8300drv_flash_write_enable(sis8300drv_dev *sisdevice) {
    int status;
    
    status = sis8300drv_flash_take(sisdevice);
    if (status) {
        return status;
    }
    
    /* Enable write command. */
    status = sis8300drv_flash_byte_exch(sisdevice, FLASH_SPI_WRITE_ENABLE, NULL);
    if (status) {
        sis8300drv_flash_release(sisdevice);
        return status;
    }
    
    return sis8300drv_flash_release(sisdevice);
}


/**
 * @brief Poll the flash chip itself until it is not busy anymore.
 * @param [in] sisdevice Device context struct.
 * @param [in] timeout Timeout in milliseconds.
 *
 * @retval status_success Flash chip bus not busy.
 * @retval status_spi_busy Timeout expired and SPI bus was still busy.
 * @retval status_flash_busy Timeout expired and flash chip was still busy.
 * @retval status_device_access Can't access device registers.
 */
int sis8300drv_flash_busy(sis8300drv_dev *sisdevice, unsigned timeout) {
    int         status;
    unsigned    flash_status, iter, poll_period;
    
    if (timeout > 100) {
        /* Poll with 100ms interval. */
        poll_period = 100;
        iter = (timeout + poll_period) / poll_period;
    } else {
        /* Poll with 1ms interval. */
        poll_period = 1;
        iter = timeout;
    }
    /* Convert to us. */
    poll_period *= 1000;
    
    do {
        status = sis8300drv_flash_take(sisdevice);
        if (status) {
            return status;
        }
    
        /* Read status command. */
        status = sis8300drv_flash_byte_exch(sisdevice, FLASH_SPI_READ_STATUS, NULL);
        if (status) {
            sis8300drv_flash_release(sisdevice);
            return status;
        }
        
        /* Read status. */
        status = sis8300drv_flash_byte_exch(sisdevice, 0, &flash_status);
        if (status) {
            sis8300drv_flash_release(sisdevice);
            return status;
        }
        
        status = sis8300drv_flash_release(sisdevice);
        if (status) {
            return status;
        }
        
        usleep(poll_period);
        iter--;
    } while (iter && (flash_status & FLASH_SPI_FLASH_BUSY));
    
    if (!iter) {
        status = status_flash_busy;
    }
    
    return status;
}


/**
 * @brief Read read data from device flash.
 * @param [in] sisdevice Device context struct.
 * @param [in] offset Offset to read from.
 * @param [in] size Size of the data to read from flash.
 * @param [out] data User buffer to read into.
 *
 * @retval status_success Data read back successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_spi_busy SPI bus busy.
 */
int sis8300drv_flash_read_block(sis8300drv_dev *sisdevice, unsigned offset, unsigned size, void *data) {
    int         status, iter;
    uint32_t    ui32_reg_val;
    
    status = sis8300drv_flash_take(sisdevice);
    if (status) {
        return status;
    }
    
    /* Read data command. */
    status = sis8300drv_flash_offset(sisdevice, FLASH_SPI_READ_DATA, offset);
    if (status) {
        sis8300drv_flash_release(sisdevice);
        return status;
    }
    
    /* Select block mode. */
    status = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
            FLASH_SPI_RD_BLK_EN);
    if (status) {
        sis8300drv_flash_release(sisdevice);
        return status_device_access;
    }
    
    usleep(1);

    /* Read data from fifo. */
    for (iter = 0; iter < size; iter++) {
        /* Read byte. */
        status = sis8300_reg_read(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                &ui32_reg_val);
        if (status) {
            sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                    FLASH_SPI_RD_BLK_EN FLASH_SPI_UNWIND);
            sis8300drv_flash_release(sisdevice);
            return status_device_access;
        }
        
        ((uint8_t *)data)[iter] = (uint8_t)ui32_reg_val;
        
        /* Advance fifo. */
        status = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                FLASH_SPI_RD_BLK_FIFO);
        if (status) {
            sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                    FLASH_SPI_RD_BLK_EN FLASH_SPI_UNWIND);
            sis8300drv_flash_release(sisdevice);
            return status_device_access;
        }
    }
    
    /* Deselect block mode. */
    status = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
            FLASH_SPI_RD_BLK_EN FLASH_SPI_UNWIND);
    if (status) {
        sis8300drv_flash_release(sisdevice);
        return status;
    }

    return sis8300drv_flash_release(sisdevice);
}


/**
 * @brief Erase a block on device flash.
 * @param [in] sisdevice Device context struct.
 * @param [in] offset Offset to read from.
 *
 * @retval status_success Block erased successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_spi_busy SPI bus busy.
 * @retval status_flash_busy Timeout expired and flash chip was still busy.
 */
int sis8300drv_flash_erase_block(sis8300drv_dev *sisdevice, unsigned offset) {
    int status;
    
    status = sis8300drv_flash_write_enable(sisdevice);
    if (status) {
        return status;
    }
    
    status = sis8300drv_flash_take(sisdevice);
    if (status) {
        return status;
    }
    
    /* Erase block command. */
    status = sis8300drv_flash_offset(sisdevice, FLASH_SPI_BLOCK_ERASE, offset);
    if (status) {
        sis8300drv_flash_release(sisdevice);
        return status;
    }
    
    status = sis8300drv_flash_release(sisdevice);
    if (status) {
        return status;
    }
    
    /* poll busy, for a maximum of 5 seconds
     * - L/L2 datasheet: blockerase max: 1000ms
     * - KU   datasheet: blockerase max: 2600ms
     */
    return sis8300drv_flash_busy(sisdevice, 5000);
}


/**
 * @brief Write a page to device flash.
 * @param [in] sisdevice Device context struct.
 * @param [in] offset Offset to write to.
 * @param [in] size Size of the data to write.
 * @param [in] data User buffer to write.
 *
 * @retval status_success Block erased successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_spi_busy SPI bus busy.
 * @retval status_flash_busy Timeout expired and flash chip was still busy.
 */
int sis8300drv_flash_write_page(sis8300drv_dev *sisdevice, unsigned offset, unsigned size, void *data) {
    int         status, iter;
    uint32_t    ui32_reg_val;
    
    status = sis8300drv_flash_write_enable(sisdevice);
    if (status) {
        return status;
    }
    
    status = sis8300drv_flash_take(sisdevice);
    if (status) {
        return status;
    }
    
    /* Program page command. */
    status = sis8300drv_flash_offset(sisdevice, FLASH_SPI_PROGRAM_PAGE, offset);
    if (status) {
        sis8300drv_flash_release(sisdevice);
        return status;
    }
    
    /* Select block mode. */
    status = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
            FLASH_SPI_WR_BLK_EN);
    if (status) {
        sis8300drv_flash_release(sisdevice);
        return status;
    }
        
    /* Pipe data in. */
    for (iter = 0; iter < size; iter++) {      
        ui32_reg_val = (uint32_t)((uint8_t *)data)[iter];     
        status = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                FLASH_SPI_WR_BLK_FILL | ui32_reg_val);
        if (status) {
            sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                    FLASH_SPI_WR_BLK_EN FLASH_SPI_UNWIND);
            sis8300drv_flash_release(sisdevice);
            return status_device_access;
        }
    }
    
    status = sis8300drv_flash_spi_busy(sisdevice, 1);
    if (status) {
        sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
                FLASH_SPI_WR_BLK_EN FLASH_SPI_UNWIND);
        sis8300drv_flash_release(sisdevice);
        return status;
    }
    
    /* Deselect block mode. */
    status = sis8300_reg_write(sisdevice->handle, SIS8300_FLASH_SPI_REG, 
            FLASH_SPI_WR_BLK_EN FLASH_SPI_UNWIND);
    if (status) {
        sis8300drv_flash_release(sisdevice);
        return status_device_access;
    }
    
    status = sis8300drv_flash_release(sisdevice);
    if (status) {
        return status;
    }
    
    /* datasheet: page program max: 3ms */
    return sis8300drv_flash_busy(sisdevice, 5);
}


