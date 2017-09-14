/**
 * Struck 8300 Linux userspace library.
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
 * @file sis8300drv_utils.c
 * @brief Implementation of device helper functions
 * @author kstrnisa
 */


#include <stdlib.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "sis8300_reg.h"
#include "sis8300_defs.h"
#include "sis8300drv.h"
#include "sis8300drv_utils.h"


#define SIS8300DRV_ADC_SPI_WRITE(device, value)                                     \
    do {                                                                            \
    status = sis8300_reg_write((device)->handle, SIS8300_ADC_SPI_REG, (value));     \
    if (status) {                                                                   \
        return status_device_access;                                                \
    }                                                                               \
    usleep(1);                                                                      \
    } while (0);


/******************************************************************************/
/*                              PRIVATE FUNCTIONS                             */
/******************************************************************************/


/**
 * @brief Free resources related to the device context.
 * @param [in] sisdevice device context struct.
 *
 * Deallocates the device context struct and members.
 *
 * This function is private and must never be called by client code. It assumes
 * that #sis8300drv_devlist_lock is held by the caller and that there is no one
 * left using the device (the reference count is zero).
 */
void sis8300drv_free(sis8300drv_dev *sisdevice) {
    close(sisdevice->handle);
    pthread_mutex_destroy(&sisdevice->lock);
    free(sisdevice->file);
    free(sisdevice);
}


/**
 * @brief Set analog channel configuration.
 * @param [in] sisuser User context struct.
 * @param [in] nsamples Number of samples for each analog channel.
 * @param [in] channel_mask Mask describing which channels are enabled for
 * data acquisition.
 *
 * This function configures the memory layout of the device memory. The layout
 * is such that the memory areas for each enabled channel follow each other with
 * no empty space between them, starting with the first enabled channel.
 *
 * This function also sets the correct enable/disable bits in device registers
 * according to the provided channel mask.
 *
 * This function assumes that the device lock is held by the caller and that 
 * the device is not armed.
 */
int sis8300drv_conf_ch(sis8300drv_dev *sisdevice, unsigned nsamples, unsigned channel_mask, unsigned mem_size) {
    int             status;
    unsigned        nchannels, nbytes, nblocks, channel;
    uint32_t        ui32_reg_val;

    /* Calculate if the selected channels will fit into card memory
     * with selected number of samples. */
    nbytes = nsamples * SIS8300DRV_SAMPLE_BYTES;
    nblocks = nsamples / SIS8300DRV_BLOCK_SAMPLES;
    nchannels = 0;
    for (channel = 0; channel < SIS8300DRV_NUM_AI_CHANNELS; channel++) {
        nchannels += (channel_mask & (1 << channel)) >> channel;
    }
    if (nbytes && (nchannels > mem_size/nbytes)) {
        return status_argument_range;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_SAMPLE_CONTROL_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    /* Preserve trigger configuration. */
    ui32_reg_val &= (uint32_t)SAMPLE_CONTROL_TRIGGER;
    ui32_reg_val |= SAMPLE_CONTROL_CH_DIS & (uint32_t)~channel_mask;

    status = sis8300_reg_write(sisdevice->handle,
            SIS8300_SAMPLE_CONTROL_REG, ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    /* Set onboard memory addresses for enabled channels. */
    nchannels = 0;
    for (channel = 0; channel < SIS8300DRV_NUM_AI_CHANNELS; channel++) {
        if (channel_mask & (1 << channel)) {
            ui32_reg_val = (uint32_t)nchannels*nblocks;
            status = sis8300_reg_write(sisdevice->handle,
                    SIS8300DRV_CH_ADDRESS_FIRST + (uint32_t)channel, ui32_reg_val);
            if (status) {
                return status_device_access;
            }
            nchannels++;
        }
    }

    return status_success;
}


/**
 * @brief Read device register.
 * @param [in] handle Device file descriptor.
 * @param [in] address Address of register to read from.
 * @param [out] data Register value.
 */
int sis8300_reg_read(int handle, uint32_t address, uint32_t *data) {
    sis8300_reg    uint32_reg;
    int            ret;

    uint32_reg.offset = address;
    ret = ioctl(handle, SIS8300_REG_READ, &uint32_reg);
    *data = uint32_reg.data;

    return ret;
}


/**
 * @brief Write a value to a device register.
 * @param [in] handle Device file descriptor.
 * @param [in] address Address of register to write to.
 * @param [in] data Register value to write.
 */
int sis8300_reg_write(int handle, uint32_t address, uint32_t data) {
    sis8300_reg    uint32_reg;

    uint32_reg.offset = address;
    uint32_reg.data = data;
    return ioctl(handle, SIS8300_REG_WRITE, &uint32_reg);
}


/**
 * @brief Read contents of device memory.
 * @param [in] sisuser User context struct.
 * @param [in] offset Offset to read from in device memory.
 * @param [in] size Size of the buffer to read.
 * @param [in] data User buffer to read into.
 *
 * @retval status_success Data transfered successfully.
 * @retval status_argument_range Requested data (with offset taken into account)
 * exceeds device memory.
 * @retval status_argument_incalid Size or offset are not properly alligned.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_write Can't read data from device memory.
 *
 * Due to the limitation of the firmware data can be read from device memory 
 * only in multiples of #SIS8300DRV_BLOCK_BYTES and reads have to be alligned 
 * to #SIS8300DRV_BLOCK_BYTES offsets.
 * 
 * This function assumes that the device lock is held by the caller and that 
 * the device is not armed.
 */
int sis8300drv_read_ram_unlocked(sis8300drv_dev *sisdevice, unsigned offset, unsigned size, void *data) {
    int             status;

    if (offset + size > sisdevice->mem_size) {
        return status_argument_range;
    }
    
    if (size % SIS8300DRV_BLOCK_BYTES || offset % SIS8300DRV_BLOCK_BYTES) {
        return status_argument_invalid;
    }
     
    status = lseek(sisdevice->handle, offset, SEEK_SET);
    if (status < 0) {
        return status_device_access;
    }

    status = read(sisdevice->handle, data, size);
    if (status < 0) {
        return status_read;
    }

    return status_success;
}


/**
 * @brief Write user supplied data to device memory.
 * @param [in] sisuser User context struct.
 * @param [in] offset Offset to write the data to in device memory.
 * @param [in] size Size of the data to write.
 * @param [in] data User buffer containing the data to write.
 *
 * @retval status_success Data transfered successfully.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_argument_range Requested data (with offset taken into account)
 * exceeds device memory.
 * @retval status_argument_incalid Size or offset are not properly alligned.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_write Can't transfer data to device memory.
 *
 * Due to the limitation of the firmware data can be written to device memory only in
 * #SIS8300DRV_BLOCK_BYTES chunks and the writes have to be alligned to #SIS8300DRV_BLOCK_BYTES
 * offsets.
 * 
 * This function assumes that the device lock is held by the caller and that 
 * the device is not armed.
 */
int sis8300drv_write_ram_unlocked(sis8300drv_dev *sisdevice, unsigned offset, unsigned size, void *data) {
    int             status, chunk_size, chunk_offset;

    if (offset + size > sisdevice->mem_size) {
        return status_argument_range;
    }
    
    if (size % SIS8300DRV_BLOCK_BYTES || offset % SIS8300DRV_BLOCK_BYTES) {
        return status_argument_invalid;
    }

    /* Enable DDR2 test write interface. */
    status = sis8300_reg_write(sisdevice->handle,
            SIS8300_DDR2_ACCESS_CONTROL, SIS8300DRV_DDR2_TEST_ENABLE);
    if (status) {
        return status_device_access;
    }
    
    /* Allow 2 outstanding PCIe requests. */
    status = sis8300_reg_write(sisdevice->handle,
            SIS8300_PCIE_REQUEST_NUM, 2);
    if (status) {
        return status_device_access;
    }
    
    /* HACK: The board sometimes writes garbage values if the size of
     * the chunk written is too large. Max chunk size seems to be 64 bytes. */
    chunk_offset = 0;
    while (size) {
        
        chunk_size = size < SIS8300DRV_WRITE_CHUNK_SIZE ? size : SIS8300DRV_WRITE_CHUNK_SIZE;
        
        status = lseek(sisdevice->handle, offset + chunk_offset, SEEK_SET);
        if (status < 0) {
            return status_device_access;
        }
        
        status = write(sisdevice->handle, &((char *)data)[chunk_offset], chunk_size);
        if (status < 0) {
            return status_write;
        }
        
        size -= chunk_size;
        chunk_offset += chunk_size;
    }

    status = sis8300_reg_write(sisdevice->handle,
            SIS8300_DDR2_ACCESS_CONTROL, SIS8300DRV_DDR2_TEST_DISABLE);
    if (status) {
        return status_device_access;
    }
    
    status = sis8300_reg_write(sisdevice->handle,
            SIS8300_PCIE_REQUEST_NUM, 1);
    if (status) {
        return status_device_access;
    }

    return status_success;
}


/**
 * @brief Configure the parameters of the ADC AD9268 chip.
 * @param [in] handle Device file descriptor.
 * @param [in] adc ADC to configure (0 to 4).
 *
 * @retval status_success Initialization successful.
 * @retval status_argument_range Invalid ADC number.
 * @retval status_device_access Can't access device registers.
 * 
 * This function assumes that the device lock is held by the caller and that 
 * the device is not armed.
 */
int sis8300drv_adc_spi_setup(sis8300drv_dev *sisdevice, unsigned adc) {
    int             status;
    unsigned        uint_adc_mux_select, addr, data;
    uint32_t        ui32_reg_val;

    if (adc >= SIS8300DRV_NUM_ADCS) {
        return status_argument_range;
    }

    uint_adc_mux_select = adc << 24;

    /* output type LVDS */
    addr = (0x14 & 0xffff) << 8;
    data = (0x40 & 0xff);
    ui32_reg_val = uint_adc_mux_select + addr + data;
    SIS8300DRV_ADC_SPI_WRITE(sisdevice, ui32_reg_val);

    addr = (0x16 & 0xffff) << 8;
    data = (0x00 & 0xff);
    ui32_reg_val = uint_adc_mux_select + addr + data;
    SIS8300DRV_ADC_SPI_WRITE(sisdevice, ui32_reg_val);
    
    addr = (0x17 & 0xffff) << 8;
    data = (0x00 & 0xff);
    ui32_reg_val = uint_adc_mux_select + addr + data;
    SIS8300DRV_ADC_SPI_WRITE(sisdevice, ui32_reg_val);

    /* register update cmd */
    addr = (0xff & 0xffff) << 8;
    data = (0x01 & 0xff);
    ui32_reg_val = uint_adc_mux_select + addr + data;
    SIS8300DRV_ADC_SPI_WRITE(sisdevice, ui32_reg_val);

    return status_success;
}

