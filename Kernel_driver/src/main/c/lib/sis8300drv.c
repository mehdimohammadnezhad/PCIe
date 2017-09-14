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
 * @file sis8300drv.c
 * @brief Implementation of sis8300 digitizer userspace api.
 * @author kstrnisa
 */


#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <libudev.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "sis8300_reg.h"
#include "sis8300_defs.h"
#include "sis8300drv.h"
#include "sis8300drv_ad9510.h"
#include "sis8300drv_flash.h"
#include "sis8300drv_utils.h"


LIST_HEAD(sis8300drv_devlist);                                          /**< List of device private structs for already opened devices. */
pthread_mutex_t sis8300drv_devlist_lock = PTHREAD_MUTEX_INITIALIZER;    /**< Lock that serializes access to the list of already opened devices. */


/**
 * @brief Open device with a specified file node.
 * @param [in] sisuser User context struct.
 *
 * @retval status_success Device successfully opened.
 * @retval status_argument_invalid Supplied user context struct or file node invalid.
 * @retval status_no_device Can't open the device with the specified node.
 * @retval status_internal Can't allocate memory for device context struct.
 * @retval status_device_access Can't access device registers.
 *
 * Before calling this function the user must allocate memory for the user
 * context struct #sis8300drv_usr and fill the name of the device file node.
 * The pointer to the user context struct must be passed to every subsequent
 * device access call.
 *
 * If the device with the specified file node is already opened the function
 * searches the list for the device context. Otherwise the device
 * context is allocated and added to the list.
 */
int sis8300drv_open_device(sis8300drv_usr *sisuser) {
    int             status, handle;
    unsigned        fw_version, fw_options, device_type;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    if (!sisuser || !sisuser->file) {
        return status_argument_invalid;
    }

    /* Check if the device context struct already exists in the list.
     * If a matching device context is found (same device file node name)
     * the firmware version register is read to check if the device is 
     * physically present. */
    pthread_mutex_lock(&sis8300drv_devlist_lock);
    list_for_each_entry(sisdevice, &sis8300drv_devlist, list) {
        if (!strcmp(sisdevice->file, sisuser->file)) {
            /* If the device is not physically present might be that the board
             * was hotswaped and either another device context already exists
             * and if not we must still try to open the device. */
            status = sis8300_reg_read(sisdevice->handle, 
                    SIS8300_IDENTIFIER_VERSION_REG, &ui32_reg_val);
            if (status) {
                continue;
            }
            sisuser->device = sisdevice;
            sisdevice->count++;
            pthread_mutex_unlock(&sis8300drv_devlist_lock);
            return status_success;
        }
    }
    
    handle = open(sisuser->file, O_RDWR);
    if (handle < 0) {
        pthread_mutex_unlock(&sis8300drv_devlist_lock);
        return status_no_device;
    }

    sisdevice = calloc(1, sizeof(sis8300drv_dev));
    if (!sisdevice) {
        pthread_mutex_unlock(&sis8300drv_devlist_lock);
        return status_internal;
    }
    sisdevice->handle = handle;
    sisdevice->file = strdup(sisuser->file);
    sisdevice->armed = 0;
    sisdevice->count = 1;
    sisdevice->poll_period = 1;
    pthread_mutex_init(&sisdevice->lock, NULL);

    sisuser->device = sisdevice;
    
    /* Determine type of board and amount of onboard memory. */
    status = sis8300drv_get_fw_version(sisuser, &fw_version);
    if (status) {
        pthread_mutex_unlock(&sis8300drv_devlist_lock);
        sis8300drv_free(sisdevice);
        sisuser->device = NULL;
        return status_device_access;
    }

    status = sis8300drv_get_fw_options(sisuser, &fw_options);
    if (status) {
        pthread_mutex_unlock(&sis8300drv_devlist_lock);
        sis8300drv_free(sisdevice);
        sisuser->device = NULL;
        return status_device_access;
    }
    
    device_type = fw_version >> 16;
    switch (device_type) {
        case SIS8300_SIS8300:
            sisdevice->type = SIS8300_SIS8300;
            sisdevice->mem_size = (SIS8300_FPGA_SX_1GByte_Memory & fw_options) ?
                SIS8300_1GB_MEMORY : SIS8300_512MB_MEMORY;
            break;
        case SIS8300_SIS8300L:
        case SIS8300_SIS8300L2:
        case SIS8300_SIS8300KU:
            sisdevice->type = SIS8300_SIS8300L;
            sisdevice->mem_size = SIS8300_2GB_MEMORY;
            break;
        default:
            pthread_mutex_unlock(&sis8300drv_devlist_lock);
            sis8300drv_free(sisdevice);
            sisuser->device = NULL;
            return status_incompatible;
            break;
    }
    
    /* On a generic device all device memory is dedicated to adc data. */
    sisdevice->adc_mem_size = sisdevice->mem_size;

    list_add_tail(&sisdevice->list, &sis8300drv_devlist);

    pthread_mutex_unlock(&sis8300drv_devlist_lock);

    return status_success;
}


/**
 * @brief Close device corresponding to the specified user context struct.
 * @param [in] sisuser User context struct.
 *
 * @retval status_success Device successfully closed.
 * @retval status_no_device Device not opened.
 *
 * Closes the file descriptor associated with the user context struct. It also
 * decrements the reference counter of the device context struct corresponding
 * to the specified user context struct. if this is the last user of the device
 * then this also destroys the device context.
 * 
 * This function is not thread safe with respect to other functions in the
 * library except #sis8300drv_open_device.
 */
int sis8300drv_close_device(sis8300drv_usr *sisuser) {
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    /* Close device if this is the last user. */
    pthread_mutex_lock(&sis8300drv_devlist_lock);
    sisdevice->count--;
    if (sisdevice->count < 1) {
        list_del(&sisdevice->list);
        sis8300drv_free(sisdevice);
    }
    pthread_mutex_unlock(&sis8300drv_devlist_lock);

    sisuser->device = NULL;

    return status_success;
}


/**
 * @brief Initialize ADCs.
 * @param [in] sisuser User context struct.
 *
 * @retval status_success ADCs successfully initialized.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_init_adc(sis8300drv_usr *sisuser) {
    int             status, iter;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    for (iter = 0; iter < SIS8300DRV_NUM_ADCS; iter++) {
        status = sis8300drv_adc_spi_setup(sisdevice, iter);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_access;
        }
    }
    
    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Initialize DACs.
 * @param [in] sisuser User context struct.
 *
 * @retval status_success DACs successfully initialized.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_no_device Device not opened.
 *
 * Powers on the two DACs on the boards and sets the input format to
 * two's complement. The clock for the DACs is set to the FPGA clock and
 * the input for the DACs is set to come from the DAC data register.
 */
int sis8300drv_init_dac(sis8300drv_usr *sisuser) {
    int             status;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    /* Power on DACs and use two's complement format for input. */
    status = sis8300_reg_write(sisdevice->handle,
            SIS8300_DAC_CONTROL_REG,
            SIS8300DRV_DAC_POWER | SIS8300DRV_DAC_TORB | SIS8300DRV_DAC_RESET);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Perform a reset of the board FPGA.
 * @param [in] sisuser User context struct.
 *
 * @retval status_success Master reset performed.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_master_reset(sis8300drv_usr *sisuser) {
    int             status;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300_reg_write(sisdevice->handle, SIS8300_MASTER_RESET_REG, 0x1);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Get the mask describing currentlly enabled channels.
 * @param [in] sisuser User context struct.
 * @param [out] channel_mask Bit mask describing enabled/disabled channels.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * The channel mask is a bit mask where the least significant ten bits
 * describe whether respective channels are enabled (bit high) or
 * disabled (bit low).
 */
int sis8300drv_get_channel_mask(sis8300drv_usr *sisuser, unsigned *channel_mask) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_SAMPLE_CONTROL_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *channel_mask = (SAMPLE_CONTROL_CH_DIS & (unsigned)~ui32_reg_val);

    return status_success;
}


/**
 * @brief Get the mask describing currentlly enabled channels.
 * @param [in] sisuser User context struct.
 * @param [in] channel_mask Bit mask describing which channels to
 * enable/disable.
 *
 * @retval status_success Parameter set successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_argument_range Enabling desired channel would not
 * fit into device memory.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_no_device Device not opened.
 *
 * The channel mask is a bit mask where the least significant ten bits
 * describe whether respective channels are enabled (bit high) or
 * disabled (bit low).
 *
 * This function also appropriately configures memory locations for
 * channel data in device memory with respect to currently selected
 * number of samples.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_set_channel_mask(sis8300drv_usr *sisuser, unsigned channel_mask) {
    int             status;
    unsigned        nsamples;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300drv_get_nsamples(sisuser, &nsamples);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* Adjust channel locations in onboard memory. */
    status = sis8300drv_conf_ch(sisdevice, nsamples, channel_mask, sisdevice->adc_mem_size);

    pthread_mutex_unlock(&sisdevice->lock);

    return status;
}



/**
 * @brief Get the current number of samples for each analog channel.
 * @param [in] sisuser User context struct.
 * @param [out] nsamples Number of samples for each analog channel.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_get_nsamples(sis8300drv_usr *sisuser, unsigned *nsamples) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_SAMPLE_LENGTH_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    *nsamples = (unsigned)ui32_reg_val * SIS8300DRV_BLOCK_SAMPLES;

    /* This is a hack working around the fact that the board always
     * acquires one extra block of samples. */
    *nsamples += SIS8300DRV_BLOCK_SAMPLES;

    return status_success;
}


/**
 * @brief Set the number of samples for each analog channel.
 * @param [in] sisuser User context struct.
 * @param [in] nsamples Number of samples for each analog channel.
 *
 * @retval status_success Parameter set successfully.
 * @retval status_argument_invalid The desired number of samples is not a
 * multiple of #SIS8300DRV_BLOCK_SAMPLES or is not a positive number.
 * @retval status_argument_range The number of samples desired exceeds the
 * memory available on the device.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * Number of samples must be a positive number, a multiple of
 * #SIS8300DRV_BLOCK_SAMPLES and must not exceed the memory available
 * on the device where the currently set channel mask is taken into account.
 */
int sis8300drv_set_nsamples(sis8300drv_usr *sisuser, unsigned nsamples) {
    int             status;
    unsigned        channel_mask;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    if (nsamples <= 0) {
        return status_argument_invalid;
    }

    if (nsamples % SIS8300DRV_BLOCK_SAMPLES) {
        return status_argument_invalid;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300drv_get_channel_mask(sisuser, &channel_mask);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* Adjust channel locations in onboard memory. */
    status = sis8300drv_conf_ch(sisdevice, nsamples, channel_mask, sisdevice->adc_mem_size);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* This is a hack working around the fact that the board always
     * acquires one extra block of samples. */
    if (nsamples) {
        nsamples -= SIS8300DRV_BLOCK_SAMPLES;
    }

    status = sis8300_reg_write(sisdevice->handle,
            SIS8300_SAMPLE_LENGTH_REG, (uint32_t)(nsamples / SIS8300DRV_BLOCK_SAMPLES));
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status;
}


/**
 * @brief Get the current number of pre-trigger for each analog channel.
 * @param [in] sisuser User context struct.
 * @param [out] npretrig Number of pre-trigger samples for each analog channel.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_get_npretrig(sis8300drv_usr *sisuser, unsigned *npretrig) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_PRETRIGGER_DELAY_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    *npretrig = (unsigned)ui32_reg_val;

    return status_success;
}


/**
 * @brief Set the number of pre-trigger samples for each analog channel.
 * @param [in] sisuser User context struct.
 * @param [in] npretrig Number of pre-trigger samples for each analog channel.
 *
 * @retval status_success Parameter set successfully.
 * @retval status_argument_range The number of samples desired exceeds
 * #SIS8300DRV_MAX_PRETRIG.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * Sets the number of pre-trigger samples for each analog channel.
 * This does not alter the total number of samples acquired at each acquisition,
 * it only specifies how many of those samples are to be acquired before the
 * actual trigger.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_set_npretrig(sis8300drv_usr *sisuser, unsigned npretrig) {
    int             status;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    if (npretrig > SIS8300DRV_MAX_PRETRIG) {
        return status_argument_range;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300_PRETRIGGER_DELAY_REG, (uint32_t)npretrig);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Get the clock source currently set on the device.
 * @param [in] sisuser User context struct.
 * @param [out] clk_src Current clock source.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_state The information retrieved from the device
 * does not correspond to a valid clock source.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_get_clock_source(sis8300drv_usr *sisuser, sis8300drv_clk_src *clk_src) {
    int             status;
    uint32_t        ui32_reg_val, clock_source_ad9510;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    /* Lock the device so there is no race condition during SPI communication. */
    pthread_mutex_lock(&sisdevice->lock);
    
    status = sis8300drv_ad9510_spi_get_source(sisdevice, &clock_source_ad9510);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);
    
    if (clock_source_ad9510 == SIS8300DRV_CLKSRC_AD9510_RTM) {
        *clk_src = clk_src_rtm01;
        return status_success;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_CLOCK_DISTRIBUTION_MUX_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    switch(ui32_reg_val) {
        case SIS8300DRV_CLKSRC_INTERNAL:
            *clk_src = clk_src_internal;
            break;
        case SIS8300DRV_CLKSRC_RTM2:
            *clk_src = clk_src_rtm2;
            break;
        case SIS8300DRV_CLKSRC_SMA:
            *clk_src = clk_src_sma;
            break;
        case SIS8300DRV_CLKSRC_HAR:
            *clk_src = clk_src_harlink;
            break;
        case SIS8300DRV_CLKSRC_BPA:
            *clk_src = clk_src_backplane_a;
            break;
        case SIS8300DRV_CLKSRC_BPB:
            *clk_src = clk_src_backplane_b;
            break;
        default:
            return status_device_state;
    }

    return status_success;
}


/**
 * @brief Set the clock source used for analog data acquisition.
 * @param [in] sisuser User context struct.
 * @param [in] clk_src Clock source to set.
 *
 * @retval status_success Parameter set successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_argument_invalid The supplied clock source is not a valid choice.
 * @retval status_device_state The information retrieved from the device
 * does not correspond to a valid state.
 * @retval status_no_device Device not opened.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_set_clock_source(sis8300drv_usr *sisuser, sis8300drv_clk_src clk_src) {
    int             status;
    uint32_t        ui32_reg_val, divider_cmd, divider_bypass, clock_source_ad9510;
    uint32_t        clock_divider_conf[8];
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    pthread_mutex_lock(&sisdevice->lock);
    
    status = sis8300drv_ad9510_spi_get_divider(sisdevice, &divider_cmd, &divider_bypass);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }
    
    clock_divider_conf[0] = divider_bypass + divider_cmd;
    clock_divider_conf[1] = divider_bypass + divider_cmd;
    clock_divider_conf[2] = divider_bypass + divider_cmd;
    clock_divider_conf[3] = divider_bypass + divider_cmd;
    clock_divider_conf[4] = divider_bypass + divider_cmd;
    clock_divider_conf[5] = divider_bypass + divider_cmd;
    clock_divider_conf[6] = divider_bypass + divider_cmd;
    clock_divider_conf[7] = 0xC000 + 0x00;

    clock_source_ad9510 = SIS8300DRV_CLKSRC_AD9510_MUX;
    ui32_reg_val = SIS8300DRV_CLKSRC_INTERNAL;

    switch (clk_src) {
        case clk_src_internal:
            ui32_reg_val = SIS8300DRV_CLKSRC_INTERNAL;
            break;
        case clk_src_rtm2:
            ui32_reg_val = SIS8300DRV_CLKSRC_RTM2;
            break;
        case clk_src_sma:
            ui32_reg_val = SIS8300DRV_CLKSRC_SMA;
            break;
        case clk_src_harlink:
            ui32_reg_val = SIS8300DRV_CLKSRC_HAR;
            break;
        case clk_src_backplane_a:
            ui32_reg_val = SIS8300DRV_CLKSRC_BPA;
            break;
        case clk_src_backplane_b:
            ui32_reg_val = SIS8300DRV_CLKSRC_BPB;
            break;
        case clk_src_rtm01:
            clock_source_ad9510 = SIS8300DRV_CLKSRC_AD9510_RTM;
            break;
        default:
            pthread_mutex_unlock(&sisdevice->lock);
            return status_argument_invalid;
    }

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }
	
	if (clock_source_ad9510 == SIS8300DRV_CLKSRC_AD9510_MUX) {
		status = sis8300_reg_write(sisdevice->handle,
		            SIS8300_CLOCK_DISTRIBUTION_MUX_REG, ui32_reg_val);
		if (status) {
		    pthread_mutex_unlock(&sisdevice->lock);
		    return status_device_access;
		}
    }
    
    status = sis8300drv_ad9510_spi_setup(sisdevice, clock_divider_conf, 1, clock_source_ad9510);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Get the clock divider for the clock used for analog data acquisition.
 * @param [in] sisuser User context struct.
 * @param [out] clk_div Current clock divider.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_state The information retrieved from the device
 * does not correspond to a valid clock divider.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_get_clock_divider(sis8300drv_usr *sisuser, sis8300drv_clk_div *clk_div) {
    int             status;
    uint32_t        divider_cmd, divider_bypass;
    sis8300drv_dev  *sisdevice;
    unsigned char hi_cyc, lo_cyc;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    /* Lock the device so there is no race condition during SPI communication. */
    pthread_mutex_lock(&sisdevice->lock);
    
    status = sis8300drv_ad9510_spi_get_divider(sisdevice, &divider_cmd, &divider_bypass);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);
    
    if (divider_bypass) {
        *clk_div = SIS8300DRV_CLKDIV_MIN;
        return status_success;
    }

    /* Mask out high and low cycles, see formula in setter function */
    hi_cyc = divider_cmd >> 4;
    lo_cyc = divider_cmd & 0xF;

    /* Calculate divider according to ad9510 manual */
    *clk_div = (hi_cyc + 1) + (lo_cyc + 1);

    return status_success;
}


/**
 * @brief Set the clock divider for the clock used for analog data acquisition.
 * @param [in] sisuser User context struct.
 * @param [in] clk_div Clock divider to set.
 *
 * @retval status_success Parameter set successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_argument_invalid The supplied clock divider is not a valid choice.
 * @retval status_device_state The information retrieved from the device
 * does not correspond to a valid state.
 * @retval status_no_device Device not opened.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_set_clock_divider(sis8300drv_usr *sisuser, sis8300drv_clk_div clk_div) {
    int             status;
    unsigned        clock_source_ad9510;
    uint32_t        divider_cmd, divider_bypass;
    uint32_t        clock_divider_conf[8];
    sis8300drv_dev  *sisdevice;
    unsigned char hi_cyc, lo_cyc;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    if (clk_div < SIS8300DRV_CLKDIV_MIN || clk_div > SIS8300DRV_CLKDIV_MAX) {
            return status_argument_range;
    }

    divider_bypass = SIS8300DRV_CLKDIV_NOBYPASS;

    if (clk_div == SIS8300DRV_CLKDIV_MIN) {
        divider_bypass = SIS8300DRV_CLKDIV_BYPASS;
    } 

    /* Formulas on how to calculate divider and duty cycle from ad9510 manual
     * Divide Ratio = (HIGH_CYCLES + 1) + (LOW_CYCLES + 1)
     * Duty Cycle = (HIGH_CYCLES + 1)/((HIGH_CYCLES + 1) + (LOW_CYCLES + 1))
     *
     * Calculate low and high cycles. For even dividers this will generate 50% duty cycle 
     * and for odd dividers it will generate the closest duty cycle over 50%
     */
    lo_cyc = clk_div / 2 - 1;
    hi_cyc = clk_div / 2 - 1 + clk_div % 2;
    
    /* Construct divider command */
    divider_cmd = hi_cyc << 4 | lo_cyc;
 
    clock_divider_conf[0] = divider_bypass + divider_cmd;
    clock_divider_conf[1] = divider_bypass + divider_cmd;
    clock_divider_conf[2] = divider_bypass + divider_cmd;
    clock_divider_conf[3] = divider_bypass + divider_cmd;
    clock_divider_conf[4] = divider_bypass + divider_cmd;
    clock_divider_conf[5] = divider_bypass + divider_cmd;
    clock_divider_conf[6] = divider_bypass + divider_cmd;
    clock_divider_conf[7] = 0xC000 + 0x00;

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }
    
    status = sis8300drv_ad9510_spi_get_source(sisdevice, &clock_source_ad9510);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    status = sis8300drv_ad9510_spi_setup(sisdevice, clock_divider_conf, 1, clock_source_ad9510);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Get the trigger source currently set on the device.
 * @param [in] sisuser User context struct.
 * @param [out] trg_src Current trigger source.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_state The information retrieved from the device
 * does not correspond to a valid trigger source.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_get_trigger_source(sis8300drv_usr *sisuser, sis8300drv_trg_src *trg_src) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_SAMPLE_CONTROL_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    ui32_reg_val &= SAMPLE_CONTROL_TRIGGER;

    switch(ui32_reg_val) {
        case SIS8300DRV_TRGSRC_SOFT:
            *trg_src = trg_src_soft;
            break;
        case SIS8300DRV_TRGSRC_EXT:
            *trg_src = trg_src_external;
            break;
        case SIS8300DRV_TRGSRC_INT:
            *trg_src = trg_src_internal;
            break;
        default:
            return status_device_state;
    }

    return status_success;
}


/**
 * @brief Set the trigger source used for analog data acquisition.
 * @param [in] sisuser User context struct.
 * @param [in] trg_src Trigger source to set.
 *
 * @retval status_success Parameter set successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_argument_invalid The supplied trigger source is not a valid choice.
 * @retval status_no_device Device not opened.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_set_trigger_source(sis8300drv_usr *sisuser, sis8300drv_trg_src trg_src) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_SAMPLE_CONTROL_REG, &ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    ui32_reg_val &= SAMPLE_CONTROL_CH_DIS;

    switch(trg_src) {
        case trg_src_soft:
            ui32_reg_val |= SIS8300DRV_TRGSRC_SOFT;
            break;
        case trg_src_external:
            ui32_reg_val |= SIS8300DRV_TRGSRC_EXT;
            break;
        case trg_src_internal:
            ui32_reg_val |= SIS8300DRV_TRGSRC_INT;
            break;
        default:
            pthread_mutex_unlock(&sisdevice->lock);
            return status_argument_invalid;
    }

    status = sis8300_reg_write(sisdevice->handle,
            SIS8300_SAMPLE_CONTROL_REG, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Get setup of the specified external trigger source.
 * @param [in] sisuser User context struct.
 * @param [in] trg_ext External trigger source.
 * @param [out] trg_mask Mask describing enabled trigger lines.
 * @param [out] edge_mask Mask describing trigger lines that trigger on
 * falling edge.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_argument_invalid The supplied external trigger source is not
 * a valid choice.
 * @retval status_no_device Device not opened.
 *
 * The trg_mask describes which trigger lines of the specified external
 * trigger source are enabled. Bit 0 corresponds to the first trigger line
 * and so on. #trg_ext_harlink has four trigger lines (bits 0-3) and
 * #trg_ext_mlvds has eight trigger lines (bits 0-7). Each set bit in
 * trg_mask means that the corresponding trigger line is enabled. The meaning
 * of trg_edge is analogous where each set bit means that the corresponding
 * trigger line triggers on the falling edge of the signal.
 */
int sis8300drv_get_external_setup(sis8300drv_usr *sisuser,
        sis8300drv_trg_ext trg_ext, unsigned *trg_mask, unsigned *edge_mask) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    switch(trg_ext) {
        case trg_ext_harlink:
            status = sis8300_reg_read(sisdevice->handle,
                    SIS8300_HARLINK_IN_OUT_CONTROL_REG, &ui32_reg_val);
            if (status) {
                return status_device_access;
            }
            *trg_mask = (ui32_reg_val & 0xF00) >> 8;
            *edge_mask = (ui32_reg_val & 0xF000) >> 12;
            break;
        case trg_ext_mlvds:
            status = sis8300_reg_read(sisdevice->handle,
                    SIS8300_MLVDS_IO_CONTROL_REG, &ui32_reg_val);
            if (status) {
                return status_device_access;
            }
            *trg_mask = (ui32_reg_val & 0xFF00) >> 8;
            *edge_mask = ui32_reg_val & 0xFF;
            break;
        default:
            return status_argument_invalid;
    }

    return status_success;
}


/**
 * @brief Configure the setup of the specified external trigger source.
 * @param [in] sisuser User context struct.
 * @param [in] trg_ext External trigger source.
 * @param [in] trg_mask Mask describing enabled trigger lines.
 * @param [in] edge_mask Mask describing trigger lines that trigger on
 * falling edge.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_argument_invalid The supplied external trigger source is not
 * a valid choice.
 * @retval status_no_device Device not opened.
 *
 * The trg_mask describes which trigger lines of the specified external
 * trigger source are enabled. Bit 0 corresponds to the first trigger line
 * and so on. #trg_ext_harlink has four trigger lines (bits 0-3) and
 * #trg_ext_mlvds has eight trigger lines (bits 0-7). Each set bit in
 * trg_mask means that the corresponding trigger line is enabled. The meaning
 * of trg_edge is analogous where each set bit means that the corresponding
 * trigger line triggers on the falling edge of the signal.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_set_external_setup(sis8300drv_usr *sisuser,
        sis8300drv_trg_ext trg_ext, unsigned trg_mask, unsigned edge_mask) {
    int             status;
    uint32_t        ui32_reg_val, ui32_reg_addr, ui32_reg_mask;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    switch(trg_ext) {
        case trg_ext_harlink:
            ui32_reg_addr = SIS8300_HARLINK_IN_OUT_CONTROL_REG;
            ui32_reg_mask = ((trg_mask & 0xF) | ((edge_mask & 0xF) << 4)) << 8;
            break;
        case trg_ext_mlvds:
            ui32_reg_addr = SIS8300_MLVDS_IO_CONTROL_REG;
            ui32_reg_mask = ((trg_mask & 0xFF) << 8) | (edge_mask & 0xFF);
            break;
        default:
            return status_argument_invalid;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300_reg_read(sisdevice->handle,
            ui32_reg_addr, &ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    ui32_reg_val &= EXT_TRG_IO;
    ui32_reg_val |= ui32_reg_mask;

    status = sis8300_reg_write(sisdevice->handle,
            ui32_reg_addr, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Get setup of the specified channel's trigger configuration.
 * @param [in] sisuser User context struct.
 * @param [in] channel Channel number.
 * @param [out] enable Whether the channel is enabled as an internal
 * trigger source (0 - disabled, 1 - enabled).
 * @param [out] mode The trigger mode for the channel (0 - threshold, 1 - FIR).
 * @param [out] threshold The contents of the threshold register corresponding
 * to the channel's trigger configuration (the meaning is different for
 * different trigger modes).
 * @param [out] condition The trigger condition for the channel
 * (0 - GT, 1 - LT).
 * @param [out] pulse_length Pulse length of the FIR filter.
 * @param [out] sum_gap Gap time of the FIR filter.
 * @param [out] peaking_time Peaking time of the FIR filter.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_argument_range The desired channel doesn't exist.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * For a detailed description of the meaning of return values consult the
 * device technical documentation.
 */
int sis8300drv_get_internal_setup(sis8300drv_usr *sisuser, unsigned channel,
        unsigned *enable, unsigned *mode, unsigned *threshold, unsigned *condition,
        unsigned *pulse_length, unsigned *sum_gap, unsigned *peaking_time) {
    int             status;
    uint32_t        ui32_reg_val, ui32_reg_addr;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    if (channel >= SIS8300DRV_NUM_AI_CHANNELS) {
        return status_argument_range;
    }

    ui32_reg_addr = SIS8300DRV_CH_SETUP_FIRST + channel;
    status = sis8300_reg_read(sisdevice->handle,
            ui32_reg_addr, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    *enable = (ui32_reg_val >> CH_SETUP_ENABLE) & 0x1;
    *condition = (ui32_reg_val >> CH_SETUP_CONDITION) & 0x1;
    *mode = (ui32_reg_val >> CH_SETUP_MODE) & 0x1;
    *pulse_length = (ui32_reg_val >> CH_SETUP_PULSE) & 0xFF;
    *sum_gap = (ui32_reg_val >> CH_SETUP_SUMG) & 0x1F;
    *peaking_time = (ui32_reg_val & 0x1F) >> CH_SETUP_PEAK;

    ui32_reg_addr = SIS8300DRV_CH_THRESHOLD_FIRST + channel;
    status = sis8300_reg_read(sisdevice->handle,
            ui32_reg_addr, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    *threshold = (unsigned)ui32_reg_val;

    return status_success;
}


/**
 * @brief Set the specified channel's trigger configuration.
 * 
 * @param [in] sisuser User context struct.
 * @param [in] channel Channel number.
 * @param [in] enable Whether the channel is enabled as an internal
 * trigger source (0 - disabled, 1 - enabled).
 * @param [in] mode The trigger mode for the channel (0 - threshold, 1 - FIR).
 * @param [in] threshold The contents of the threshold register corresponding
 * to the channel's trigger configuration (the meaning is different for
 * different trigger modes).
 * @param [in] condition The trigger condition for the channel
 * (0 - GT, 1 - LT).
 * @param [in] pulse_length Pulse length of the FIR filter.
 * @param [in] sum_gap Gap time of the FIR filter.
 * @param [in] peaking_time Peaking time of the FIR filter.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_argument_range One of the supplied arguments is out of range.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * Allowed values of funtion arguments:
 *  * channel < 10
 *  * enable {0,1}
 *  * mode {0,1}
 *  * condition {0,1}
 *  * pulse_length < 256
 *  * sum_gap < 17
 *  * peaking_time < 17
 *
 * For a detailed description of the meaning of function arguments consult the
 * device technical documentation.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_set_internal_setup(sis8300drv_usr *sisuser, unsigned channel,
        unsigned enable, unsigned mode, unsigned threshold, unsigned condition,
        unsigned pulse_length, unsigned sum_gap, unsigned peaking_time) {
    int             status;
    uint32_t        ui32_reg_val, ui32_reg_addr;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    if (channel >= SIS8300DRV_NUM_AI_CHANNELS ||
        enable > 1 ||
        mode > 1 ||
        condition > 1 ||
        pulse_length > 0xFF ||
        sum_gap > 16 ||
        peaking_time > 16) {
        return status_argument_range;
    }

    ui32_reg_val = 0;
    ui32_reg_val |= enable << CH_SETUP_ENABLE;
    ui32_reg_val |= condition << CH_SETUP_CONDITION;
    ui32_reg_val |= mode << CH_SETUP_MODE;
    ui32_reg_val |= pulse_length << CH_SETUP_PULSE;
    ui32_reg_val |= sum_gap << CH_SETUP_SUMG;
    ui32_reg_val |= peaking_time << CH_SETUP_PEAK;
    ui32_reg_addr = SIS8300DRV_CH_SETUP_FIRST + channel;

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300_reg_write(sisdevice->handle,
            ui32_reg_addr, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    ui32_reg_val = (uint32_t)threshold;
    ui32_reg_addr = SIS8300DRV_CH_THRESHOLD_FIRST + channel;
    status = sis8300_reg_write(sisdevice->handle,
            ui32_reg_addr, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Arm or trigger the device.
 * @param [in] sisuser User context struct.
 *
 * @retval status_success Card armed successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * Resets the sample logic and configures the card so it starts waiting
 * for internal/external trigger or triggers an acquisition immediately,
 * depending on current trigger choice. If the current trigger choice is
 * #trg_src_external or #trg_src_internal the device starts waiting for a trigger
 * and if the current trigger choice is #trg_src_soft it starts an acquisition
 * immediately.
 *
 * It also marks the device as armed which disables setting all parameters
 * related to analog input functionality. The user must call
 * #sis8300drv_wait_acq_end before accessing analog input functionality.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_arm_device(sis8300drv_usr *sisuser) {
    int             status;
    uint32_t        ui32_reg_val, type;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    /* Should not arm if there are pending operations on device. */
    pthread_mutex_lock(&sisdevice->lock);

    /* Check if we need to arm or start immediately. */
    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_SAMPLE_CONTROL_REG, &ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    ui32_reg_val &= SAMPLE_CONTROL_TRIGGER;
    type = SIS8300DRV_TRG_START;
    if (ui32_reg_val) {
        type = SIS8300DRV_TRG_ARM;
    }

    if (!sisdevice->armed) {
        /* Reset sampling logic. */
        status = sis8300_reg_write(sisdevice->handle,
                SIS8300_ACQUISITION_CONTROL_STATUS_REG, SIS8300DRV_RESET_ACQ);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_access;
        }

        /* Wait until internal sampling logic is not busy anymore. */
        do {
            status = sis8300_reg_read(sisdevice->handle,
                    SIS8300_ACQUISITION_CONTROL_STATUS_REG, &ui32_reg_val);
            if (status) {
                pthread_mutex_unlock(&sisdevice->lock);
                return status_device_access;
            }
        } while (ui32_reg_val & 0x30);

        status = sis8300_reg_write(sisdevice->handle,
                SIS8300_ACQUISITION_CONTROL_STATUS_REG, type);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_access;
        }
        
        sisdevice->armed = 1;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Disarm the device.
 * @param [in] sisuser User context struct.
 *
 * @retval status_success Card disarmed successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * Disables sampling, resets the sample logic and set the device as disarmed.
 * The device is marked as disarmed even if the function fails in communicating
 * with the device.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_disarm_device(sis8300drv_usr *sisuser) {
    int             status;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    sisdevice->armed = 0;

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300_ACQUISITION_CONTROL_STATUS_REG, SIS8300DRV_RESET_ACQ);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Wait for the device to be removed (physically) from the system.
 * @param [in] sisuser User context struct.
 *
 * @retval status_success Device removed.
 * @retval status_argument_invalid Supplied user context struct or file node invalid.
 * @retval status_internal Could not establish udev monitoring.
 * 
 * Start monitoring udev events and block until a remove event is received
 * for the device represented by the node specified in the user 
 * context struct #sis8300drv_usr. The device does not have to be actually
 * opened (through a call to #sis8300drv_open_device) only the user context
 * has to be allocated and the name of the device file node has to be valid.
 */
int sis8300drv_wait_remove(sis8300drv_usr *sisuser) {
    struct udev         *udev;
    struct udev_device  *udev_dev;
    struct udev_monitor *udev_mon;
    int                 status, removed, udev_mon_fd;
    fd_set              udev_mon_fds;
    
    if (!sisuser || !sisuser->file) {
        return status_argument_invalid;
    }
    
    udev = udev_new();
    if (!udev) {
        return status_internal;
    }
    
    udev_mon = udev_monitor_new_from_netlink(udev, "udev");
    if (!udev_mon) {
        udev_unref(udev);
        return status_internal;
    }
    
    status = udev_monitor_filter_add_match_subsystem_devtype(udev_mon, 
            "sis8300", NULL);
    if (status) {
        udev_monitor_unref(udev_mon);
        udev_unref(udev);
        return status_internal;
    }
            
    status = udev_monitor_enable_receiving(udev_mon);
    if (status) {
        udev_monitor_unref(udev_mon);
        udev_unref(udev);
        return status_internal;
    }
    
    udev_mon_fd = udev_monitor_get_fd(udev_mon);
    
    removed = 0;
    do {
        FD_ZERO(&udev_mon_fds);
        FD_SET(udev_mon_fd, &udev_mon_fds);
        
        status = select(udev_mon_fd + 1, &udev_mon_fds, NULL, NULL, NULL);
        
        if (status > 0 && FD_ISSET(udev_mon_fd, &udev_mon_fds)) {
            udev_dev = udev_monitor_receive_device(udev_mon);
            if (udev_dev) {
                if (!strcmp(udev_device_get_devnode(udev_dev), sisuser->file) &&
                        !strcmp(udev_device_get_action(udev_dev), "remove")) {
                    removed = 1;
                }
                udev_device_unref(udev_dev);
            }
        }
    } while (!removed);
    
    udev_monitor_unref(udev_mon);
    udev_unref(udev);
    
    return status_success;
}


/**
 * @brief Wait until acquisition and sampling is complete.
 * @param [in] sisuser User context struct.
 *
 * @retval status_success Acquisition not in progress.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * Blocks until acquisition and sampling is complete. This is achieved by
 * polling the relevant device register until it indicates end of acquisition
 * and sampling. During register polling it also checks if the user has
 * requested that the device should be unarmed in which case it stops.
 * The device is marked as disarmed even if the function fails in communicating
 * with the device.
 */
int sis8300drv_wait_acq_end(sis8300drv_usr *sisuser) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    do {
        status = sis8300_reg_read(sisdevice->handle,
                SIS8300_ACQUISITION_CONTROL_STATUS_REG, &ui32_reg_val);
        usleep(sisdevice->poll_period);
    } while (!status && sisdevice->armed && (ui32_reg_val & SIS8300DRV_ACQ_ACTIVE));

    sisdevice->armed = 0;

    return status < 0 ? status_device_access : status_success;
}


/**
 * @brief Wait until a "daq-done" or "user defined interrupt" happens.
 * @param [in] sisuser User context struct.
 * @param [in] irq_type Type of irq to wait for.
 * @param [in] timeout Timeout to wait for in milliseconds.
 *
 * @retval status_success Irq happened.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 * @retval status_argument_invalid Irq type invalid.
 * @retval status_irq_timeout Waiting for irq timedout.
 * @retval status_irq_error Waiting was interrupted by a signal.
 *
 * Blocks until the board fires the interrupt that is defined by the 
 * irq_type parameter or timeout expires (a zero timeout means wait indefinitely).
 * Possible values for the irq_type parameter are enumerated in #sis8300drv_irq_type.
 *
 * NOTE: The "daq-done" interrupt is only supported by a limited number
 * of firmware versions. Newer firmware versions (2.x and later) don't support it.
 */
int sis8300drv_wait_irq(sis8300drv_usr *sisuser, sis8300drv_irq_type irq_type, unsigned timeout) {
    int             status;
    sis8300_irq     irq;
    sis8300drv_dev  *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    switch (irq_type) {
        case irq_type_daq:
            irq.type = SIS8300_DAQ_IRQ;
            break;
        case irq_type_usr:
            irq.type = SIS8300_USR_IRQ;
            break;
        default:
            return status_argument_invalid;
    }
    
    irq.timeout = timeout;
    
    status = ioctl(sisdevice->handle, SIS8300_WAIT_IRQ, &irq);
    if (status) {
        return status_device_access;
    }
    switch (irq.status) {
        case SIS8300_IRQ_RELEASE:
            status = status_irq_release;
            break;
        case SIS8300_IRQ_TIMEOUT:
            status = status_irq_timeout;
            break;
        case SIS8300_IRQ_ERROR:
            status = status_irq_error;
            break;
        default:
            status = status_success;
            break;
    }

    return status;
}


/**
 * @brief Release all waiters on "daq-done" or "user defined interrupt".
 * @param [in] sisuser User context struct.
 * @param [in] irq_type Type of irq to release waiters for.
 *
 * @retval status_success 
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 * @retval status_argument_invalid Irq type invalid.
 * 
 * Possible values for the irq_type parameter are enumerated in #sis8300drv_irq_type.
 *
 * NOTE: The "daq-done" interrupt is only supported by a limited number
 * of firmware versions. Newer firmware versions (2.x and later) don't support it.
 */
int sis8300drv_release_irq(sis8300drv_usr *sisuser, sis8300drv_irq_type irq_type) {
    int             status;
    sis8300_irq     irq;
    sis8300drv_dev  *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    switch (irq_type) {
        case irq_type_daq:
            irq.type = SIS8300_DAQ_IRQ;
            break;
        case irq_type_usr:
            irq.type = SIS8300_USR_IRQ;
            break;
        default:
            return status_argument_invalid;
    }
    
    status = ioctl(sisdevice->handle, SIS8300_RELEASE_IRQ, &irq);
    if (status) {
        return status_device_access;
    }

    return status_success;
}


/**
 * @brief Read channel data from device memory.
 * @param [in] sisuser User context struct.
 * @param [in] channel The channel for which to transfer data.
 * @param [out] data User buffer that will contain the data on success.
 *
 * @retval status_success Data transfered successfully.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_argument_range The requested channel is greater than the number
 * of channels on the device.
 * @retval status_argument_invalid The requested channel is disabled.
 * @retval status_device_access Can't access device memory location or
 * device registers.
 * @retval status_device_read Can't transfer data from device memory.
 * @retval status_no_device Device not opened.
 *
 * Reads the data associated with a particular channel from device memory.
 * The channel must be enabled and the device must not be armed. The buffer
 * provided must be able to hold at least number-of-samples of 2-byte values.
 * 
 * Due to the limitation of the firmware data can be read from device memory only in
 * #SIS8300DRV_BLOCK_BYTES chunks and the reads have to be alligned to #SIS8300DRV_BLOCK_BYTES
 * offsets. This function doesn't check for that since the size to read and offset
 * to read from are determined by #sis8300drv_set_nsamples and #sis8300drv_conf_ch
 * respectively. Those functions ensure that the size of one channel worth of data
 * and that the locations in device memory where channels data is located are
 * alligned properly.
 * 
 * This function relies on reading the number of samples from registers and
 * using it to calculate how much data to read and from which addres in device
 * memory. Thus the number of samples should not be changed between the starting
 * the acquisition and reading channel data.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_read_ai(sis8300drv_usr *sisuser, unsigned channel, void *data) {
    int             status;
    unsigned        channel_mask, nsamples, offset, size;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    if (channel >= SIS8300DRV_NUM_AI_CHANNELS) {
        return status_argument_range;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300drv_get_nsamples(sisuser, &nsamples);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    status = sis8300drv_get_channel_mask(sisuser, &channel_mask);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    if (!(channel_mask & (1 << channel))) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_argument_invalid;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300DRV_CH_ADDRESS_FIRST + channel, &ui32_reg_val);
    if (status < 0) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    
    size = nsamples * SIS8300DRV_SAMPLE_BYTES;
    offset = (unsigned)ui32_reg_val * SIS8300DRV_BLOCK_BYTES;
    /* It wraps around the available memory size (happens for the last 
     * enabled channel when all memory is used). */
    if (offset == 0) {
        offset = sisdevice->mem_size;
    }
    /* The channel address registers actually hold the address
     * just after where the last sample was placed. */
    offset -= size;
    
    status = sis8300drv_read_ram_unlocked(sisdevice, offset, size, data);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Set analog output channel to a specific value.
 * @param [in] sisuser User context struct.
 * @param [in] channel Channel number to write to (0 or 1).
 * @param [in] data Value to write, in range [-1V,1V].
 *
 * @retval status_success Output successful.
 * @retval status_argument_range The channel argument is out of range
 * (should be 0 or 1).
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_write_ao(sis8300drv_usr *sisuser, unsigned channel, double data) {
    int             status;
    uint32_t        ui32_reg_val;
    uint16_t        data_raw;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    if (channel > 1 || data > 1 || data < -1) {
        return status_argument_range;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_DAC_DATA_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    /* This is supposed to be two's complement (chosen when we init the board)
     * but I don't get how this format is related to it (1V -> 0x0, -1V -> 0xFFFF). */
    data_raw = (uint16_t)((-data + 1.0) * 65535.0/2.0);
    
    ui32_reg_val &= 0xFFFF0000 >> ((uint32_t)channel*16);
    ui32_reg_val |= (uint32_t)data_raw << (channel*16);

    status = sis8300_reg_write(sisdevice->handle,
            SIS8300_DAC_DATA_REG, ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    return status_success;
}


/**
 * @brief Read analog output channel value.
 * @param [in] sisuser User context struct.
 * @param [in] channel Channel number to write to (0 or 1).
 * @param [out] data Analog output channel value.
 *
 * @retval status_success Data transfered successfully.
 * @retval status_argument_range The channel argument is out of range
 * (should be 0 or 1).
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_read_ao(sis8300drv_usr *sisuser, unsigned channel, double *data) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    if (channel > 1) {
        return status_argument_range;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_DAC_DATA_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    ui32_reg_val &= 0x0000FFFF << ((uint32_t)channel*16);
    ui32_reg_val >>= ((uint32_t)channel*16);

    *data = -((double)ui32_reg_val * 2.0/65535.0 - 1.0);

    return status_success;
}


/**
 * @brief Read the state of Harlink digital input port.
 * @param [in] sisuser User context struct.
 * @param [out] data Digital input port value.
 *
 * @retval status_success Data transfered successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_read_harlink(sis8300drv_usr *sisuser, unsigned *data) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_HARLINK_IN_OUT_CONTROL_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *data = (unsigned)(ui32_reg_val & 0xF);

    return status_success;
}


/**
 * @brief Get the serial number of the device.
 * @param [in] sisuser User context struct.
 * @param [out] serial Device serial number.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_get_serial(sis8300drv_usr *sisuser, unsigned *serial) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_SERIAL_NUMBER_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    *serial = (unsigned)ui32_reg_val & 0xFF;

    return status_success;
}


/**
 * @brief Get the firmware version of the device.
 * @param [in] sisuser User context struct.
 * @param [out] fw_version Device firmware version.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_get_fw_version(sis8300drv_usr *sisuser, unsigned *fw_version) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_IDENTIFIER_VERSION_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    *fw_version = (unsigned)ui32_reg_val;

    return status_success;
}


/**
 * @brief Get the firmware options of the device.
 * @param [in] sisuser User context struct.
 * @param [out] fw_options Device firmware options.
 *
 * @retval status_success Information retrieved successfully.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_get_fw_options(sis8300drv_usr *sisuser, unsigned *fw_options) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
            SIS8300_FIRMWARE_OPTIONS_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    *fw_options = (unsigned)ui32_reg_val;

    return status_success;
}


/**
 * @brief Get the maximum number of samples that will fit into device memory.
 * @param [in] sisuser User context struct.
 * @param [out] space Space on device memory in number of samples.
 *
 * @retval status_no_device Device not opened.
 */
int sis8300drv_get_space(sis8300drv_usr *sisuser, unsigned *space) {
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    *space = sisdevice->adc_mem_size / SIS8300DRV_SAMPLE_BYTES;

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
 * @retval status_no_device Device not opened.
 *
 * Details of return values and finction behaviour can be found in the
 * description of #sis8300drv_write_ram_unlocked. This function checks
 * whether the device is opened, waits until the device can be locked, and
 * then checks whether the device is armed and if not calls
 * #sis8300drv_write_ram_unlocked.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_write_ram(sis8300drv_usr *sisuser, unsigned offset, unsigned size, void *data) {
    int             status;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }
    
    status = sis8300drv_write_ram_unlocked(sisdevice, offset, size, data);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Read contents of device memory.
 * @param [in] sisuser User context struct.
 * @param [in] offset Offset to read from in device memory.
 * @param [in] size Size of the buffer to read.
 * @param [in] data User buffer to read into.
 *
 * @retval status_success Data transfered successfully.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_no_device Device not opened.
 *
 * Details of return values and finction behaviour can be found in the
 * description of #sis8300drv_read_ram_unlocked. This function checks
 * whether the device is opened, waits until the device can be locked, and
 * then checks whether the device is armed and if not calls
 * #sis8300drv_read_ram_unlocked.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_read_ram(sis8300drv_usr *sisuser, unsigned offset, unsigned size, void *data) {
    int             status;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }
     
    status = sis8300drv_read_ram_unlocked(sisdevice, offset, size, data);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Write a value to a device register.
 * @param [in] sisuser User context struct.
 * @param [in] address Address of register to write to.
 * @param [in] data Register value to write.
 *
 * @retval status_success Device successfully initialized.
 * @retval status_device_access Can't access device registers.
 * @retval status_device_armed This operation is not allowed on an armed device.
 * @retval status_no_device Device not opened.
 *
 * Calls to this function are serialized with respect to other calls that alter
 * the functionality of the device. This means that this function may block.
 */
int sis8300drv_reg_write(sis8300drv_usr *sisuser, unsigned address, unsigned data) {
    int             status;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300_reg_write(sisdevice->handle, (uint32_t)address, (uint32_t)data);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}


/**
 * @brief Read device register.
 * @param [in] sisuser User context struct.
 * @param [in] address Address of register to read from.
 * @param [out] data Register value.
 *
 * @retval status_success Device successfully initialized.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300drv_reg_read(sis8300drv_usr *sisuser, unsigned address, unsigned *data) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle, (uint32_t)address, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    *data = (unsigned)ui32_reg_val;

    return status_success;
}

