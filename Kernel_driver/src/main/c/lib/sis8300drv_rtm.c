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
 * @file sis8300drv_rtm.c
 * @brief Implementation RTM control related functions
 * @author kstrnisa
 */


#include <stdint.h>
#include <unistd.h>

#include "sis8300_reg.h"
#include "sis8300_defs.h"
#include "sis8300drv.h"
#include "sis8300drv_rtm.h"
#include "sis8300drv_utils.h"


/**
 * @brief Enable/disable VM output by setting the ILOCK status
 *
 * @param [in] sisuser  Device user context struct
 * @param [in] rtm      RTM Type, has to be VM capable, #rtm_dwc8vm1 or #rtm_ds8vm1
 * @param [in] enable   0 to disable, any other valule enable
 *
 * @retval status_success           Enable successful
 * @retval status_device_access     Can't access device registers.
 * @retval status_no_device         Device not opened.
 * @retval status_device_armed      Operation not allowed when device is armed
 * @retval status_argument_invalid  Invalid choice for RTM,
 *
 * The VM Modulator RTMs (DWC8VM1 and DS8VM1) will allow VM output when
 * ILOCK0 and ILOCKEN bits are high in #SIS8300_RTM_LVDS_IO_CONTROL_REG
 * struck register. This function can either enable or disable VM output,
 * but only does this if one of the VM type RTMs is selected (@see
 * #rtm_dwc8vm1 or #rtm_ds8vm1
 */
int sis8300drv_rtm_vm_output_enable(sis8300drv_usr *sisuser, sis8300drv_rtm rtm, unsigned enable) {
    int             status;
    uint32_t        ui32_reg_val;
    sis8300drv_dev  *sisdevice;

    /* check if the device is armed, take the lock */
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    /* 704 MHz will allow rf output on 0x700 and 0x500
     * 352 MHz will allow rf output on any input but 0x600 or 0x400
     * -> ILOCK0 in ILOCK EN morta bit high da mamo output*/
    ui32_reg_val = (uint32_t) (enable) ? 0x700 : 0x600;

    switch(rtm) {
    case rtm_ds8vm1:
    case rtm_dwc8vm1:
        status = sis8300_reg_write(sisdevice->handle, SIS8300_RTM_LVDS_IO_CONTROL_REG, ui32_reg_val);
        if (status) {
            status = status_device_access;
        }
        break;
    default:
        status = status_argument_invalid;
        break;
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status;
}


/* general - exported functions. for doxygen @see #_sis8300drv_i2c_rw */
int sis8300drv_i2c_rtm_read(sis8300drv_usr *sisuser, unsigned i2c_bus_addr, int command, unsigned *data_bytes, int nr_bytes) {
    sis8300drv_dev *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    return sis8300drv_i2c_rw( sisdevice, (uint32_t) SIS8300_I2C_READ,
                               (uint32_t) SIS8300_RTM_I2C_BUS_REG, (uint32_t) i2c_bus_addr,
                               command, data_bytes, nr_bytes);
}


int sis8300drv_i2c_rtm_write(sis8300drv_usr *sisuser, unsigned i2c_bus_addr, int command, unsigned *data_bytes, int nr_bytes) {
    sis8300drv_dev *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    return sis8300drv_i2c_rw( sisdevice, (uint32_t) SIS8300_I2C_WRITE,
                               (uint32_t) SIS8300_RTM_I2C_BUS_REG, (uint32_t) i2c_bus_addr,
                               command, data_bytes, nr_bytes);
}


/* pca9535 specific
 * with this I/O expander, the registers are paired together. Consecutive reads, without a restart will return all
 * the paired register values, not return the same register value twice. This is why these reads/writes are
 * made byte by byte - call @see #_sis8300drv_i2c_rw with num of bytes = 1
 * */
int sis8300drv_i2c_rtm_pca9535_read(sis8300drv_usr *sisuser, unsigned i2c_bus_addr, unsigned pca9535_reg, unsigned *data_byte) {
    sis8300drv_dev *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    return sis8300drv_i2c_rw( sisdevice, (uint32_t) SIS8300_I2C_READ,
                               (uint32_t) SIS8300_RTM_I2C_BUS_REG, (uint32_t) i2c_bus_addr,
                               (int)pca9535_reg, data_byte, 1);
}


int sis8300drv_i2c_rtm_pca9535_write(sis8300drv_usr *sisuser, unsigned i2c_bus_addr, unsigned pca9535_reg, unsigned data_byte) {
    sis8300drv_dev *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    return sis8300drv_i2c_rw( sisdevice, (uint32_t) SIS8300_I2C_WRITE,
                               (uint32_t) SIS8300_RTM_I2C_BUS_REG, (uint32_t) i2c_bus_addr,
                               (int)pca9535_reg, &data_byte, 1);
}


/* ltc2493 specific
 * This one does not have several registers and requires no command byte. which is why it is called with command
 * byte = -1
 * */
int sis8300drv_i2c_rtm_ltc2493_read(sis8300drv_usr *sisuser, unsigned i2c_bus_addr, unsigned data_bytes[4]) {
    sis8300drv_dev *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    return sis8300drv_i2c_rw( sisdevice, (uint32_t) SIS8300_I2C_READ,
                               (uint32_t) SIS8300_RTM_I2C_BUS_REG, (uint32_t) i2c_bus_addr,
                               -1, data_bytes, 4);
}


int sis8300drv_i2c_rtm_ltc2493_write(sis8300drv_usr *sisuser, unsigned i2c_bus_addr, unsigned data_bytes[2]) {
    sis8300drv_dev *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    return sis8300drv_i2c_rw( sisdevice, (uint32_t) SIS8300_I2C_WRITE,
                               (uint32_t) SIS8300_RTM_I2C_BUS_REG, (uint32_t) i2c_bus_addr,
                               -1, data_bytes, 2);
}


/**
 * @brief set the RTM channel attenuation
 *
 * @param [in] sisuser      Device user context struct
 * @param [in] rtm          RTM type, @see #sis8300drv_rtm
 * @param [in] att_num      Attenuator number to set (@see #SIS8300DRV_RTM_ATT_CH0 to #SIS8300DRV_RTM_ATT_VM)
 * @param [in] att_val_idx  Value for attenuation, (6 bits, 0x0 to 0x3f,
 *                          AI0 - AI7: attenuation[dBm] = -31.5  + att_val_idx / 2, max = 0.0
 *                          VM output: attenuation[dBm] = -15.75 + att_val_idx / 4. max = 0.0
 *
 * @retval status_success           Attenuator value set successful
 * @retval status_device_access     Can't access device registers.
 * @retval status_no_device         Device not opened.
 * @retval status_device_armed      Operation not allowed when device is armed
 * @retval status_argument_invalid  Invalid choice for RTM or att_val_idx,
 *                                  or the device is not SIS8300L with 0x2 major version fw
 * @retval status_i2c_busy          i2c bus timeout
 * @retval status_i2c_nack          i2c write not acknowledged from slave
 *
 * This will setup the attenuator value for the RTM trough SIS8300L register 0x47,
 * intended for i2c communication with the attached RTM.
 * Only dwc8vm1 and ds8vm1 RTMs were available for testing, although
 * the function should also be similar with the dwc8300
 * (that one only has 10 AI channels for att setting).
 *
 * RTM type specific quirks:
 * The function will take into account the fact that the SPI and CLK lines
 * are swapped on the ds8vm1 RTM with respect to the dwc8vm1.
 * It will also make sure to keep the VM common mode bit to high when writing,
 * if this is a dwc8vm1 RTM. The ds8vm1 RTM does not have any need for setting
 * the common mode, because it has it fixed.
 */
int sis8300drv_i2c_rtm_attenuator_set(sis8300drv_usr *sisuser, sis8300drv_rtm rtm, unsigned att_num, unsigned att_val_idx) {
    int             status, i;
    sis8300drv_dev  *sisdevice;

    uint32_t ui32_data, data_byte, mask;

    uint32_t clk_bit = SIS8300_PCA9535_CLK_BIT;
    uint32_t spi_bit = SIS8300_PCA9535_SPI_BIT;

    /* check if the device is armed, take the lock */
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    /* check which rtm we are talking about */
    switch (rtm) {
        case rtm_sis8900:
            status = status_argument_invalid;
            break;
        case rtm_dwc8vm1:
            status = (att_num > SIS8300DRV_RTM_ATT_VM) ? status_argument_invalid : status_success;
            break;
        case rtm_ds8vm1:
            status = (att_num > SIS8300DRV_RTM_ATT_VM) ? status_argument_invalid : status_success;
            /* they are swapped on the direct sampling rtm */
            clk_bit = SIS8300_PCA9535_SPI_BIT;
            spi_bit = SIS8300_PCA9535_CLK_BIT;
            break;
      case rtm_dwc8300lf:
            status = (att_num > SIS8300DRV_RTM_ATT_CH9) ? status_argument_invalid : status_success;
            break;
        default:
            status = status_argument_invalid;
            break;
    }
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* Set pca IO pins to output by writing 0 to registers 6 and 7     */
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 6, 0x0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 7, 0x0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* write 0 to register 2 that has SDA and CLK pins */
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 2, 0x0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* if this is the DWC 704 rtm, set common mode to high!!
     * 8900 and dwc8300 don't have COMM DAC, ds has it fixed*/
    if (rtm == rtm_dwc8vm1) {
        // VCM_DAC_CS const to high!
        status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 3, SIS8300_PCA9535_CMDDAC_BIT);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }
    }

    /* now load the data to the PCA, MSB first */
    mask = 0x1 << (SIS8300_PCA9535_ATT_DATA_BITS - 1);
    ui32_data = (uint32_t) att_val_idx;
    data_byte = 0;

    for (i = 0; i < SIS8300_PCA9535_ATT_DATA_BITS; i++) {
        /* check if bit is set and set or clear the SPI data bit*/
        data_byte = (ui32_data & mask) ? spi_bit : 0x0;

        /* write the SPI and hold it while toggling the clock*/
        status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 2, data_byte);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }
        status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 2, data_byte | clk_bit);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }
        status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 2, data_byte);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }

        /* move to the next bit */
        mask >>= 1;
    }

    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 2, 0x0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }


    /* now the data is loaded in. Latch the correct io pin, to load the
     * value to the correct attenuator */
    if (att_num < 6) {
        /* first 6 attenuators (0-5) are on the register 2 */
        data_byte = (uint32_t) 0x1 << (att_num + 2);
        ui32_data = (uint32_t) 0x2;
    }
    else {
        /* attenuators 6 -9 or 6 -7 plus VM att and COMM setup are on register 3 */
        data_byte = (uint32_t) 0x1 << (att_num - 6);
        ui32_data = (uint32_t) 0x3;
        if (rtm == rtm_dwc8vm1) {
            data_byte |= SIS8300_PCA9535_CMDDAC_BIT; // VCM_DAC_CS const to high!
        }
    }

    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, ui32_data, data_byte);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }
    /* unlatch,
       if we are writing to reg 3, we need to keep the cmd mode up,
       and NOT IF we are writing to register 2 */
    data_byte = (rtm == rtm_dwc8vm1 && ui32_data == 0x3) ? SIS8300_PCA9535_CMDDAC_BIT : 0x0; // VCM_DAC_CS const to high!
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, ui32_data, data_byte);

    pthread_mutex_unlock(&sisdevice->lock);
    return status;
}

/**
 * @biref Set the Common mode DAC

 * @param [in] sisuser      Device user context struct
 * @param [in] rtm          RTM type, @see #sis8300drv_rtm
 * @param [in] val          Value for the common mode voltage.
 *                          According to the manual, the desired CMMOD is 1.7 V (
 *                          @see #SIS8300DRV_DWC8VM1_CMDAC_DEFOPT) for the currently
 *                          used chip
 * @retval status_success           CMMOD value set successful
 * @retval status_device_access     Can't access device registers.
 * @retval status_no_device         Device not opened.
 * @retval status_device_armed      Operation not allowed when device is armed
 * @retval status_argument_invalid  Invalid choice for RTM,
 *                                  or the device is not SIS8300L with 0x2 major version fw
 * @retval status_i2c_busy          i2c bus timeout
 * @retval status_i2c_nack          i2c write not acknowledged from slave
 *
 * This will set the common mode voltage on the dwc8vm1 RTM.
 * The ds8vm1 RTM has this mode fixed, and other RTMs do not have a vm.
 */
int sis8300drv_i2c_rtm_vm_common_mode_set(sis8300drv_usr *sisuser, sis8300drv_rtm rtm, unsigned val) {
    int status, i;
    sis8300drv_dev *sisdevice;

    uint32_t ui32_data, mask, data_byte;

    uint32_t clk_bit = SIS8300_PCA9535_CLK_BIT;
    uint32_t spi_bit = SIS8300_PCA9535_SPI_BIT;

    /* this setting only makes sense on the dwc8vm1 rtm,
     * dwc8300 does not hav VM, ds8vm1 has the common mode fixed
     *
     * ALSO, the desired common mode voltage for currently used chip is
     * 1.7 V (0x352)
     */
    if (rtm != rtm_dwc8vm1) {
        return status_argument_invalid;
    }

    /* check if the device is armed, take the lock */
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    /* Set pca IO pins to output by writing 0 to registers 6 and 7     */
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 6, 0x0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 7, 0x0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* write 0 to register 2 that has SDA and CLK pins */
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 2, 0x0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* VM DAC CS to high and than to low */
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 3, SIS8300_PCA9535_CMDDAC_BIT);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 3, 0x0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* construct the data to send */
    ui32_data = (uint32_t) val;
    ui32_data &= 0x0000ffff;
    ui32_data <<= 4;
    /* Command definition:
     * C2 - C0: 0x3  (Write to and update DAC)
     * A2 - A0: 0x7  (All DACs)
     *
     * CmdAddr [x x c2 c1 c0 a2 a1 a0]: 0x1f
     */
    ui32_data |= 0x1f << 16;

    /* send the data bit by bit */
    mask = (uint32_t) 0x1 << (SIS8300_PCA9535_CMDAC_DATA_BITS - 1);
    for (i = 0; i < SIS8300_PCA9535_CMDAC_DATA_BITS; i++) {

        data_byte = (mask & ui32_data) ? spi_bit : 0;

        /* write the data and toggle the clock line */
        status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 0x2, data_byte);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }
        status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 0x2, data_byte | clk_bit);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }
        status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 0x2, data_byte);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }

        /* move to the next bit */
        mask >>= 1;
    }

    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 2, 0x0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    /* VCM DAC CS to high */
    status = sis8300drv_i2c_rtm_pca9535_write(sisuser, SIS8300_PCA9535_I2C_ADDR, 3, SIS8300_PCA9535_CMDDAC_BIT);

    pthread_mutex_unlock(&sisdevice->lock);
    return status;
}


/**
 * @brief Get the temperature from RTM

 * @param [in] sisuser      Device user context struct
 * @param [in] rtm          RTM type, @see #sis8300drv_rtm
 * @param [in] source       Actual I2C device returning temperature readout, @see #sis8300drv_rtm_temp
 * @param [out] val         Temperature value in C degrees.
 * @retval status_success           Temperature readout successful
 * @retval status_device_access     Can't access device registers.
 * @retval status_no_device         Device not opened.
 * @retval status_argument_invalid  Invalid choice for RTM,
 *                                  or the device is not SIS8300L with 0x2 major version fw
 * @retval status_i2c_busy          i2c bus timeout
 * @retval status_i2c_nack          i2c write not acknowledged from slave
 *
 * This will get the temperature from the desired source. Tested on DESY
 * DWC8VM1 RTM.
 * NOTE: This function should not be called during data acquisition as it will
 *       lock the access to the driver for too long. It taks >150 ms to
 *       perform temperature readout and conversion.
 */
int sis8300drv_i2c_rtm_temperature_get(sis8300drv_usr *sisuser,
		sis8300drv_rtm rtm, sis8300drv_rtm_temp source, double *val) {
    int             status;
    sis8300drv_dev  *sisdevice;

    uint32_t ui32_data[4];
    uint32_t conv;

    /* check if the device is armed, take the lock */
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&sisdevice->lock);

    /* check which rtm we are talking about */
    switch (rtm) {
        case rtm_dwc8vm1:
        case rtm_dwc8300lf:
        	status = status_success;
            break;
        default:
            status = status_argument_invalid;
            break;
    }
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    switch (source) {
		case rtm_temp_ad8363:
			ui32_data[0] = SIS8300_LTC2493_AD8363_TEMP_CMD & 0xFF;
			ui32_data[1] = (SIS8300_LTC2493_AD8363_TEMP_CMD >> 8) & 0xFF;
			status = status_success;
			break;
		case rtm_temp_ltc2493:
			ui32_data[0] = SIS8300_LTC2493_LTC2493_TEMP_CMD & 0xFF;
			ui32_data[1] = (SIS8300_LTC2493_LTC2493_TEMP_CMD >> 8) & 0xFF;
			status = status_success;
			break;
		default:
			status = status_argument_invalid;
			break;
    }
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    status = sis8300drv_i2c_rtm_ltc2493_write(sisuser, SIS8300_LTC2493_I2C_ADDR, ui32_data);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }


	/* Teperature chip conversion takes > 150 ms! */
    usleep(200000);

    ui32_data[0] = 0;
    ui32_data[1] = 0;
    ui32_data[2] = 0;
    ui32_data[3] = 0;
    status = sis8300drv_i2c_rtm_ltc2493_read(sisuser, SIS8300_LTC2493_I2C_ADDR, ui32_data);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    conv = (ui32_data[3] & 0xFF);
    conv |= (ui32_data[2] & 0xFF) << 8;
    conv |= (ui32_data[1] & 0xFF) << 16;
    conv |= (ui32_data[0] & 0xFF) << 24;

    switch (source) {
		case rtm_temp_ad8363:
			conv = conv & ~0x80000000;
			conv >>= 6;
		    *val = (double)((conv * (3.3 / 0x1000000)) - 1.275) / 0.005;
			status = status_success;
			break;
		case rtm_temp_ltc2493:
		    conv = conv & ~0x80000000;
		    conv >>= 7;
		    *val = (double)((conv * 3.3)/1570) - 273.15;
			status = status_success;
			break;
		default:
			status = status_argument_invalid;
			break;
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status;
}


/******************************************************************************/
/*                 INTERNAL FUNCTIONS TO CONTROL I2C TRANSFER                 */
/******************************************************************************/


/**
 * Write and read transactions are practically identical,
 * as far as preparing for the transaction goes
 *
 * devices that do not require a command byte should have it set to -1 and it will be
 * ignored
 */
int sis8300drv_i2c_rw(sis8300drv_dev *sisdevice, uint32_t rw, uint32_t i2c_reg_addr,
        uint32_t i2c_bus_addr, int command_byte, unsigned *data_bytes, int nr_bytes) {
    int             status, i, timeout;
    uint32_t        ui32_reg_val, ack;

    timeout = SIS8300_I2C_TIMEOUT;

    switch (rw) {
        case SIS8300_I2C_READ:
        case SIS8300_I2C_WRITE:
            break;
        default:
            return status_argument_invalid;
    }

    /* 1. CHECK IF THE DEVICE SUPPORTS I2C FUNCTIONALITY */
    if (sisdevice->type != SIS8300_SIS8300L) {
        return status_argument_invalid;
    }

    status = sis8300_reg_read(sisdevice->handle, SIS8300_IDENTIFIER_VERSION_REG, &ui32_reg_val);
    if(status) {
        return status_device_access;
    }

    if ((ui32_reg_val & 0xf000) != 0x2000) {
        return status_argument_invalid;
    }

    /* 2. START I2C TRANSFER */
    status = sis8300drv_i2c_request_send(sisdevice, i2c_reg_addr, SIS8300_I2C_START, timeout);
    if (status) {
        return status;
    }

    /* 5. do a R or W */
    if (rw == SIS8300_I2C_READ) {

        /* 3. ADDRESS THE SLAVE
         *  device located at at bus address specified by i2c_bus_addr
         *  Data is transmitted to the PCA9535/PCA9535C by sending the device address and
         *  setting the least significant bit to a logic 0 to indicate a write
         *
         *  Slave Address: 0 1 0 0 A2 A1 A0 R/W, As are configurable, the beginning is not
         */
    	status = sis8300drv_i2c_byte_write(sisdevice, i2c_reg_addr, (i2c_bus_addr << 1) | rw, &ack, timeout);
    	if (status) {
    		return status;
    	}
    	if (!ack) {
    		/* try to free the bus, don't care about result - it's write_err */
    		sis8300drv_i2c_request_send(sisdevice, i2c_reg_addr, SIS8300_I2C_STOP, timeout);
    		return status_i2c_nack;
    	}

        /* 4. SEND THE COMMAND BYTE TO THE DEVICE */
        if (command_byte >= 0) {
            status = sis8300drv_i2c_byte_write(sisdevice, i2c_reg_addr, (uint32_t) command_byte, &ack, timeout);
            if (status) {
                return status;
            }
            if (!ack) {
                /* try to free the bus, don't care about result - its write_err */
                sis8300drv_i2c_request_send(sisdevice, i2c_reg_addr, SIS8300_I2C_STOP, timeout);
                return status_read;
            }
        }

        /* get the data byte by byte */
        for (i = 0; i < nr_bytes; i++) {
            status = sis8300drv_i2c_byte_read(sisdevice, i2c_reg_addr, &ui32_reg_val, 0, timeout);
            data_bytes[i] = (unsigned) ui32_reg_val;
            if (status) {
                return status;
            }
        }
    } else if (rw == SIS8300_I2C_WRITE) {

        /* 3. ADDRESS THE SLAVE
         *  device located at at bus address specified by i2c_bus_addr
         *  Data is transmitted to the PCA9535/PCA9535C by sending the device address and
         *  setting the least significant bit to a logic 0 to indicate a write
         *
         *  Slave Address: 0 1 0 0 A2 A1 A0 R/W, As are configurable, the beginning is not
         */
    	status = sis8300drv_i2c_byte_write(sisdevice, i2c_reg_addr, i2c_bus_addr << 1, &ack, timeout);
    	if (status) {
    		return status;
    	}
    	if (!ack) {
    		/* try to free the bus, don't care about result - it's write_err */
    		sis8300drv_i2c_request_send(sisdevice, i2c_reg_addr, SIS8300_I2C_STOP, timeout);
    		return status_i2c_nack;
    	}

        /* 4. SEND THE COMMAND BYTE TO THE DEVICE */
        if (command_byte >= 0) {
            status = sis8300drv_i2c_byte_write(sisdevice, i2c_reg_addr, (uint32_t) command_byte, &ack, timeout);
            if (status) {
                return status;
            }
            if (!ack) {
                /* try to free the bus, don't care about result - its write_err */
                sis8300drv_i2c_request_send(sisdevice, i2c_reg_addr, SIS8300_I2C_STOP, timeout);
                return status_read;
            }
        }

    	/* if writing just do it */
        for (i = 0; i < nr_bytes; i++) {
            status = sis8300drv_i2c_byte_write(sisdevice, i2c_reg_addr, (uint32_t)data_bytes[i], &ack, timeout);
            if (status) {
                return status;
            }
            if (!ack) {
                /* NOT ACKNOWLEDGED! try to free the bus, don't care about result - its write_err */
                sis8300drv_i2c_request_send(sisdevice, i2c_reg_addr, SIS8300_I2C_STOP, timeout);
                return status_i2c_nack;
            }
        }
    }

    /* 6. STOP THE I2C TRANSACTION */
    return sis8300drv_i2c_request_send(sisdevice, i2c_reg_addr, SIS8300_I2C_STOP, timeout);
}


/* All writes must be explicitly acknowledged from slave! */
int sis8300drv_i2c_byte_write(sis8300drv_dev *sisdevice, uint32_t i2c_reg_addr,
        uint32_t data_byte, uint32_t *ack, int timeout) {
    int         status;
    uint32_t    ui32_reg_val;

    ui32_reg_val = (data_byte & SIS8300_I2C_DATA_MASK) | SIS8300_I2C_BYTE_WRITE_CYCLE;

    status = sis8300drv_i2c_transfer_and_wait(sisdevice, i2c_reg_addr, &ui32_reg_val, timeout);
    if (status) {
        return status;
    }

    /* check slave acknowledge */
    *ack = (ui32_reg_val & SIS8300_I2C_MASTER_ACK) ? 1 : 0;

    return status_success;
}


/* no limitation on received bytes. But after the final byte is
 * received, the master must not acknowledge the data
 */
int sis8300drv_i2c_byte_read(sis8300drv_dev *sisdevice, uint32_t i2c_reg_addr,
        uint32_t *data_byte, uint32_t ack, int timeout) {
    int         status;
    uint32_t    ui32_reg_val;

    ui32_reg_val = ack ? SIS8300_I2C_MASTER_ACK : 0x0;
    ui32_reg_val |= SIS8300_I2C_BYTE_READ_CYCLE;

    status = sis8300drv_i2c_transfer_and_wait(sisdevice, i2c_reg_addr, &ui32_reg_val, timeout);
    if (status) {
        return status;
    }

    *data_byte = ui32_reg_val & SIS8300_I2C_DATA_MASK;

    return status_success;
}


int sis8300drv_i2c_request_send(sis8300drv_dev *sisdevice, uint32_t i2c_reg_addr, uint32_t val, int timeout) {
    return sis8300drv_i2c_transfer_and_wait(sisdevice, i2c_reg_addr, &val, timeout);
}


int sis8300drv_i2c_transfer_and_wait(sis8300drv_dev *sisdevice, uint32_t i2c_reg_addr, uint32_t *val, int timeout) {
    int status;

    status = sis8300_reg_write(sisdevice->handle, i2c_reg_addr, *val);
    if (status) {
        return status_device_access;
    }

    do {
        status = sis8300_reg_read(sisdevice->handle, i2c_reg_addr, val);
        if (status) {
            /* try to free bus but do not care about retval, cause it;s device acces that's the real problem */
            sis8300_reg_write(sisdevice->handle, i2c_reg_addr, SIS8300_I2C_STOP);
            return status_device_access;
        }

        if (timeout == 0) {
            /* try to free bus, but do not check return, we want to return status busy anyway */
            sis8300_reg_write(sisdevice->handle, i2c_reg_addr, SIS8300_I2C_STOP);
            return status_i2c_busy;
        }

        usleep(1);
        timeout--;

    } while (*val & SIS8300_I2C_LOGIC_BUSY);

    return status_success;
}

