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
 * @brief Implementation ad9510 clock control chip functions
 * @author kstrnisa
 */


#include <stdint.h>
#include <unistd.h>

#include "sis8300_reg.h"
#include "sis8300_defs.h"
#include "sis8300drv.h"
#include "sis8300drv_ad9510.h"
#include "sis8300drv_utils.h"


#define SIS8300DRV_AD9510_SPI_WRITE(value)                                          \
    do {                                                                            \
    status = sis8300_reg_write(sisdevice->handle, SIS8300_AD9510_SPI_REG, (value)); \
    if (status) {                                                                   \
        return status_device_access;                                                \
    }                                                                               \
    usleep(1);                                                                      \
    } while (0);
    
    
#define SIS8300DRV_AD9510_SPI_READ(value)                                           \
    do {                                                                            \
    status = sis8300_reg_read(sisdevice->handle, SIS8300_AD9510_SPI_REG, (value));  \
    if (status) {                                                                   \
        return status_device_access;                                                \
    }                                                                               \
    } while (0);


/**
 * @brief Read the parameters of the clock distribution IC AD9510 to determine
 * the clock source of the chip, either the device muxes or the RTM clk0 and
 * clk1 clock lines.
 * @param [in] sisuser User context struct.
 * @param [out] clock_source AD9510 clock source 
 * (0 - mux_D and mux_E, 1 - RTM_0 and RTM_1).
 *
 * @retval status_success Initialization successful..
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * The AD9510 clock distribution chip has two possible clock sources. One is
 * from the device muxes that multiplex between:
 *  - internal oscilator
 *  - RTM clk2 clock line
 *  - SMA on frontpanel
 *  - Harlink on frontpanel
 *  - Backplane line A
 *  - Backplane line B
 * 
 * The other one is from the RTM clk0 and clk1 clock lines. If this option is
 * chosen the first AD9510 chip (clock for ADC 1, 2 and 3) is clocked by RTM clk0
 * and the second AD9510 chip (clock for ADC 3 and 4) is clocked by RTM clk1.
 * God knows what happens if those clock are too different.
 * 
 * This function assumes that the device lock is held by the caller and that 
 * the device is not armed.
 */
int sis8300drv_ad9510_spi_get_source(sis8300drv_dev *sisdevice, unsigned *clock_source) {
    int             status;
    uint32_t        ui32_reg_val;
    
    /* Register 0x45 of the AD9510 chip holds the information which of the two
     * clock inputs is enabled. */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_READ_CYCLE + 0x4500;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    ui32_reg_val = 0xFFFFFFFF;
    SIS8300DRV_AD9510_SPI_READ(&ui32_reg_val);
    *clock_source = (unsigned)(~ui32_reg_val & 0x1);
    
    return status_success;
}


/**
 * @brief Read the parameters of the clock distribution IC AD9510 to determine
 * the clock divider of the chip.
 * @param [in] sisuser User context struct.
 * @param [out] divider Clock divider of the AD951 chip.
 * @param [out] bypass Bypass parameter of the AD951 chip.
 *
 * @retval status_success Initialization successful..
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 *
 * The AD9510 has a minimum clock divider of 2. So if one wants to choose
 * a divider of 1 then the divider circuit has to be bypassed. That is
 * the meaning of the #bypass parameter. 
 *
 * The AD9510 has a separate clock divider settings for each separate ADC.
 * The clock divider is determined by reading only thre divider for ADC 1 since
 * it is always set to the same value for all ADCs.
 * 
 * This function assumes that the device lock is held by the caller and that 
 * the device is not armed.
 */
int sis8300drv_ad9510_spi_get_divider(sis8300drv_dev *sisdevice, unsigned *divider, unsigned *bypass) {
    int             status;
    uint32_t        ui32_reg_val;
    
    /* Register 0x56 of the AD9510 chip holds the information about the clock
     * divider for ADC 1. */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_READ_CYCLE + 0x5600;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    ui32_reg_val = 0xFFFFFFFF;
    SIS8300DRV_AD9510_SPI_READ(&ui32_reg_val);
    *divider = (unsigned)ui32_reg_val;

    /* Register 0x57 of the AD9510 chip holds the information about the clock
     * divider bypass setting for ADC 1. */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_READ_CYCLE + 0x5700;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    ui32_reg_val = 0xFFFFFFFF;
    SIS8300DRV_AD9510_SPI_READ(&ui32_reg_val);
    *bypass = (unsigned)((ui32_reg_val & 0x80) << 8);
    
    return status_success;
}


/**
 * @brief Configure the parameters of the clock distribution IC AD9510.
 * @param [in] sisuser User context struct.
 * @param [in] ch_divider_configuration_array Configuration array.
 * @param [in] ad9510_synch_cmd Sync command.
 * @param [in] clock_source AD9510 clock source (0 - mux_D and mux_E, 1 - RTM_0 and RTM_0).
 *
 * @retval status_success Initialization successful.
 * @retval status_argument_invalid Invalid device type in context struct.
 * @retval status_device_access Can't access device registers.
 *
 * Regarding the #clock_source parameter. The AD9510 clock distribution chip 
 * has two possible clock sources. One is #SIS8300DRV_CLKSRC_AD9510_MUX and
 * sets the closk source to the device muxes that multiplex between:
 *  - internal oscilator
 *  - RTM clk2 clock line
 *  - SMA on frontpanel
 *  - Harlink on frontpanel
 *  - Backplane line A
 *  - Backplane line B
 * 
 * The other one is #SIS8300DRV_CLKSRC_AD9510_RTM and sets the clock source to 
 * the RTM clk0 and clk1 clock lines. If this option is chosen the 
 * first AD9510 chip (clock for ADC 1, 2 and 3) is clocked by RTM clk0
 * and the second AD9510 chip (clock for ADC 3 and 4) is clocked by RTM clk1.
 * God knows what happens if those clock are too different.
 *
 * regarding the #ch_divider_configuration_array parameter. This is an 
 * unsigned array of size 8 and controls the clock divider settings of the 8
 * outputs of both AD9510 chips (4 outputs each). only 16 bits of each array
 * elements are relevant. For details look at the description below and the
 * AD9510 documentation.
 *
 * Vendor information about configuration input parameters:
 *
 * Out 4  (FPGA DIV-CLK05) : LVDS 3.5 mA
 * Out 5  (ADC3-CLK, ch4/5) : LVDS 3.5 mA
 * Out 6  (ADC2-CLK, ch3/4) : LVDS 3.5 mA
 * Out 7  (ADC1-CLK, ch1/2) : LVDS 3.5 mA
 *
 * Out 4  (FPGA DIV-CLK69) : LVDS 3.5 mA
 * Out 5  (ADC5-CLK, ch8/9) : LVDS 3.5 mA
 * Out 6  (ADC4-CLK, ch6/7) : LVDS 3.5 mA
 * Out 7  (Frontpanel Clk, Harlink) : LVDS 3.5 mA
 *
 * ch_divider_configuration_array[i] :
 * bits <3:0>:   Divider High
 * bits <7:4>:   Divider Low
 * bits <11:8>:  Phase Offset
 * bit  <12>:    Select Start High
 * bit  <13>:    Force
 * bit  <14>:    Nosyn (individual)
 * bit  <15>:    Bypass Divider

 * i=0:    AD910 #1 Out 7  (ADC1-CLK, ch1/2)
 * i=1:    AD910 #1 Out 6  (ADC2-CLK, ch3/4)
 * i=2:    AD910 #1 Out 5  (ADC3-CLK, ch4/5)
 * i=3:    AD910 #2 Out 6  (ADC4-CLK, ch6/7)
 * i=4:    AD910 #2 Out 5  (ADC5-CLK, ch8/9)
 * i=5:    AD910 #2 Out 7  (Frontpanel Clk, Harlink)
 * i=6:    AD910 #1 Out 4  (FPGA DIV-CLK05) used for synch. of external Triggers
 * i=7:    AD910 #2 Out 4  (FPGA DIV-CLK69) used for sychn. of AD910 ISc
 * 
 * This function assumes that the device lock is held by the caller and that 
 * the device is not armed.
 */
int sis8300drv_ad9510_spi_setup(sis8300drv_dev *sisdevice,
        unsigned *ch_divider_configuration_array,
        unsigned ad9510_synch_cmd,
        unsigned clock_source) {
    int             status;
    uint32_t        ui32_reg_val;
    
    switch (sisdevice->type) {
        case SIS8300_SIS8300L:
            break;
        case SIS8300_SIS8300:
            break;
        default:
            return status_argument_invalid;
    }
    
    
    /**************************************************************************/
    /* AD9510 #1 (ADC channels 1 to 6)                                        */

    /* set AD9510 to Bidirectional Mode and Soft Reset
     * on 8300 module the bus uses SDIO, on 8300L the bus uses seperate SDI SDO */
    switch (sisdevice->type) {
        case SIS8300_SIS8300L:
            ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x0030;
            break;
        case SIS8300_SIS8300:
            ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x00B0;
            break;
        default:
            break;
    }
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    /* set AD9510 out of reset
     * set AD9510 to Bidirectional Mode */
    switch (sisdevice->type) {
        case SIS8300_SIS8300L:
            ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x0010;
            break;
        case SIS8300_SIS8300:
            ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x0090;
            break;
        default:
            break;
    }
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    

    /* default: Asychrnon PowerDown, no Prescaler */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x0A01;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 0 (not used) : total Power-Down */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x3C0B;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 1 (not used) : total Power-Down */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x3D0B;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 2 (not used) : total Power-Down */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x3E0B;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 3 (not used) : total Power-Down */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x3F0B;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 4  (FPGA DIV-CLK05) : LVDS 3.5 mA */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x4002;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 5  (ADC3-CLK, ch4/5) : LVDS 3.5 mA */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x4102;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 6  (ADC2-CLK, ch3/4) : LVDS 3.5 mA */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x4202;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 7  (ADC1-CLK, ch1/2) : LVDS 3.5 mA */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x4302;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x4500;
    ui32_reg_val += 0x10; /* Power-Down RefIn */
    ui32_reg_val += 0x08; /* Shut Down Clk to PLL Prescaler */
    if (clock_source == 1) {
        /* Clocks from RTM */
        ui32_reg_val += 0x02; /* Power-Down CLK1 (from mux_E) */
        ui32_reg_val += 0x00; /* CLK2 Drives Distribution Sect (from RTM_0) */
    } else {
        /* Clocks from muxes on device */
        ui32_reg_val += 0x04; /* Power-Down CLK2 (from RTM_0) */
        ui32_reg_val += 0x01; /* CLK1 Drives Distribution Sect (from mux_E) */
    }
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);


    /* Out 4  (FPGA DIV-CLK05) : Divider Low/High */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5000;
    ui32_reg_val += ch_divider_configuration_array[6] & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 4  (FPGA DIV-CLK05) : Bypasse Diver (7), No Sychn (6), Force Individual Start (5), Start High (4), Phase Offset (3:0) */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5100;
    ui32_reg_val += (ch_divider_configuration_array[6] >> 8) & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 5  (ADC3-CLK, ch4/5) : Divider Low/High */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5200;
    ui32_reg_val += ch_divider_configuration_array[2] & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 5  (ADC3-CLK, ch4/5) : Bypasse Diver (7), No Sychn (6), Force Individual Start (5), Start High (4), Phase Offset (3:0) */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5300;
    ui32_reg_val += (ch_divider_configuration_array[2] >> 8) & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 6  (ADC2-CLK, ch2/3) : Divider Low/High */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5400;
    ui32_reg_val += ch_divider_configuration_array[1] & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 6  (ADC2-CLK, ch2/3) : Bypasse Diver (7), No Sychn (6), Force Individual Start (5), Start High (4), Phase Offset (3:0) */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5500;
    ui32_reg_val += (ch_divider_configuration_array[1] >> 8) & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 7  (ADC1-CLK, ch1/2) : Divider Low/High */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5600;
    ui32_reg_val += ch_divider_configuration_array[0] & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 7  (ADC1-CLK, ch1/2) : Bypasse Diver (7), No Sychn (6), Force Individual Start (5), Start High (4), Phase Offset (3:0) */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5700;
    ui32_reg_val += (ch_divider_configuration_array[0] >> 8) & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* update command */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5A01;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    
    /**************************************************************************/
    /* AD9510 #2 (ADC channels 7 to 10)                                       */
    
    /* set AD9510 to Bidirectional Mode and Soft Reset */
    switch (sisdevice->type) {
        case SIS8300_SIS8300L:
            ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x0030;
            break;
        case SIS8300_SIS8300:
            ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x00B0;
            break;
        default:
            break;
    }
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    /* set AD9510 out of reset
     * set AD9510 to Bidirectional Mode */
    switch (sisdevice->type) {
        case SIS8300_SIS8300L:
            ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x0010;
            break;
        case SIS8300_SIS8300:
            ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x0090;
            break;
        default:
            break;
    }
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    /* default: Asychrnon PowerDown, no Prescaler */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x0A01;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    
    /* Out 0 (not used) : total Power-Down */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x3C0B;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 1 (not used) : total Power-Down */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x3D0B;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 2 (not used) : total Power-Down */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x3E0B;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 3 (not used) : total Power-Down */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x3F0B;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 4  (FPGA DIV-CLK69) : LVDS 3.5 mA */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x4002;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 5  (ADC5-CLK, ch8/9) : LVDS 3.5 mA */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x4102;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 6  (ADC4-CLK, ch6/7) : LVDS 3.5 mA */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x4202;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 7  (Frontpanel Clk, Harlink) : LVDS 3.5 mA */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x4302;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x4500;
    ui32_reg_val += 0x10 ; /* Power-Down RefIn */
    ui32_reg_val += 0x08 ; /* Shut Down Clk to PLL Prescaler */
        if (clock_source == 1) {
        /* Clocks from RTM */
        ui32_reg_val += 0x02; /* Power-Down CLK1 (from mux_D) */
        ui32_reg_val += 0x00; /* CLK2 Drives Distribution Sect (from RTM_1) */
    } else {
        /* Clocks from muxes on device */
        ui32_reg_val += 0x04; /* Power-Down CLK2 (from RTM_1) */
        ui32_reg_val += 0x01; /* CLK1 Drives Distribution Sect (from mux_D) */
    }
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    
    
    /* Out 4  (FPGA DIV-CLK69) : Divider Low/High */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5000;
    ui32_reg_val += ch_divider_configuration_array[7] & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 4  (FPGA DIV-CLK69) : Bypasse Diver (7), No Sychn (6), Force Individual Start (5), Start High (4), Phase Offset (3:0) */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5100;
    ui32_reg_val += (ch_divider_configuration_array[7] >> 8) & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 5  (ADC5-CLK, ch8/9) : Divider Low/High */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5200;
    ui32_reg_val += ch_divider_configuration_array[4] & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 5  (ADC5-CLK, ch8/9) : Bypasse Diver (7), No Sychn (6), Force Individual Start (5), Start High (4), Phase Offset (3:0) */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5300;
    ui32_reg_val += (ch_divider_configuration_array[4] >> 8) & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 6  (ADC4-CLK, ch6/7) : Divider Low/High */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5400;
    ui32_reg_val += ch_divider_configuration_array[3] & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 6  (ADC4-CLK, ch6/7) : Bypasse Diver (7), No Sychn (6), Force Individual Start (5), Start High (4), Phase Offset (3:0) */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5500;
    ui32_reg_val += (ch_divider_configuration_array[3] >> 8) & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 7  (Frontpanel Clk, Harlink) : Divider Low/High */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5600;
    ui32_reg_val += ch_divider_configuration_array[5] & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* Out 7  (Frontpanel Clk, Harlink) : Bypasse Diver (7), No Sychn (6), Force Individual Start (5), Start High (4), Phase Offset (3:0) */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5700;
    ui32_reg_val += (ch_divider_configuration_array[5] >> 8) & 0xff;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

    /* update command */
    ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5A01;
    SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);


    /* synch Cmd */
    if (ad9510_synch_cmd == 1) {
        /* set Function of "Function pin" to SYNCB (Default Reset) */
        ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5822;
        SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

        /* update command */
        ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + 0x5A01;
        SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

        /* set Function of "Function pin" to SYNCB (Default Reset) */
        ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5822;
        SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

        /* update command */
        ui32_reg_val = AD9510_GENERATE_SPI_RW_CMD + AD9510_SPI_SELECT_NO2 + 0x5A01;
        SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

        /* set use "FPGA DIV-CLK69" for Sychn Pulse  - Logic (VIRTEX5) */
        ui32_reg_val = AD9510_SPI_SET_FUNCTION_SYNCH_FPGA_CLK69;
        SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);

        /* generate sych Pulse   (pulse Function pins) and hold FPGA_CLK69 to sycnh pulse */
        ui32_reg_val = AD9510_GENERATE_FUNCTION_PULSE_CMD + AD9510_SPI_SET_FUNCTION_SYNCH_FPGA_CLK69;
        SIS8300DRV_AD9510_SPI_WRITE(ui32_reg_val);
    }

    return status_success;
}

