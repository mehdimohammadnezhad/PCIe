/**
 * @file sis8300drv_ad9510.h
 * @brief Header for ad9510 clock control chip functions
 */


#ifndef SIS8300DRV_AD9510_H_
#define SIS8300DRV_AD9510_H_


#include "sis8300drv_utils.h"


int sis8300drv_ad9510_spi_get_source(sis8300drv_dev *sisdevice, 
        unsigned *clock_source);

int sis8300drv_ad9510_spi_get_divider(sis8300drv_dev *sisdevice, 
        unsigned *divider, 
        unsigned *bypass);

int sis8300drv_ad9510_spi_setup(sis8300drv_dev *sisdevice,
        unsigned *ch_divider_configuration_array,
        unsigned ad9510_synch_cmd,
        unsigned clock_source);


#endif /* SIS8300DRV_AD9510_H_ */
