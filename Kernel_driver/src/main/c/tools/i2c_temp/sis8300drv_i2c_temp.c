/**
 * Struck 8300 Linux userspace library.
 * Copyright (C) 2016 ESS
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
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sis8300drv.h"

int main(int argc, char **argv) {
	sis8300drv_usr *sisuser;
    sis8300drv_rtm rtm_type;
    int status;
    double v;

    if (argc != 3) {
        printf("Usage: %s [device_node] [rtm_type]\n", argv[0]);
        printf("RTM Type is \n\trtm_sis8900 = 0\n\trtm_dwc8vm1 = 1\n\trtm_ds8vm1 = 2\n");
        return -1;
    }
    
    rtm_type = (sis8300drv_rtm)strtoul(argv[2], NULL, 10);
    if (rtm_type != rtm_dwc8vm1) {
    	printf("This RTM type does not support temperature read-out\n");
    	return -1;
    }

    sisuser = malloc(sizeof(sis8300drv_usr));
    sisuser->file = argv[1];

    status = sis8300drv_open_device(sisuser);
    if (status) {
        printf("sis8300drv_open_device error: %d\n", status);
        free(sisuser);
        return -1;
    }

	printf("Reading AD8363 temperature..\n");
	status = sis8300drv_i2c_rtm_temperature_get(sisuser, rtm_type, rtm_temp_ad8363, &v);
	if (status) {
		printf("Fail %i\n", status);
	} else {
		printf("Temperature of AD8363 %f\n", v);
	}

	usleep(200000);

	printf("Reading LTC2493 temperature..\n");
	status = sis8300drv_i2c_rtm_temperature_get(sisuser, rtm_type, rtm_temp_ltc2493, &v);
	if (status) {
		printf("Fail %i\n", status);
	} else {
		printf("Temperature of LTC2493 %f\n", v);
	}

    sis8300drv_close_device(sisuser);
    free(sisuser);
    
    return 0;
}
