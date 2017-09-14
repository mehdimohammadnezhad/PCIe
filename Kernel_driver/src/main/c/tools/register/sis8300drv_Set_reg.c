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
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

#include "sis8300drv.h"
#include "LLRF_registers.h"

void *llrf_ctrl(void *void_ptr)
{
    long int cn =0;
    sis8300drv_usr  *sisuser= (sis8300drv_usr *)void_ptr ;
    printf("Start trigger. \n");
    while(1)
    {
        usleep(100000);
        Set_pulse_trig(sisuser);
        cn++;
        printf("pulse cn: %d \n",cn);
    }
    return NULL;

}



void Read_all_registers(sis8300drv_usr  *sisuser)
{
    int status;
    unsigned        reg_value, offset;
	offset = 0x400;
    reg_value = 0;

    
    for (int i=0;i<79;i++)
    {
        status = sis8300drv_reg_read(sisuser, offset+i, &reg_value);
        if (status) {
            printf("sis8300drv_reg_read error: %s (%d)\n", 
                    sis8300drv_strerror(status), status);
            return -1;
        }
        
        //printf("0x%x\n", reg_value);
        printf("read from offset 0x%x: 0x%x\n", offset+i, reg_value);
    }
}
void Set_pulse_trig(sis8300drv_usr  *sisuser)
{
    int             status;
    status = sis8300drv_reg_write(sisuser, LLRF_GIP, PULSE_COMMING);
    if (status) {
        printf("sis8300drv_reg_write error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    usleep(300);
    status = sis8300drv_reg_write(sisuser, LLRF_GIP, PULSE_START);
    if (status) {
        printf("sis8300drv_reg_write error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    usleep(3000);
    status = sis8300drv_reg_write(sisuser, LLRF_GIP, PULSE_END);
    if (status) {
        printf("sis8300drv_reg_write error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
}

int main(int argc, char **argv) {
    //char            c;
    int             status, verbose, reg_write;
    pthread_t       t_llrf_ctrl;
	unsigned        reg_value, offset;
	sis8300drv_usr  *sisuser;
	
	verbose = 0;
	reg_value = 0;
	reg_write = 0;


    sisuser = malloc(sizeof(sis8300drv_usr));
    sisuser->file = "/dev/sis8300-4";           //argv[optind];

    status = sis8300drv_open_device(sisuser);
    if (status) {
        printf("sis8300drv_open_device error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    //Read_all_registers(sisuser);

    status = sis8300drv_reg_write(sisuser, 0x403, 0x01);
    if (status) {
        printf("sis8300drv_reg_write error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }

    status = sis8300drv_reg_read(sisuser, LLRF_ID, &reg_value);
    if (status) {
        printf("sis8300drv_reg_read error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    printf("LLRF_ID: 0x%x\n", reg_value);

    
    
    /* create a second thread which executes inc_x(&x) */
    if(pthread_create(&t_llrf_ctrl, NULL, llrf_ctrl, sisuser)) 
    {
    
        fprintf(stderr, "Error creating thread\n");
        return 1;
    
    }


    if(pthread_join(t_llrf_ctrl, NULL)) 
    {
        
        fprintf(stderr, "Error joining thread\n");
        return 2;
        
    }
    

    sis8300drv_close_device(sisuser);

	return(0);
}
