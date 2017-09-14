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
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/mman.h>

#include "sis8300drv.h"
#include "sis8300drv_utils.h"


int main(int argc, char **argv) {
    char c, value, *wdata, *rdata;
    unsigned nbytes, offset;
    int status, iter, pflag, wflag;
    sis8300drv_usr *sisuser;
    sis8300drv_dev *sisdevice;
    
    nbytes = 256;
    value = 0;
    offset = 0;
    pflag = 0;
    wflag = 0;
    
    while ((c = getopt(argc, argv, "hN:O:V:P")) != -1) {
        switch (c) {
            case 'N':
            	nbytes = strtol(optarg, NULL, 10);
                break;
            case 'O':
            	offset = strtol(optarg, NULL, 10);
                break;
            case 'V':
                wflag = 1;
                sscanf(optarg, "%c", &value);
                break;
            case 'P':
                pflag = 1;
                break;
            case ':':
                printf("Option -%c requires an operand.\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device [-h] [-N nbytes] [-O offset] [-V value] \n", argv[0]);
                printf("   \n");
                printf("       -N unsigned int      Number of bytes to read (default: 256, must be multiple of 32) \n");
                printf("       -O unsigned int      Offset to read from (default: 0, must be multiple of 32) \n");
                printf("       -V unsigned char     Value to write (default: no write performed) \n");
                printf("       -P                   Print readback values \n");
                printf("   \n");
                printf("       -h                   Print this message \n");
                printf("   \n");
                return -1;
        }
    }
    
    if (optind != argc - 1) {
        printf ("Device argument required.\n");
        return -1;
    }
    
    sisuser = malloc(sizeof(sis8300drv_usr));
    sisuser->file = argv[optind];

    status = sis8300drv_open_device(sisuser);
    if (status) {
        printf("sis8300drv_open_device error: %d\n", status);
        return -1;
    }
    
    sisdevice = (sis8300drv_dev *)sisuser->device;
        
    if (wflag) {
        wdata = (char *)malloc(nbytes);
        memset(wdata, value, nbytes);
        status = sis8300drv_write_ram(sisuser, offset, nbytes, wdata);
        if (status) {
            printf("sis8300drv_write_ram error: %d\n", status);
            return -1;
        }
        free(wdata);
    }
    
    rdata = (char *)mmap(NULL, nbytes, PROT_READ, MAP_SHARED, sisdevice->handle, 0);
    if (rdata == MAP_FAILED) {
        printf("mmap failed\n");
        sis8300drv_close_device(sisuser);
    }
    
    status = sis8300drv_read_ram(sisuser, offset, nbytes, rdata);
    if (status) {
        printf("sis8300drv_read_ram error: %d\n", status);
        return -1;
    }
    
    if (pflag) {
        for (iter = 0; iter < nbytes; iter++) {
            if (iter && !(iter % 16)) {
                printf("\n");
            }
            printf("%c ", rdata[iter]);
        }
        printf("\n");
    }
    
    munmap((void *)rdata, nbytes);
    sis8300drv_close_device(sisuser);

    return 0;
}
