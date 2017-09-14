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
#include <limits.h>
#include <time.h>
#include <math.h>
#include <sys/mman.h>

#include "sis8300drv.h"
#include "sis8300drv_utils.h"
#include "sis8300_reg.h"


int pattern(char *data, unsigned nbytes, int verify) {
    int iter;
    
    for (iter = 0; iter < nbytes; ++iter) {
        if (verify) {
            if (data[iter] != iter % CHAR_MAX) {
                printf("Inconsistency at iter %d value %u should be %u\n", iter, (unsigned)data[iter], (unsigned)(iter % CHAR_MAX));
                return -1;
            }
        } else {
            data[iter] = iter % CHAR_MAX;
        }
    }
    
    return 0;
}

double time_msec() {
    struct timespec ts;
    
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1.0e3 + ts.tv_nsec / 1.0e6;
}


int main(int argc, char **argv) {
    char *data;
    char c;
    unsigned nbytes, nreads, bsize;
    int status, iter, zflag;
    double start, end, speed, avg, sig;
    sis8300drv_usr *sisuser;
    sis8300drv_dev *sisdevice;
    
    bsize = 0x400000;
    nbytes = 1;
    nreads = 1;
    zflag = 0;
    
    while ((c = getopt(argc, argv, "hS:N:Z")) != -1) {
        switch (c) {
            case 'S':
            	nbytes = strtol(optarg, NULL, 10);
                break;
            case 'N':
            	nreads = strtol(optarg, NULL, 10);
                break;
            case 'Z':
                zflag = 1;
                break;
            case ':':
                printf("Option -%c requires an operand.\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device [-h] [-N nbytes] [-V value] \n", argv[0]);
                printf("   \n");
                printf("       -S unsigned int      Number of 4MB blocks to read (default: 1) \n");
                printf("       -N unsigned int      Number of reads to perform (default: 1) \n");
                printf("       -Z                   Use zero copy read \n");
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
    
    nbytes *= bsize;
    
    sisuser = malloc(sizeof(sis8300drv_usr));
    sisuser->file = argv[optind];

    status = sis8300drv_open_device(sisuser);
    if (status) {
        printf("sis8300drv_open_device error: %d\n", status);
        return -1;
    }
    
    sisdevice = (sis8300drv_dev *)sisuser->device;
    
    data = (char *)malloc(nbytes);
    pattern(data, nbytes, 0);
    
    start = time_msec();
    
    status = sis8300drv_write_ram(sisuser, 0, nbytes, data);
    if (status) {
        printf("sis8300drv_write_ram error: %d\n", status);
        return -1;
    }
    
    end = time_msec();
    
    speed = ((double)nbytes)/(end - start) / 1.0e3;
    
    printf("Write speed %lf MB/s\n", speed);
    
    if (zflag) {
        data = (char *)mmap(NULL, nbytes, PROT_READ, MAP_SHARED, sisdevice->handle, 0);
    }
    
    avg = 0;
    sig = 0;
    for (iter = 0; iter < nreads; ++iter) {
        
        if (!zflag) {
            memset(data, 0, nbytes);
        }
    
        start = time_msec();
        
        status = sis8300drv_read_ram(sisuser, 0, nbytes, data);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }
        
        end = time_msec();
        
        pattern(data, nbytes, 1);
        
        speed = ((double)nbytes)/(end - start) / 1.0e3;
        avg += speed;
        sig += speed*speed;
    }
    avg /= nreads;
    sig = sqrt(sig/nreads - avg*avg);
    
    printf("Read speed %lf MB/s +- %lf\n", avg, sig);
    
    if (zflag) {
        munmap((void *)data, nbytes);
    }
    sis8300drv_close_device(sisuser);

    return 0;
}
