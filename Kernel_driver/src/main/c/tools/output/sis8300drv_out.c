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
#include <stdint.h>
#include <string.h>

#include "sis8300drv.h"


int main(int argc, char **argv) {
    char c;
    int status, verbose, write;
    unsigned channel;
    double value;
    sis8300drv_usr *sisuser;
    
    write = 0;
    verbose = 0;
    value = 0;
    channel = 0;
    
    while ((c = getopt(argc, argv, "hva:w:")) != -1) {
        switch (c) {
            case 'a':
            	channel = strtol(optarg, NULL, 10);
                break;
            case 'w':
            	value = strtold(optarg, NULL);
                write = 1;
                break;
            case 'v':
                verbose = 1;
                break;
            case ':':
                printf("Option -%c requires an operand.\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device [-h] [-a channel] [-w value] \n", argv[0]);
                printf("   \n");
                printf("       -a unsigned int      Analog output channel to output on (0 or 1, default: 0) \n");
                printf("       -w double            Write value to analog output\n");
                printf("   \n");
                printf("       -v                   Verbose output \n");
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
        printf("sis8300drv_open_device error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    if (write) {
        if (verbose) {
            printf("set value on channel %d: %lf\n", channel, value);
        }
        
        status = sis8300drv_write_ao(sisuser, channel, value);
        if (status) {
            printf("sis8300drv_write_ao error: %s (%d)\n", 
                    sis8300drv_strerror(status), status);
            return -1;
        }
    }
    
    value = 0;
    status = sis8300drv_read_ao(sisuser, channel, &value);
    if (status) {
        printf("sis8300drv_read_ao error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    if (!write && !verbose) {
        printf("%lf\n", value);
    }
    
    if (verbose) {
        printf("read from channel %d: %lf\n", channel, value);
    }
    
    sis8300drv_close_device(sisuser);

    return 0;
}
