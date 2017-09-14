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

#include "sis8300drv.h"


int main(int argc, char **argv) {
    char c;
    int status, nsamples, offset, binary, pretty, value;
    unsigned nbytes, channel, channel_nsamples, channel_mask, iter;
    uint16_t *data;
    sis8300drv_usr *sisuser;

    binary = 0;
    pretty = 0;
    nsamples = 256;
    offset = 0;
    channel = -1;
    value = -1;
    
    while ((c = getopt(argc, argv, "hbpn:o:w:")) != -1) {
        switch (c) {
            case 'n':
            	nsamples = strtol(optarg, NULL, 10);
                break;
            case 'o':
            	offset = strtol(optarg, NULL, 10);
                break;
            case 'w':
                value = strtol(optarg, NULL, 10);
                break;
            case 'b':
                binary = 1;
                break;
            case 'p':
                pretty = 1;
                break;
            case ':':
                printf("Option -%c requires an operand.\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device [-h] [-b] [-p] [-n nsamples] [-o offset] [-w value] \n", argv[0]);
                printf("   \n");
                printf("       -n int       Number of samples to read/write (default: 256, must be multiple of 16), \n");
                printf("                         a zero or negative value means to read/write the amount of samples currently set on the board \n");
                printf("       -o int       Offset (in number of samples) to read/write from/to (default: 0, must be multiple of 16), \n");
                printf("                         a negative value means to read/write from/to the address of a specific channel, \n");
                printf("                         the channel number is the negated offset (-1 - channel 1, ... , -10 - channel 10) \n");
                printf("       -w int       Value to write to all memory locations \n");
                printf("       -b           Output read values in binary \n");
                printf("       -p           Pretty print read values \n");
                printf("   \n");
                printf("       -h           Print this message \n");
                printf("   \n");
                return -1;
        }
    }
    
    if (optind != argc - 1) {
        printf ("Device argument required.\n");
        return -1;
    }
    
    if (nsamples > 0 && nsamples % SIS8300DRV_BLOCK_SAMPLES) {
        printf("error: positive nsamples must be a multiple of %d\n", 
                SIS8300DRV_BLOCK_SAMPLES);
        return -1;
    }
    
    if (offset >= 0 && offset % SIS8300DRV_BLOCK_SAMPLES) {
        printf("error: non-negative offset must be a multiple of %d\n", 
                SIS8300DRV_BLOCK_SAMPLES);
        return -1;
    }
    
    if (offset < -10) {
        printf("error: negative offset must be a in the range [-10,-1]\n");
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
    
    status = sis8300drv_get_nsamples(sisuser, &channel_nsamples);
    if (status) {
        printf("sis8300drv_get_nsamples error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    if (nsamples <= 0) {
        nsamples = channel_nsamples;
    }
    
    if (offset < 0) {
        channel = -(offset + 1);
        
        status = sis8300drv_get_channel_mask(sisuser, &channel_mask);
        if (status) {
            printf("sis8300drv_get_channel_mask error: %s (%d)\n", 
                    sis8300drv_strerror(status), status);
            return -1;
        }

        if (!(channel_mask & (1 << channel))) {
            printf("error: channel %u not enabled\n", channel);
            return -1;
        }
        
        offset = 0;
        for (iter = 0; iter < channel; iter++) {
            if (channel_mask & (1 << iter)) {
                offset += channel_nsamples;
            }
        }
    }
    
    offset *= sizeof(uint16_t);
    nbytes = nsamples * sizeof(uint16_t);
    data = (uint16_t *)malloc(nbytes);
    
    if (value < 0) {
        /* Read data from device memory. */
        status = sis8300drv_read_ram(sisuser, offset, nbytes, data);
        if (status) {
            printf("sis8300drv_read_ram error: %s (%d)\n", 
                    sis8300drv_strerror(status), status);
            return -1;
        }
        if (pretty) {
            printf("\n%d samples from offset %d\n", nsamples, offset);
        }
        for (iter = 0; iter < nsamples; iter++) {
            if (binary) {
                fwrite(data, sizeof(uint16_t), nsamples, stdout);
            } else if (pretty) {
                if (!(iter % SIS8300DRV_BLOCK_SAMPLES)) {
                    printf("\n");
                }
                printf("%d ", data[iter]);
            } else {
                printf("%d\n", data[iter]);
            }
        }
        if (pretty) {
            printf("\n");
        }
    } else {
        /* Write data to device memory. */
        for (iter = 0; iter < nsamples; iter++) {
            data[iter] = value;
        }
        status = sis8300drv_write_ram(sisuser, offset, nbytes, data);
        if (status) {
            printf("sis8300drv_write_ram error: %s (%d)\n", 
                    sis8300drv_strerror(status), status);
            return -1;
        }
        printf("wrote %d samples (value %d) to offset %d\n", 
                nsamples, value, offset);
    }
    
    free(data);

    sis8300drv_close_device(sisuser);

    return 0;
}
