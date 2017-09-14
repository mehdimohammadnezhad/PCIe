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
#include <time.h>

#include "sis8300drv.h"

#define FILENAME_BUFFER_SIZE 256


int main(int argc, char **argv) {
    char c;
    int status, verbose, binary, iter;
    unsigned channel, average, min, max;
    FILE *datafile;
    char datafile_base[FILENAME_BUFFER_SIZE];
    char datafile_channel[FILENAME_BUFFER_SIZE];
    uint16_t *data[SIS8300DRV_NUM_AI_CHANNELS];
    sis8300drv_usr *sisuser;
    sis8300drv_clk_src clksrc;
    sis8300drv_clk_div clkdiv;
    sis8300drv_trg_src trgsrc;
    sis8300drv_trg_ext trgext;

    unsigned trgline, trgmask, nsamples, npretrig, channel_mask;

    struct timespec start, end;

    binary = 0;
    verbose = 0;
    nsamples = 256;
    npretrig = 0;
    channel_mask = 0x3ff;
    clksrc = clk_src_internal;
    clkdiv = 2;
    trgsrc = trg_src_soft;
    trgext = trg_ext_harlink;
    trgline = 0;
    trgmask = 0;
    
    while ((c = getopt(argc, argv, "hvba:n:p:t:l:c:d:f:")) != -1) {
        switch (c) {
            case 'a':
            	channel_mask = strtol(optarg, NULL, 16);
                break;
            case 'n':
            	nsamples = strtol(optarg, NULL, 10);
                break;
            case 'p':
            	npretrig = strtol(optarg, NULL, 10);
                break;
            case 't':
            	trgsrc = (sis8300drv_trg_src)strtol(optarg, NULL, 10);
                break;
            case 'l':
            	trgline = strtol(optarg, NULL, 10);
                break;
            case 'c':
            	clksrc = (sis8300drv_clk_src)strtol(optarg, NULL, 10);
                break;
            case 'd':
            	clkdiv = (sis8300drv_clk_div)strtol(optarg, NULL, 10);
                break;
            case 'f':
                sscanf(optarg, "%s", datafile_base);
                break;
            case 'b':
                binary = 1;
                break;
            case 'v':
                verbose = 1;
                break;
            case ':':
                printf("Option -%c requires an operand.\n\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device [-h] [-v] [-b] [-a chmask] [-n nsamples] [-p npretrig] [-t trgsrc] [-l trgline] [-c clksrc] [-d clkdiv] [-f datafile]\n", argv[0]);
                printf("   \n");
                printf("       -a unsigned int      Channel enable mask in HEX (default: 3ff - all ten channels) \n");
                printf("       -n unsigned int      Number of samples (default: 256) \n");
                printf("       -p unsigned int      Number of pretrigger samples (default: 0, max: 2046) \n");
                printf("   \n");
                printf("       -t 0                 Software trigger \n");
                printf("       -t 1                 External trigger \n");
                printf("       -t 2                 Internal trigger \n");
                printf("       -l unsigned int      External trigger line (0-3: frontpanel, 4-11: backplane, default: 0) \n");
                printf("   \n");
                printf("       -c 0                 Internal 250MHz clock oscillator \n");
                printf("       -c 1                 External RTM clock 2 \n");
                printf("       -c 2                 External SMA clock \n");
                printf("       -c 3                 External HARLINK clock \n");
                printf("       -c 4                 Backplane clock A \n");
                printf("       -c 5                 Backplane clock B \n");
                printf("   \n");
                printf("       -d unsigned int      Clock divider (1-18, only even values, value 1 not applicable with internal clock) \n");
                printf("   \n");
                printf("       -f string            Write data to file with the specified base filename (files will be named <datafile>_<channel>.dat) \n");
                printf("       -b                   Write data to files in binary (instead of ascii values) \n");
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
    
    if (verbose) {
        printf("\nStarting acquisition on %s with:\n\n", argv[optind]);
        printf("Channel enable mask             %x\n", channel_mask);
        printf("Number of samples               %u\n", nsamples);
        printf("Number of pretrigger samples    %u\n", npretrig);
        printf("Trigger source                  %u\n", (uint32_t)trgsrc);
        printf("External trigger line           %u\n", trgline);
        printf("Clock source                    %u\n", (uint32_t)clksrc);
        printf("Clock divider                   %u\n\n", (uint32_t)clkdiv);
    }
    
    sisuser = malloc(sizeof(sis8300drv_usr));
    sisuser->file = argv[optind];

    status = sis8300drv_open_device(sisuser);
    if (status) {
        printf("sis8300drv_open_device error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    status = sis8300drv_init_adc(sisuser);
    if (status) {
        printf("sis8300drv_init_adc error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }

    status = sis8300drv_set_nsamples(sisuser, nsamples);
    if (status) {
        printf("sis8300drv_set_nsamples error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }

    status = sis8300drv_set_npretrig(sisuser, npretrig);
    if (status) {
        printf("sis8300drv_set_npretrig error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }

    status = sis8300drv_set_channel_mask(sisuser, channel_mask);
    if (status) {
        printf("sis8300drv_set_channel_mask error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }

    status = sis8300drv_set_clock_source(sisuser, clksrc);
    if (status) {
        printf("sis8300drv_set_clock_source error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }

    status = sis8300drv_set_clock_divider(sisuser, clkdiv);
    if (status) {
        printf("sis8300drv_set_clock_divider error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }

    status = sis8300drv_set_trigger_source(sisuser, trgsrc);
    if (status) {
        printf("sis8300drv_set_trigger_source error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }

    if (trgsrc == trg_src_external) {
        if (trgline >= SIS8300DRV_NUM_FP_TRG) {
            trgext = trg_ext_mlvds;
            trgmask = 1 << (trgline - SIS8300DRV_NUM_FP_TRG);
        }
        status = sis8300drv_set_external_setup(sisuser, trgext, trgmask, 0);
        if (status) {
            printf("sis8300drv_set_external_setup error: %s (%d)\n", 
                    sis8300drv_strerror(status), status);
            return -1;
        }
    }
    /* Measure start time */
    clock_gettime(CLOCK_REALTIME, &start);
    sis8300drv_arm_device(sisuser);

    status = sis8300drv_wait_acq_end(sisuser);
    clock_gettime(CLOCK_REALTIME, &end);
    if (status) {
        printf("sis8300drv_wait_acq_end error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    printf("acquisition completed %fs\n", (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / (double) 1000000000);
    
    if (!verbose && !strlen(datafile_base)) {
        sis8300drv_close_device(sisuser);
        return 0;
    }
    
    if (verbose) {
        printf("\n");
    }

    for (channel = 0; channel < SIS8300DRV_NUM_AI_CHANNELS; channel++) {
        
        data[channel] = (uint16_t *)calloc(nsamples, sizeof(uint16_t));
        if (!(channel_mask & (1 << channel))) {
            continue;
        }
        
        status = sis8300drv_read_ai(sisuser, channel, data[channel]);
        if (status) {
            printf("sis8300drv_read_ai for channel %u error: %s (%d)\n", 
                    channel, sis8300drv_strerror(status), status);
            return -1;
        }
        
        if (strlen(datafile_base)) {
            sprintf(datafile_channel, "%s_%u.dat", datafile_base, channel);
            datafile = fopen(datafile_channel, "wb+");
            if (!datafile) {
                printf("error: could not open/create file %s\n", datafile_channel);
                return -1;
            }
            if (binary) {
                fwrite(data[channel], sizeof(uint16_t), nsamples, datafile);
            } else {
                for (iter = 0; iter < nsamples; iter++) {
                    fprintf(datafile, "%d\n", data[channel][iter]);
                }
            }
            fclose(datafile);
        }
        
        if (verbose) {
            average = 0;
            min = UINT16_MAX;
            max = 0;
            for (iter = 0; iter < nsamples; iter++) {
                average += data[channel][iter];
                if (data[channel][iter] < min) {
                    min = data[channel][iter];
                }
                if (data[channel][iter] > max) {
                    max = data[channel][iter];
                }
            }
            average /= nsamples;
            printf("Channel %u: average=%u min=%u max=%u\n", 
                    channel, average, min, max);
        }
        
        free(data[channel]);
    }
    
    if (verbose) {
        printf("\n");
    }
    
    printf("data readout successful\n");
    
    sis8300drv_close_device(sisuser);

    return 0;
}
