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
#include <sys/mman.h>

#include <xmmintrin.h>
#include <math.h>

#include "sis8300drv.h"
#include "sis8300drv_utils.h"
#include "sis8300_reg.h"
#include "sis8300_defs.h"


#define ROUNDUP_HEX(val) (((unsigned)(val) + 0xF) & ~0xF)


double time_msec() {
    struct timespec ts;
    
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1.0e3 + ts.tv_nsec / 1.0e6;
}


int readch_orig(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads,
        unsigned dec_offset,
        unsigned dec_factor,
        double scale_offset,
        double scale_factor) {

    int status, iter, iter_src, iter_dest, iter_scale;
    unsigned nsamples_dec;
    double start, end, speed, timech;
    uint16_t *data_raw;
    volatile float *data;

    nsamples_dec = nsamples - dec_offset;
    nsamples_dec /= dec_factor;

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        data = (volatile float *)malloc(sizeof(float) * nsamples);

        data_raw = (uint16_t *)(data + nsamples);
        data_raw -= nsamples;

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_dest = 0, iter_src = dec_offset; iter_dest < nsamples_dec; iter_dest++, iter_src += dec_factor) {
            data[iter_dest] = (float)data_raw[iter_src];
        }

        for (iter_scale = 0; iter_scale < nsamples_dec; iter_scale++) {
            data[iter_scale] = data[iter_scale] * scale_factor + scale_offset;
        }

        free((void *)data);

    }

    end = time_msec();

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_orig  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_norealloc(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads,
        unsigned dec_offset,
        unsigned dec_factor,
        double scale_offset,
        double scale_factor) {

    int status, iter, iter_src, iter_dest, iter_scale;
    unsigned nsamples_dec;
    double start, end, speed, timech;
    uint16_t *data_raw;
    volatile float *data;

    nsamples_dec = nsamples - dec_offset;
    nsamples_dec /= dec_factor;

    data = (volatile float *)malloc(sizeof(float) * nsamples);

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        data_raw = (uint16_t *)(data + nsamples);
        data_raw -= nsamples;

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_dest = 0, iter_src = dec_offset; iter_dest < nsamples_dec; iter_dest++, iter_src += dec_factor) {
            data[iter_dest] = (float)data_raw[iter_src];
        }

        for (iter_scale = 0; iter_scale < nsamples_dec; iter_scale++) {
            data[iter_scale] = data[iter_scale] * scale_factor + scale_offset;
        }

    }

    end = time_msec();

    free((void *)data);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_norealloc  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_separatebuf(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads,
        unsigned dec_offset,
        unsigned dec_factor,
        double scale_offset,
        double scale_factor) {

    int status, iter, iter_scale;
    unsigned nsamples_dec;
    double start, end, speed, timech;
    uint16_t *data_raw;
    volatile float *data;

    nsamples_dec = nsamples - dec_offset;
    nsamples_dec /= dec_factor;

    data_raw = (uint16_t *)malloc(sizeof(uint16_t) * nsamples);
    data = (volatile float *)malloc(sizeof(float) * nsamples_dec);

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_scale = 0; iter_scale < nsamples_dec; iter_scale++) {
            data[iter_scale] = data_raw[iter_scale * dec_factor + dec_offset] * scale_factor + scale_offset;
        }

    }

    end = time_msec();

    free((void *)data);
    free((void *)data_raw);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_separatebuf  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_nodec(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads,
        double scale_offset,
        double scale_factor) {

    int status, iter, iter_scale;
    unsigned nsamples_dec;
    double start, end, speed, timech;
    uint16_t *data_raw;
    volatile float *data;

    nsamples_dec = nsamples;

    data_raw = (uint16_t *)malloc(sizeof(uint16_t) * nsamples);
    data = (volatile float *)malloc(sizeof(float) * nsamples_dec);

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_scale = 0; iter_scale < nsamples_dec; iter_scale++) {
            data[iter_scale] = data_raw[iter_scale] * scale_factor + scale_offset;
        }

    }

    end = time_msec();

    free((void *)data);
    free((void *)data_raw);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_nodec  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_noscale(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads,
        unsigned dec_offset,
        unsigned dec_factor) {

    int status, iter, iter_scale;
    unsigned nsamples_dec;
    double start, end, speed, timech;
    uint16_t *data_raw;
    volatile float *data;

    nsamples_dec = nsamples - dec_offset;
    nsamples_dec /= dec_factor;

    data_raw = (uint16_t *)malloc(sizeof(uint16_t) * nsamples);
    data = (volatile float *)malloc(sizeof(float) * nsamples_dec);

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_scale = 0; iter_scale < nsamples_dec; iter_scale++) {
            data[iter_scale] = data_raw[iter_scale * dec_factor + dec_offset];
        }

    }

    end = time_msec();

    free((void *)data);
    free((void *)data_raw);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_noscale  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_nodecscale(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads) {

    int status, iter, iter_scale;
    unsigned nsamples_dec;
    double start, end, speed, timech;
    uint16_t *data_raw;
    volatile float *data;

    nsamples_dec = nsamples;

    data_raw = (uint16_t *)malloc(sizeof(uint16_t) * nsamples);
    data = (volatile float *)malloc(sizeof(float) * nsamples_dec);

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_scale = 0; iter_scale < nsamples_dec; iter_scale++) {
            data[iter_scale] = data_raw[iter_scale];
        }

    }

    end = time_msec();

    free((void *)data);
    free((void *)data_raw);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_nodecscale  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


#define NSAMPLES 0x50000
uint16_t DATA_RAW[NSAMPLES];
volatile float DATA[NSAMPLES];

int readch_nodecscale_fixed(sis8300drv_usr *sisuser,
     unsigned nreads) {

    int status, iter, iter_scale;
    double start, end, speed, timech;

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * NSAMPLES, DATA_RAW);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_scale = 0; iter_scale < NSAMPLES; iter_scale++) {
            DATA[iter_scale] = DATA_RAW[iter_scale];
        }

    }

    end = time_msec();

    speed = ((double)(sizeof(uint16_t) * NSAMPLES))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_nodecscale_fixed  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_intrinsic(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads,
        unsigned dec_offset,
        unsigned dec_factor,
        double scale_offset,
        double scale_factor) {

    int status, iter, iter_scale;
    double start, end, speed, timech;
    unsigned nsamples_dec;
    uint16_t *data_raw, *data_raw_dec;
    __m64 *data_raw_dec_intrinsic;
    volatile __m128 *data;
    __m128 scale_offset_intrinsic;
    __m128 scale_factor_intrinsic;

    nsamples_dec = nsamples - dec_offset;
    nsamples_dec /= dec_factor;

    data_raw = (uint16_t *)malloc(sizeof(uint16_t) * nsamples);
    data_raw_dec = (uint16_t *)malloc(sizeof(uint16_t) * nsamples_dec);
    data_raw_dec_intrinsic = (__m64 *)data_raw_dec;
    data = (volatile __m128 *)malloc(sizeof(float) * nsamples_dec);

    for (iter = 0; iter < 4; iter++) {
        *((float *)(&(scale_offset_intrinsic)) + iter) = (float)scale_offset;
        *((float *)(&(scale_factor_intrinsic)) + iter) = (float)scale_factor;
    }

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_scale = 0; iter_scale < nsamples_dec; iter_scale++) {
            data_raw_dec[iter_scale] = data_raw[iter_scale * dec_factor + dec_offset];
        }

        for (iter_scale = 0; iter_scale < nsamples_dec / 4; iter_scale++) {
            data[iter_scale] = _mm_cvtpu16_ps(data_raw_dec_intrinsic[iter_scale]);
            data[iter_scale] = _mm_mul_ps(data[iter_scale], scale_factor_intrinsic);
            data[iter_scale] = _mm_add_ps(data[iter_scale], scale_offset_intrinsic);
        }

    }

    end = time_msec();

    free((void *)data);
    free((void *)data_raw_dec);
    free((void *)data_raw);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_intrinsic  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_nodec_intrinsic(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads,
        double scale_offset,
        double scale_factor) {

    int status, iter, iter_scale;
    double start, end, speed, timech;
    __m64 *data_raw;
    volatile __m128 *data;
    __m128 scale_offset_intrinsic;
    __m128 scale_factor_intrinsic;

    data_raw = (__m64 *)malloc(sizeof(uint16_t) * nsamples);
    data = (volatile __m128 *)malloc(sizeof(float) * nsamples);

    for (iter = 0; iter < 4; iter++) {
        *((float *)(&(scale_offset_intrinsic)) + iter) = (float)scale_offset;
        *((float *)(&(scale_factor_intrinsic)) + iter) = (float)scale_factor;
    }

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_scale = 0; iter_scale < nsamples / 4; iter_scale++) {
            data[iter_scale] = _mm_cvtpu16_ps(data_raw[iter_scale]);
            data[iter_scale] = _mm_mul_ps(data[iter_scale], scale_factor_intrinsic);
            data[iter_scale] = _mm_add_ps(data[iter_scale], scale_offset_intrinsic);
        }

    }

    end = time_msec();

    free((void *)data);
    free((void *)data_raw);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_nodec_intrinsic  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_nodecscale_intrinsic(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads) {

    int status, iter, iter_scale;
    double start, end, speed, timech;
    __m64 *data_raw;
    volatile __m128 *data;

    data_raw = (__m64 *)malloc(sizeof(uint16_t) * nsamples);
    data = (volatile __m128 *)malloc(sizeof(float) * nsamples);

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

        for (iter_scale = 0; iter_scale < nsamples / 4; iter_scale++) {
            data[iter_scale] = _mm_cvtpu16_ps(data_raw[iter_scale]);
        }

    }

    end = time_msec();

    free((void *)data);
    free((void *)data_raw);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_nodecscale_intrinsic  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_nofloat(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads) {

    int status, iter;
    double start, end, speed, timech;
    uint16_t *data_raw;

    data_raw = (uint16_t *)malloc(sizeof(uint16_t) * nsamples);

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {

        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }
    }

    end = time_msec();

    free((void *)data_raw);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_nofloat  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int readch_nofloat_mmap(sis8300drv_usr *sisuser,
        unsigned nsamples,
        unsigned nreads) {

    int status, iter;
    double start, end, speed, timech;
    uint16_t *data_raw;
    sis8300drv_dev *sisdevice;
    
    sisdevice = (sis8300drv_dev *)sisuser->device;

    data_raw = (uint16_t *)mmap(NULL, sizeof(uint16_t) * nsamples,
            PROT_READ, MAP_SHARED, sisdevice->handle, 0);
    if (data_raw == MAP_FAILED) {
        printf("mmap failed\n");
        return -1;
    }

    start = time_msec();

    for (iter = 0; iter < nreads; ++iter) {
        
        status = sis8300drv_read_ram(sisuser, 0, sizeof(uint16_t) * nsamples, data_raw);
        if (status) {
            printf("sis8300drv_read_ram error: %d\n", status);
            return -1;
        }

    }

    end = time_msec();

    munmap((void *)data_raw, sizeof(uint16_t) * nsamples);

    speed = ((double)(sizeof(uint16_t) * nsamples))/(end - start) * nreads / 1.0e3;
    timech = (end - start)/nreads;

    printf("readch_nofloat_mmap  time/ch %lf ms  speed %lf MB/s\n", timech, speed);

    return 0;
}


int main(int argc, char **argv) {
    sis8300drv_usr *sisuser;
    char c;
    unsigned nsamples, nreads, dec_offset, dec_factor;
    double scale_offset, scale_factor;
    int status;
    
    nsamples = 0x50000;
    nreads = 1;
    dec_offset = 0;
    dec_factor = 1;
    scale_offset = 0;
    scale_factor = 1;
    
    while ((c = getopt(argc, argv, "hS:N:o:f:O:F:")) != -1) {
        switch (c) {
            case 'S':
            	nsamples = strtol(optarg, NULL, 10);
                break;
            case 'N':
            	nreads = strtol(optarg, NULL, 10);
                break;
            case 'o':
            	dec_offset = strtol(optarg, NULL, 10);
                break;
            case 'f':
            	dec_factor = strtol(optarg, NULL, 10);
                break;
            case 'O':
            	scale_offset = strtold(optarg, NULL);
                break;
            case 'F':
            	scale_factor = strtold(optarg, NULL);
                break;
            case ':':
                printf("Option -%c requires an operand.\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device [-h] [-S nsamples] [-N nreads] \n", argv[0]);
                printf("   \n");
                printf("       -S unsigned int      Number of samples to read in a single iteration (default: 0x50000 ~ 320K) \n");
                printf("       -N unsigned int      Number of reads to perform (default: 1) \n");
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
    
    nsamples = ROUNDUP_HEX(nsamples);

    sisuser = malloc(sizeof(sis8300drv_usr));
    sisuser->file = argv[optind];

    status = sis8300drv_open_device(sisuser);
    if (status) {
        printf("sis8300drv_open_device error: %d\n", status);
        return -1;
    }
    
    status = sis8300drv_set_channel_mask(sisuser, 1);
    if (status) {
        printf("sis8300drv_set_channel_mask error: %d\n", status);
        return -1;
    }
    
    status = sis8300drv_set_nsamples(sisuser, nsamples);
    if (status) {
        printf("sis8300drv_set_nsamples: %d\n", status);
        return -1;
    }
    
    readch_orig(sisuser,
            nsamples,
            nreads,
            dec_offset,
            dec_factor,
            scale_offset,
            scale_factor);

    readch_norealloc(sisuser,
            nsamples,
            nreads,
            dec_offset,
            dec_factor,
            scale_offset,
            scale_factor);

    readch_separatebuf(sisuser,
            nsamples,
            nreads,
            dec_offset,
            dec_factor,
            scale_offset,
            scale_factor);

    readch_nodec(sisuser,
            nsamples,
            nreads,
            scale_offset,
            scale_factor);

    readch_noscale(sisuser,
            nsamples,
            nreads,
            dec_offset,
            dec_factor);

    readch_nodecscale(sisuser,
            nsamples,
            nreads);

    readch_nodecscale_fixed(sisuser,
            nreads);

    readch_intrinsic(sisuser,
            nsamples,
            nreads,
            dec_offset,
            dec_factor,
            scale_offset,
            scale_factor);

    readch_nodec_intrinsic(sisuser,
            nsamples,
            nreads,
            scale_offset,
            scale_factor);

    readch_nodecscale_intrinsic(sisuser,
            nsamples,
            nreads);

    readch_nofloat(sisuser,
            nsamples,
            nreads);

    readch_nofloat_mmap(sisuser,
            nsamples,
            nreads);

    sis8300drv_close_device(sisuser);

    return 0;
}
