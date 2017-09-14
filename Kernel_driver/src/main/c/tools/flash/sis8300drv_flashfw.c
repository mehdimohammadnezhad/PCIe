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
#include <sys/stat.h>

#include "sis8300drv.h"

#define SIS8300DRV_ESS_CUSTOM_FW_ID_REG 0x400
#define SIS8300DRV_ESS_CUSTOM_FW_BPM    0xa00a
#define SIS8300DRV_ESS_CUSTOM_FW_LLRF   0xb00b
#define SIS8300DRV_ESS_CUSTOM_FW_BCM    0xc00c


unsigned image_size;
unsigned image_offset;

void callback(unsigned offset);


int main(int argc, char **argv) {
    char c;
    char *image_path;
    void *image, *image_check;
    int status, info, verify;
    unsigned fw_version;
    FILE *image_file;
    struct stat stbuf;
    sis8300drv_usr *sisuser;
    
    info = 0;
    verify = 0;
    
    while ((c = getopt(argc, argv, "hvi")) != -1) {
        switch (c) {
            case 'i':
                info = 1;
                break;
            case 'v':
                verify = 1;
                break;
            case ':':
                printf("Option -%c requires an operand.\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device image [-h] [-v] [-i] \n", argv[0]);
                printf("   \n");
                printf("       -i           Only read the info about the firmware from the device \n");
                printf("       -v           Only read firmware image from device and compare it \n");
                printf("   \n");
                printf("       -h           Print this message \n");
                printf("   \n");
                return -1;
        }
    }
    
    if (info) {
        if (optind != argc - 1) {
            printf ("Only device argument required.\n");
            return -1;
        }
    } else {
        if (optind != argc - 2) {
            printf ("Device and firmware image argument required.\n");
            return -1;
        }
    }
    
    sisuser = malloc(sizeof(sis8300drv_usr));
    sisuser->file = argv[optind];
    
    image_path = argv[optind + 1];

    status = sis8300drv_open_device(sisuser);
    if (status) {
        printf("sis8300drv_open_device error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    status = sis8300drv_get_fw_version(sisuser, &fw_version);
    if (status) {
        printf("sis8300drv_get_fw_version error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    printf("Device: %s\n", sisuser->file);
    printf("Current firmware image:\n");
    printf("    Base version: 0x%04x\n", fw_version & 0xFFFF);
    
    status = sis8300drv_reg_read(sisuser, SIS8300DRV_ESS_CUSTOM_FW_ID_REG, 
            &fw_version);
    if (status) {
        printf("sis8300drv_reg_read error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    } else {
        printf("    ESS custom id: 0x%x\n", fw_version);
        printf("    ESS device: ");
        switch (fw_version >> 16) {
            case SIS8300DRV_ESS_CUSTOM_FW_BPM:
                printf("BPM ");
                break;
            case SIS8300DRV_ESS_CUSTOM_FW_LLRF:
                printf("LLRF ");
                break;
            case SIS8300DRV_ESS_CUSTOM_FW_BCM:
                printf("BCM ");
                break;
            default:
                printf("unknown ");
                break;
        }
        printf("(version 0x%04x)\n", fw_version & 0xFFFF);
    }
    
    if (info) {
        return 0;
    }
    
    
    image_file = fopen(image_path, "rb");
    if (!image_file) {
        printf("Error opening firmware image file.\n");
        return -1;
    }
    
    if ((stat(image_path, &stbuf) != 0) || (!S_ISREG(stbuf.st_mode))) {
        printf("Error stating firmware image file.\n");
        return -1;
    }
    
    image_size = stbuf.st_size;
    image = malloc(image_size);

    status = fread(image, sizeof(uint8_t), image_size, image_file);
    if (status < 0) {
        printf("Error reading firmware image file.\n");
        fclose(image_file);
        free(image);
        return -1;
    }
    fclose(image_file);
    
    printf("Provided firmware image:\n");
    printf("    File: %s\n", image_path);
    printf("    Size: %u\n", image_size);
    
    if (verify) {
        printf("Reading firmware image from device.\n");
        
        image_check = malloc(image_size);
        status = sis8300drv_read_fw_image(sisuser, image_size, image_check, callback);
        if (status) {
            printf("\nsis8300drv_read_fw_image error: %s (%d)\n", 
                    sis8300drv_strerror(status), status);
            return -1;
        }
        printf("\rCompleted 100%% (offset 0x%x)\n", image_offset);
        
        if (memcmp(image, image_check, image_size)) {
            printf("Firmware image verification failed.\n");
            return -1;
        }
        
        printf("Firmware image verification successful.\n");
    } else {
        do{
            printf("Flash firmware image? [y/n]: ");
            fflush(stdout);
            c = getchar();
            while (c != '\n' && getchar() != '\n');
        } while (c != 'y' && c != 'n');
        
        if (c == 'y') {
            printf("Writing firmware image to device.\n");
            status = sis8300drv_write_fw_image(sisuser, image_size, image, callback);
            if (status) {
                printf("\nsis8300drv_write_fw_image error: %s (%d)\n", 
                        sis8300drv_strerror(status), status);
                printf("Writing firmware image failed at offset 0x%x\n", image_offset);
                return -1;
            }
            printf("\rCompleted 100%% (offset 0x%x)\n", image_offset);
        }
        
        printf("Firmware update successful.\n");
    }

    sis8300drv_close_device(sisuser);

    return 0;
}


void callback(unsigned offset) {
    image_offset = offset;
    printf("\rCompleted %u%% (offset 0x%x)", offset * 100 / image_size, offset);
    fflush(stdout);
}






















