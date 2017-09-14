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

#include "sis8300drv.h"

int main(int argc, char **argv) {
    char            c;
	int             status, verbose, reg_write;
	unsigned        reg_value, offset;
	sis8300drv_usr  *sisuser;
	
	verbose = 0;
	reg_value = 0;
	reg_write = 0;
	
    while ((c = getopt(argc, argv, "hvw:")) != -1) {
        switch (c) {
            case 'w':
            	reg_value = strtol(optarg, NULL, 16);
                reg_write = 1;
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
                printf("Usage: %s device offset [-h] [-w value] \n", argv[0]);
                printf("   \n");
                printf("       offset unsigned int  Register offset in HEX \n");
                printf("   \n");
                printf("       -w unsigned int      Write value to register in HEX \n");
                printf("   \n");
                printf("       -v                   Verbose output \n");
                printf("       -h                   Print this message \n");
                printf("   \n");
                return -1;
        }
    }

    if (optind != argc - 2) {
        printf("Usage: %s device offset \n", argv[0]);
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
    
	offset = strtol(argv[optind + 1], NULL, 16);
    
    if (reg_write) {
        if (verbose) {
            printf("write to offset 0x%x: 0x%x\n", offset, reg_value);
        }
        
        status = sis8300drv_reg_write(sisuser, offset, reg_value);
        if (status) {
            printf("sis8300drv_reg_write error: %s (%d)\n", 
                    sis8300drv_strerror(status), status);
            return -1;
        }
    }
    
    status = sis8300drv_reg_read(sisuser, offset, &reg_value);
    if (status) {
        printf("sis8300drv_reg_read error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    if (!reg_write && !verbose) {
        printf("0x%x\n", reg_value);
    }
    
    if (verbose) {
        printf("read from offset 0x%x: 0x%x\n", offset, reg_value);
    }
    
    sis8300drv_close_device(sisuser);

	return(0);
}
