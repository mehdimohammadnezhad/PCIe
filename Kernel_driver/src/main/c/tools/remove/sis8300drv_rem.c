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
#include "sis8300_reg.h"


int main(int argc, char **argv) {
    char c;
    int status, poll, iter;
    unsigned reg_val;
    sis8300drv_usr *sisuser;
    
    poll = 0;

    while ((c = getopt(argc, argv, "hp")) != -1) {
        switch (c) {
            case 'p':
                poll = 1;
                break;
            case ':':
                printf("Option -%c requires an operand.\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device [-h] [-p] \n", argv[0]);
                printf("   \n");
                printf("       -p           Poll with register reads instead of using udev \n");
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

    sisuser = malloc(sizeof(sis8300drv_usr));
    sisuser->file = argv[optind];

    status = sis8300drv_open_device(sisuser);
    if (status) {
        printf("sis8300drv_open_device error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    if (poll) {
        printf("polling device registers\n");
        iter = 0;
        do {
            status = sis8300drv_reg_read(sisuser, 0, &reg_val);
            if (status) {
                printf("sis8300drv_reg_read error: %s (%d)\n", 
                        sis8300drv_strerror(status), status);
            }
            iter++;
            if (!(iter % 64)) {
                printf("\n");
            }
            printf(".");
            fflush(stdout);
            usleep(100000);
        } while (!status);
    } else {
        printf("waiting for device removal\n");
        sis8300drv_wait_remove(sisuser);
    }
    
    printf("device removed");

    sis8300drv_close_device(sisuser);

    return(0);
}
