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
    char c;
    int status, release;
    unsigned timeout;
    sis8300drv_irq_type irq_type;
    sis8300drv_usr *sisuser;

    release = 0;
    timeout = 0;
    irq_type = irq_type_usr;
    
    while ((c = getopt(argc, argv, "hrt:i:")) != -1) {
        switch (c) {
            case 't':
                timeout = strtol(optarg, NULL, 10);
                break;
            case 'i':
            	irq_type = (sis8300drv_irq_type)strtol(optarg, NULL, 10);
                break;
            case 'r':
                release = 1;
                break;
            case ':':
                printf("Option -%c requires an operand.\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device [-h] [-t timeout] [-i irq_type] [-r]\n", argv[0]);
                printf("   \n");
                printf("       -i unsigned int      Type of irq to wait for (0 - daq-done irq, 1 - user-defined irq, default: 1) \n");
                printf("       -t unsigned int      Wait timeout in milliseconds (default: 0 - infinite timeout) \n");
                printf("       -r                   Release all waiters for irq \n");
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
        printf("sis8300drv_open_device error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    if (release) {
    
        status = sis8300drv_release_irq(sisuser, irq_type);
        if (status) {
            printf("sis8300drv_release_irq error: %s (%d)\n", 
                    sis8300drv_strerror(status), status);
            return -1;
        }
        
        printf("released irq waiters\n");
        
    } else {
    
        status = sis8300drv_wait_irq(sisuser, irq_type, timeout);
        switch (status) {
            case status_success:
                printf("irq happened\n");
                break;
            case status_irq_timeout:
                printf("irq timeout\n");
                break;
            case status_irq_release:
                printf("irq released\n");
                break;
            default:
                if (status) {
                    printf("sis8300drv_wait_irq: %s (%d)\n", 
                            sis8300drv_strerror(status), status);
                    return -1;
                }
                break;
        }
        
    }

    sis8300drv_close_device(sisuser);

    return 0;
}
