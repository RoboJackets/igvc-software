/*
 * libraw1394 - library for raw access to the 1394 bus with the Linux subsystem.
 *
 * Copyright (C) 1999,2000 Andreas Bombe
 *
 * This library is licensed under the GNU Lesser General Public License (LGPL),
 * version 2.1 or later. See the file COPYING.LIB in the distribution for
 * details.
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/poll.h>
#include <stdlib.h>

#include "../src/raw1394.h"
#include "../src/csr.h"


#define TESTADDR (CSR_REGISTER_BASE + CSR_CYCLE_TIME)

const char not_compatible[] = "\
This libraw1394 does not work with your version of Linux. You need a different\n\
version that matches your kernel (see kernel help text for the raw1394 option to\n\
find out which is the correct version).\n";

const char not_loaded[] = "\
This probably means that you don't have raw1394 support in the kernel or that\n\
you haven't loaded the raw1394 module.\n";

quadlet_t buffer;

int my_tag_handler(raw1394handle_t handle, unsigned long tag,
                   raw1394_errcode_t errcode)
{
        int err = raw1394_errcode_to_errno(errcode);

        if (err) {
                printf("failed with error: %s\n", strerror(err));
        } else {
                printf("completed with value 0x%08x\n", buffer);
        }

        return 0;
}

int my_fcp_handler(raw1394handle_t handle, nodeid_t nodeid, int response,
                   size_t length, unsigned char *data)
{
        printf("got fcp %s from node %d of %d bytes:",
               (response ? "response" : "command"), nodeid & 0x3f, length);

        while (length) {
                printf(" %02x", *data);
                data++;
                length--;
        }

        printf("\n");

        return 0;
}


int main(int argc, char **argv)
{
        raw1394handle_t handle;
        int i, numcards;
        struct raw1394_portinfo pinf[16];

        tag_handler_t std_handler;
        int retval;
        
        struct pollfd pfd;
        unsigned char fcp_test[] = { 0x1, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef };
        quadlet_t rom[0x100];
        size_t rom_size;
        unsigned char rom_version;

        handle = raw1394_new_handle();

        if (!handle) {
                if (!errno) {
                        printf(not_compatible);
                } else {
                        perror("couldn't get handle");
                        printf(not_loaded);
                }
                exit(1);
        }

        printf("successfully got handle\n");
        printf("current generation number: %d\n", raw1394_get_generation(handle));

        numcards = raw1394_get_port_info(handle, pinf, 16);
        if (numcards < 0) {
                perror("couldn't get card info");
                exit(1);
        } else {
                printf("%d card(s) found\n", numcards);
        }

        if (!numcards) {
                exit(0);
        }

        for (i = 0; i < numcards; i++) {
                printf("  nodes on bus: %2d, card name: %s\n", pinf[i].nodes,
                       pinf[i].name);
        }
        
        if (raw1394_set_port(handle, 0) < 0) {
                perror("couldn't set port");
                exit(1);
        }

        printf("using first card found: %d nodes on bus, local ID is %d, IRM is %d\n",
               raw1394_get_nodecount(handle),
               raw1394_get_local_id(handle) & 0x3f,
               raw1394_get_irm_id(handle) & 0x3f);

        printf("\ndoing transactions with custom tag handler\n");
        std_handler = raw1394_set_tag_handler(handle, my_tag_handler);
        for (i = 0; i < pinf[0].nodes; i++) {
                printf("trying to send read request to node %d... ", i);
                fflush(stdout);
                buffer = 0;

                if (raw1394_start_read(handle, 0xffc0 | i, TESTADDR, 4,
                                       &buffer, 0) < 0) {
                        perror("failed");
                        continue;
                }
                raw1394_loop_iterate(handle);
        }

        printf("\nusing standard tag handler and synchronous calls\n");
        raw1394_set_tag_handler(handle, std_handler);
        for (i = 0; i < pinf[0].nodes; i++) {
                printf("trying to read from node %d... ", i);
                fflush(stdout);
                buffer = 0;

                retval = raw1394_read(handle, 0xffc0 | i, TESTADDR, 4, &buffer);
                if (retval < 0) {
                        perror("failed with error");
                } else {
                        printf("completed with value 0x%08x\n", buffer);
                }
        }

        printf("\ntesting FCP monitoring on local node\n");
        raw1394_set_fcp_handler(handle, my_fcp_handler);
        raw1394_start_fcp_listen(handle);
        raw1394_write(handle, raw1394_get_local_id(handle),
                      CSR_REGISTER_BASE + CSR_FCP_COMMAND, sizeof(fcp_test),
                      (quadlet_t *)fcp_test);
        raw1394_write(handle, raw1394_get_local_id(handle),
                      CSR_REGISTER_BASE + CSR_FCP_RESPONSE, sizeof(fcp_test),
                      (quadlet_t *)fcp_test);



        printf("testing config rom stuff\n");
        retval=raw1394_get_config_rom(handle, rom, 0x100, &rom_size, &rom_version);
        printf("get_config_rom returned %d, romsize %d, rom_version %d\n",retval,rom_size,rom_version);
        printf("here are the first 10 quadlets:\n");
        for (i = 0; i < 10; i++)
                printf("%d. quadlet: 0x%08x\n",i,rom[i]);

        /* some manipulation */
/*        printf("incrementing 2nd quadlet\n");
        rom[0x02/4]++; 
*/
        retval=raw1394_update_config_rom(handle, rom, rom_size, rom_version);
        printf("update_config_rom returned %d\n",retval);



        printf("\npolling for leftover messages\n");
        pfd.fd = raw1394_get_fd(handle);
        pfd.events = POLLIN;
        pfd.revents = 0;
        while (1) {
                retval = poll(&pfd, 1, 10);
                if (retval < 1) break;
                raw1394_loop_iterate(handle);
        }

        if (retval < 0) perror("poll failed");
        exit(0);
}
