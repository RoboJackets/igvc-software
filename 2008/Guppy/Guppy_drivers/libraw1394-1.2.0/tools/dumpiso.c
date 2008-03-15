/*
 * libraw1394 - library for raw access to the 1394 bus with the Linux subsystem.
 *
 * Copyright (C) 1999,2000 Andreas Bombe
 *
 * This library is licensed under the GNU Lesser General Public License (LGPL),
 * version 2.1 or later. See the file COPYING.LIB in the distribution for
 * details.
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>
#include <fcntl.h>

#include "../src/raw1394.h"

#define BUFFER 1000
#define PACKET_MAX 4096

u_int64_t listen_channels;
unsigned long which_port;
char *filename;
int file;
enum raw1394_iso_dma_recv_mode mode = RAW1394_DMA_DEFAULT;

void usage_exit(int exitcode)
{
        fprintf(stderr,
"Usage: dumpiso [opts] [FILE]\n"
"Dump IEEE 1394 isochronous channels to FILE or standard output.\n"
"\n"
"-c --channels  CHANNELS    Listen on these channels; CHANNELS is either a\n"
"                           number X or a range X-Y.\n"
"-p --port      PORT        Choose 1394 chip PORT. (default: 0)\n"
"-h --help                  Show this help.\n"
);

        exit(exitcode);
}

void parse_args(int argc, char **argv)
{
        int i;
        char *tail;
        unsigned long chan1, chan2;

        int c;
        int index;
        static struct option opts[] = {
                { "channels", required_argument, NULL, 'c' },
                { "port",     required_argument, NULL, 'p' },
                { "help",     no_argument,       NULL, 'h' },
                { 0 }
        };

        while (1) {
                c = getopt_long(argc, argv, "hc:p:", opts, &index);
                if (c == -1) break;

                switch (c) {
                case 'c':
                        chan1 = strtoul(optarg, &tail, 10);
                        chan2 = chan1;

                        if (*tail) {
                                if (tail[0] != '-' || !tail[1]) {
                                        fprintf(stderr,
                                                "invalid argument to channels: %s\n",
                                                optarg);
                                        usage_exit(1);
                                }

                                tail++;
                                chan2 = strtoul(tail, &tail, 10);
                                if (*tail) {
                                        fprintf(stderr,
                                                "invalid argument to channels: %s\n",
                                                optarg);
                                        usage_exit(1);
                                }
                        } else {
                                mode = RAW1394_DMA_PACKET_PER_BUFFER;
                        }

                        if (chan2 < chan1) {
                                unsigned long x = chan1;
                                chan1 = chan2;
                                chan2 = x;
                        }

                        if (chan2 > 63) {
                                fprintf(stderr,
                                        "invalid channel numbers: %s\n",
                                        optarg);
                                exit(1);
                        }

                        for (i = chan1; i <= chan2; i++)
                                listen_channels |= 1ULL << i;

                        break;
                case 'p':
                        which_port = strtoul(optarg, &tail, 10);
                        if (*tail) {
                                fprintf(stderr,
                                        "invalid argument to port: %s\n",
                                        optarg);
                                usage_exit(1);
                        }
                        break;
                case 'h':
                        usage_exit(0);
                case '?':
                        usage_exit(1);
                default:
                        abort();
                }
        }

        argv += optind;
        argc -= optind;

        if (argc > 1) {
                fprintf(stderr, "Too many arguments.\n");
                usage_exit(1);
        }

        if (argc) filename = *argv;

        if (!listen_channels) listen_channels = ~0ULL;
}

void write_header()
{
        static char header[32] = "1394 isodump v2";
        int i;

        for (i = 0; i < 8; i++)
                header[i+16] = (listen_channels >> (56 - 8*i)) & 0xff;

        i = 0;
        while (i < 32) {
                int ret;
                ret = write(file, header + i, 32 - i);

                if (ret < 0) {
                        perror("header write");
                        exit(1);
                }

                i += ret;
        }
}

void open_dumpfile()
{
        if (!filename || !filename[0] || (filename[0] == '-' && !filename[1])) {
                file = fileno(stdout);
                write_header();
                return;
        }

        file = open(filename, O_CREAT | O_WRONLY, 0666);
        if (file < 0) {
                perror("dumpfile open");
                exit(1);
        }
        
        ftruncate(file, 0);
        write_header();
}

static enum raw1394_iso_disposition 
iso_handler(raw1394handle_t handle, unsigned char *data, 
        unsigned int length, unsigned char channel,
        unsigned char tag, unsigned char sy, unsigned int cycle, 
        unsigned int dropped)
{
        int ret;
        static unsigned int counter = 0;

        if (++counter % 1000 == 0)
                fprintf(stderr, "\r%uK packets", counter/1000);

        /* write header */
        write(file, &length, sizeof(length));
        write(file, &channel, sizeof(channel));
        write(file, &tag, sizeof(tag));
        write(file, &sy, sizeof(sy));
        sy = 0;
        write(file, &sy, sizeof(sy));

        while (length) {
                ret = write(file, data, length);
                if (ret < 0) {
                        perror("data write");
                        return RAW1394_ISO_ERROR;
                }

                length -= ret;
                data += ret;
        }

        return RAW1394_ISO_OK;
}

int main(int argc, char **argv)
{
        raw1394handle_t handle;
        int i;

        parse_args(argc, argv);

        fprintf(stderr, "port: %ld\nchannels: %#016llx\nfile: %s\n", which_port,
                listen_channels, filename);

        handle = raw1394_new_handle();
        if (!handle) {
                if (!errno)
                        fprintf(stderr,
                                "No working kernel driver found.\n");
                else
                        perror("raw1394_get_handle");
                exit(1);
        }

        do {
                if (raw1394_get_port_info(handle, NULL, 0) <= which_port) {
                        fprintf(stderr, "Port %ld does not exist.\n",
                                which_port);
                        exit(1);
                }

                raw1394_set_port(handle, which_port);
        } while (errno == ESTALE);

        if (errno) {
                perror("raw1394_set_port");
                exit(1);
        }

        open_dumpfile();

        if (mode == RAW1394_DMA_DEFAULT) {
                raw1394_iso_multichannel_recv_init(handle, iso_handler,
                        BUFFER, 2048, -1); /* >2048 makes rawiso stall! */
                raw1394_iso_recv_set_channel_mask(handle, listen_channels);

        } else for (i = 0; i < 64; i++) {
                if (!(listen_channels & 1ULL << i))
                        continue;
                raw1394_iso_recv_init(handle, iso_handler, BUFFER, PACKET_MAX,
                        i, mode, -1);
        }
        raw1394_iso_recv_start(handle, -1, -1, 0);

        while (raw1394_loop_iterate(handle) == 0);

        fprintf(stderr, "\n");
        raw1394_iso_shutdown(handle);
        raw1394_destroy_handle(handle);

        return 0;
}
