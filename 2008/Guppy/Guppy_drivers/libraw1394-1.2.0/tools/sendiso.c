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
#include <string.h>

#include "../src/raw1394.h"

#define BUFFER 1000
#define PACKET_MAX 4096

unsigned long which_port;
char *filename;

unsigned long loopcount = 1;
unsigned int speed;

void usage_exit(int exitcode)
{
        fprintf(stderr,
"Usage: sendiso [opts] FILE\n"
"Send IEEE 1394 isochronous packets from dump file FILE.\n"
"\n"
"-l --loop      COUNT       Repeat sending data COUNT times.\n"
"-i --infinite              Repeat sending data infinitely.\n"
"-s --speed     SPEED       Send data at SPEED (valid values are 100, 200, 400 or\n"
"                           alternatively 1, 2, 4). (default: 100)\n"
"\n"
"-p --port      PORT        Choose 1394 chip PORT. (default: 0)\n"
"-h --help                  Show this help.\n"
);

        exit(exitcode);
}

void parse_args(int argc, char **argv)
{
        char *tail;

        int c;
        int index;
        static struct option opts[] = {
                { "file",     required_argument, NULL, 'f' },
                { "loop",     required_argument, NULL, 'l' },
                { "infinite", no_argument,       NULL, 'i' },
                { "speed",    required_argument, NULL, 's' },
                { "port",     required_argument, NULL, 'p' },
                { "help",     no_argument,       NULL, 'h' },
                { 0 }
        };

        while (1) {
                c = getopt_long(argc, argv, "f:l:is:p:h", opts, &index);
                if (c == -1) break;

                switch (c) {
                case 'f':
                        filename = optarg;
                        break;
                case 'l':
                        loopcount = strtoul(optarg, &tail, 10);
                        if (*tail) {
                                fprintf(stderr,
                                        "invalid argument to loop: %s\n",
                                        optarg);
                                usage_exit(1);
                        }
                        break;
                case 'i':
                        loopcount = 0;
                        break;
                case 's':
                        speed = strtoul(optarg, &tail, 10);
                        if (*tail) speed = -1;

                        switch (speed) {
                        case 1:
                        case 100:
                                speed = RAW1394_ISO_SPEED_100;
                                break;
                        case 2:
                        case 200:
                                speed = RAW1394_ISO_SPEED_200;
                                break;
                        case 4:
                        case 400:
                                speed = RAW1394_ISO_SPEED_400;
                                break;
                        default:
                                fprintf(stderr,
                                        "invalid argument to speed: %s\n",
                                        optarg);
                                usage_exit(1);
                        }
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
                case 0:
                        break;
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
}

#define BUF_SIZE 4096
#define BUF_HEAD 8
void send_file_once(raw1394handle_t handle, int file)
{
        int count, i, ret;
        unsigned channel, tag, sy;
        size_t length;
        static char buffer[BUF_SIZE + BUF_HEAD];
        static unsigned int counter = 0;
        static int inited = 0;

        while (1) {
                count = read(file, buffer, BUF_HEAD);
                if (count < 0) {
                        perror("read");
                        exit(1);
                }
                if (count < BUF_HEAD)
                        return;
        
                i = 0;
                length = ((unsigned int *)buffer)[i];
                channel = buffer[i + 4];
                tag = buffer[i + 5];
                sy = buffer[i + 6];
        
                i += BUF_HEAD;
                while (count < length + BUF_HEAD) {
                        ret = read(file, buffer + count,
                                   length - count + BUF_HEAD);

                        if (ret < 0) {
                                perror("read");
                                exit(1);

                        }
                        if (ret == 0)
                                return;

                        count += ret;
                }
                
                if (inited == 0) {
                        /*
                        fprintf(stderr, "transmitting first packet with length "
                             "%d on channel %d with tag %d and sy %d\n",
                                length, channel, tag, sy);
                        */
                        ret = raw1394_iso_xmit_init(handle, NULL, BUFFER,
                                PACKET_MAX, channel, speed, -1);
                        if (ret < 0) {
                                perror("raw1394_iso_xmit_init");
                                exit(1);
                        }
                        raw1394_iso_xmit_start(handle, -1, -1);
                        inited = 1;
                }

                if (++counter % 1000 == 0)
                        fprintf(stderr, "\r%uK packets", counter/1000);

                ret = raw1394_iso_xmit_write(handle, &buffer[i],
                        length, tag, sy);
                if (ret < 0) {
                        perror("\nraw1394_iso_xmit_write");
                        exit(1);
                }
        }
}


void send_iso_file(raw1394handle_t handle)
{
        int file;
        int count, ret;
        char buffer[32];

        if (filename[0] == '-' && filename[1] == '\0') {
                file = fileno(stdin);
        } else {
                file = open(filename, O_RDONLY, 0);

                if (file < 0) {
                        perror("open");
                        exit(1);
                }
        }

        count = 32;
        while (count) {
                ret = read(file, buffer, count);

                if (!ret) goto bad_format;

                if (ret < 0) {
                        perror("read");
                        exit(1);
                }

                count -= ret;
        }

        if (memcmp("1394 isodump v", buffer, 14)) goto bad_format;
        if (buffer[14] != '2') goto wrong_version;

        while (1) {
                send_file_once(handle, file);

                if (!loopcount) {
                        if (lseek(file, 32, SEEK_SET) < 0) {
                                perror("lseek");
                                exit(1);
                        }
                        continue;
                }
                if (!(--loopcount)) break;

                if (lseek(file, 32, SEEK_SET) < 0) {
                        perror("lseek");
                        exit(1);
                }
        }

        return;

bad_format:
        fprintf(stderr, "Input file format not recognized.\n");
        exit(1);

wrong_version:
        fprintf(stderr, "Format version of input file not supported.\n");
        exit(1);
}


int main(int argc, char **argv)
{
        raw1394handle_t handle;

        parse_args(argc, argv);

        fprintf(stderr, "port: %ld\nloops: %ld\nfile: %s\n", which_port,
                loopcount, filename);

        handle = raw1394_new_handle();
        if (!handle) {
                if (!errno)
                        fprintf(stderr, "No working kernel driver found.\n");
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

        if (filename)
                send_iso_file(handle);

        fprintf(stderr, "\n");
        raw1394_iso_shutdown(handle);
        raw1394_destroy_handle(handle);

        return 0;
}
