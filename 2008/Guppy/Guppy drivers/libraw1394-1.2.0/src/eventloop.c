/*
 * libraw1394 - library for raw access to the 1394 bus with the Linux subsystem.
 *
 * Copyright (C) 1999,2000,2001,2002 Andreas Bombe
 *                     2002 Manfred Weihs <weihs@ict.tuwien.ac.at>
 *                     2002 Christian Toegel <christian.toegel@gmx.at>
 *
 * This library is licensed under the GNU Lesser General Public License (LGPL),
 * version 2.1 or later. See the file COPYING.LIB in the distribution for
 * details.
 *
 *
 * Contributions:
 *
 * Manfred Weihs <weihs@ict.tuwien.ac.at>
 *        address range mapping
 * Christian Toegel <christian.toegel@gmx.at>
 *        address range mapping
 */

#include <config.h>
#include <unistd.h>
#include <byteswap.h>

#include "raw1394.h"
#include "kernel-raw1394.h"
#include "raw1394_private.h"


int raw1394_loop_iterate(struct raw1394_handle *handle)
{
        struct raw1394_request req;
        int retval = 0, channel;

        if (read(handle->fd, &req, sizeof(req)) < 0) {
                return -1;
        }

        switch (req.type) {
        case RAW1394_REQ_BUS_RESET:
                if (handle->protocol_version == 3) {
                        handle->num_of_nodes = req.misc & 0xffff;
                        handle->local_id = req.misc >> 16;
                } else {
                        handle->num_of_nodes = req.misc & 0xff;
                        handle->irm_id = ((req.misc >> 8) & 0xff) | 0xffc0;
                        handle->local_id = req.misc >> 16;
                }

                if (handle->bus_reset_handler) {
                        retval = handle->bus_reset_handler(handle,
                                                           req.generation);
                }
                break;

        case RAW1394_REQ_ISO_RECEIVE:
                channel = (handle->buffer[0] >> 8) & 0x3f;
#ifndef WORDS_BIGENDIAN
                handle->buffer[0] = bswap_32(handle->buffer[0]);
#endif

                if (handle->iso_handler[channel]) {
                        retval = handle->iso_handler[channel](handle, channel,
                                                              req.length,
                                                              handle->buffer);
                }
                break;

        case RAW1394_REQ_FCP_REQUEST:
                if (handle->fcp_handler) {
                        retval = handle->fcp_handler(handle, req.misc & 0xffff,
                                                     req.misc >> 16,
                                                     req.length,
                                                     (char *)handle->buffer);
                }
                break;

        case RAW1394_REQ_ARM:
                if (handle->arm_tag_handler) {
                        retval = handle->arm_tag_handler(handle, req.tag,
                                 (req.misc & (0xFF)), 
                                 ((req.misc >> 16) & (0xFFFF)),
                                 int2ptr(req.recvb));
                } 
                break;
                
        case RAW1394_REQ_ECHO:
                retval=req.misc;
                break;

        case RAW1394_REQ_RAWISO_ACTIVITY:
                retval = _raw1394_iso_iterate(handle);
                break;
        
        default:
                if (handle->tag_handler) {
                        retval = handle->tag_handler(handle, req.tag,
                                                     req.error);
                }
                break;
        }

        return retval;
}


bus_reset_handler_t raw1394_set_bus_reset_handler(struct raw1394_handle *handle,
                                                  bus_reset_handler_t new)
{
        bus_reset_handler_t old;

        old = handle->bus_reset_handler;
        handle->bus_reset_handler = new;

        return old;
}

tag_handler_t raw1394_set_tag_handler(struct raw1394_handle *handle, 
                                      tag_handler_t new)
{
        tag_handler_t old;

        old = handle->tag_handler;
        handle->tag_handler = new;

        return old;
}

arm_tag_handler_t raw1394_set_arm_tag_handler(struct raw1394_handle *handle, 
                                      arm_tag_handler_t new)
{
        arm_tag_handler_t old;

        old = handle->arm_tag_handler;
        handle->arm_tag_handler = new;

        return old;
}


iso_handler_t raw1394_set_iso_handler(struct raw1394_handle *handle,
                                      unsigned int channel, iso_handler_t new)
{
        if (channel >= 64) {
                return (iso_handler_t)-1;
        }

        if (new == NULL) {
                iso_handler_t old = handle->iso_handler[channel];
                handle->iso_handler[channel] = NULL;
                return old;
        }

        if (handle->iso_handler[channel] != NULL) {
                return (iso_handler_t)-1;
        }

        handle->iso_handler[channel] = new;
        return NULL;
}

fcp_handler_t raw1394_set_fcp_handler(struct raw1394_handle *handle,
                                      fcp_handler_t new)
{
        fcp_handler_t old;

        old = handle->fcp_handler;
        handle->fcp_handler = new;

        return old;
}
