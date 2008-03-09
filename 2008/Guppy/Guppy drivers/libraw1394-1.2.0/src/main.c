/*
 * libraw1394 - library for raw access to the 1394 bus with the Linux subsystem.
 *
 * Copyright (C) 1999,2000,2001,2002 Andreas Bombe
 *               2001, 2002 Manfred Weihs <weihs@ict.tuwien.ac.at>
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
 *        configuration ROM manipulation
 *        address range mapping
 * Christian Toegel <christian.toegel@gmx.at>
 *        address range mapping
 *        reset notification control (switch on/off)
 *        reset with selection of type (short/long)
 */

#include <config.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <netinet/in.h>

#include "raw1394.h"
#include "csr.h"
#include "kernel-raw1394.h"
#include "raw1394_private.h"


static int bus_reset_default(struct raw1394_handle *handle, unsigned int gen)
{
        raw1394_update_generation(handle, gen);
        return 0;
}

static int tag_handler_default(struct raw1394_handle *handle, unsigned long tag,
                               int error)
{
        struct raw1394_reqhandle *rh;

        if (tag) {
                rh = (struct raw1394_reqhandle *)tag;
                return rh->callback(handle, rh->data, error);
        } else {
                return -1;
        }
}

static int arm_tag_handler_default(struct raw1394_handle *handle, unsigned long tag,
                               byte_t request_type, unsigned int requested_length,
			       void *data)
{
        struct raw1394_arm_reqhandle *rh;
        struct raw1394_arm_request_response *arm_req_resp;

        if (tag) {
                rh = (struct raw1394_arm_reqhandle *)tag;
                arm_req_resp  = (struct raw1394_arm_request_response *) data;
                return rh->arm_callback(handle, arm_req_resp, 
                                        requested_length, rh->pcontext, 
                                        request_type);
        } else {
                /* error ... */
                return -1;
        }
}

int _raw1394_sync_cb(struct raw1394_handle *unused, struct sync_cb_data *data,
                     int error)
{
        data->errcode = error;
        data->done = 1;
        return 0;
}




static unsigned int init_rawdevice(struct raw1394_handle *h)
{
        struct raw1394_request req;

        CLEAR_REQ(&req);
        req.type = RAW1394_REQ_INITIALIZE;
        req.misc = RAW1394_KERNELAPI_VERSION;
        h->protocol_version = RAW1394_KERNELAPI_VERSION;

        if (write(h->fd, &req, sizeof(req)) < 0) return -1;
        if (read(h->fd, &req, sizeof(req)) < 0) return -1;

        if (req.error == RAW1394_ERROR_COMPAT && req.misc == 3) {
                h->protocol_version = 3;
                if (write(h->fd, &req, sizeof(req)) < 0) return -1;
                if (read(h->fd, &req, sizeof(req)) < 0) return -1;
        }

        if (req.error) {
                errno = 0;
                return -1;
        }
        memset(h->buffer, 0, HBUF_SIZE);

        return req.generation;
}


struct raw1394_handle *raw1394_new_handle(void)
{
        struct raw1394_handle *handle;

        handle = malloc(sizeof(struct raw1394_handle));
        if (!handle) {
                errno = ENOMEM;
                return NULL;
        }

        handle->fd = open("/dev/raw1394", O_RDWR);
        if (handle->fd < 0) {
                free(handle);
                return NULL;
        }

        handle->generation = init_rawdevice(handle);
        if (handle->generation == -1) {
                close(handle->fd);
                free(handle);
                return NULL;
        }

        handle->err = 0;
        handle->bus_reset_handler = bus_reset_default;
        handle->tag_handler = tag_handler_default;
        handle->arm_tag_handler = arm_tag_handler_default;
        memset(handle->iso_handler, 0, sizeof(handle->iso_handler));
	handle->iso_buffer = NULL;
	handle->iso_mode = ISO_INACTIVE;
        return handle;
}

void raw1394_destroy_handle(struct raw1394_handle *handle)
{
	if (handle) {
		if(handle->iso_mode != ISO_INACTIVE) {
			raw1394_iso_shutdown(handle);
		}
		close(handle->fd);
		free(handle);
	}
}

int raw1394_get_fd(struct raw1394_handle *handle)
{
        return handle->fd;
}

unsigned int raw1394_get_generation(struct raw1394_handle *handle)
{
        return handle->generation;
}

void raw1394_update_generation(struct raw1394_handle *handle, unsigned int gen)
{
        handle->generation = gen;
}

int raw1394_get_nodecount(struct raw1394_handle *handle)
{
        return handle->num_of_nodes;
}

nodeid_t raw1394_get_local_id(struct raw1394_handle *handle)
{
        return handle->local_id;
}

nodeid_t raw1394_get_irm_id(struct raw1394_handle *handle)
{
        return handle->irm_id;
}

void raw1394_set_userdata(struct raw1394_handle *handle, void *data)
{
        handle->userdata = data;
}

void *raw1394_get_userdata(struct raw1394_handle *handle)
{
        return handle->userdata;
}

int raw1394_get_port_info(struct raw1394_handle *handle, 
                          struct raw1394_portinfo *pinf, int maxports)
{
        int num;
        struct raw1394_request req;
        struct raw1394_khost_list *khl;

        CLEAR_REQ(&req);
        req.type = RAW1394_REQ_LIST_CARDS;
        req.generation = handle->generation;
        /* IMPORTANT: raw1394 will be writing directly into the memory you
           provide in pinf. The viability of this approach assumes that the 
           structure of libraw1394's raw1394_portinfo and the kernel's 
           raw1394_khost_list structs are the same!!
        */
        req.recvb = ptr2int(pinf);
        req.length = sizeof(struct raw1394_portinfo) * maxports;

        while (1) {
                if (write(handle->fd, &req, sizeof(req)) < 0) return -1;
                if (read(handle->fd, &req, sizeof(req)) < 0) return -1;

                if (!req.error) break;

                if (req.error == RAW1394_ERROR_GENERATION) {
                        handle->generation = req.generation;
                        continue;
                }

                return -1;
        }

        return req.misc;
}


int raw1394_set_port(struct raw1394_handle *handle, int port)
{
        struct raw1394_request req;

        CLEAR_REQ(&req);

        req.type = RAW1394_REQ_SET_CARD;
        req.generation = handle->generation;
        req.misc = port;

        if (write(handle->fd, &req, sizeof(req)) < 0) return -1;
        if (read(handle->fd, &req, sizeof(req)) < 0) return -1;

        switch (req.error) {
        case RAW1394_ERROR_GENERATION:
                handle->generation = req.generation;
                errno = ESTALE;
                return -1;
        case RAW1394_ERROR_INVALID_ARG:
                errno = EINVAL;
                return -1;
        case RAW1394_ERROR_NONE:
                if (handle->protocol_version == 3) {
                        handle->num_of_nodes = req.misc & 0xffff;
                        handle->local_id = req.misc >> 16;
                } else {
                        handle->num_of_nodes = req.misc & 0xff;
                        handle->irm_id = ((req.misc >> 8) & 0xff) | 0xffc0;
                        handle->local_id = req.misc >> 16;
                }
                handle->generation = req.generation;
                return 0;
        default:
                errno = 0;
                return -1;
        }
}

raw1394handle_t raw1394_new_handle_on_port(int port)
{
	raw1394handle_t handle = raw1394_new_handle();
	if (!handle)
		return NULL;

tryagain:
	if (raw1394_get_port_info(handle, NULL, 0) < 0) {
		raw1394_destroy_handle(handle);
		return NULL;
	}

	if (raw1394_set_port(handle, port)) {
		if (errno == ESTALE || errno == EINTR) {
			goto tryagain;
		} else {
			raw1394_destroy_handle(handle);
			return NULL;
		}
	}

	return handle;
}

int raw1394_reset_bus_new(struct raw1394_handle *handle, int type)
{
        struct raw1394_request req;

        CLEAR_REQ(&req);

        req.type = RAW1394_REQ_RESET_BUS;
        req.generation = handle->generation;
        req.misc = type;
	
        if (write(handle->fd, &req, sizeof(req)) < 0) return -1;

        return 0; /* success */
}


int raw1394_reset_bus(struct raw1394_handle *handle)
{
        return raw1394_reset_bus_new (handle, RAW1394_LONG_RESET);
}

int raw1394_busreset_notify (struct raw1394_handle *handle, 
                             int off_on_switch)
{
        struct raw1394_request req;

        CLEAR_REQ(&req);

        req.type = RAW1394_REQ_RESET_NOTIFY;
        req.generation = handle->generation;
        req.misc = off_on_switch;

        if (write(handle->fd, &req, sizeof(req)) < 0) return -1;

        return 0; /* success */
}

int raw1394_update_config_rom(raw1394handle_t handle, const quadlet_t
        *new_rom, size_t size, unsigned char rom_version)
{
        struct raw1394_request req;
        int status;

        CLEAR_REQ(&req);

        req.type = RAW1394_REQ_UPDATE_ROM;
        req.sendb = (unsigned long) new_rom;
        req.length = size;
        req.misc = rom_version;
        req.recvb = (unsigned long) &status;

        if (write(handle->fd, &req, sizeof(req)) < 0) return -8;

        return status;
}

int raw1394_get_config_rom(raw1394handle_t handle, quadlet_t *buffer,
        size_t buffersize, size_t *rom_size, unsigned char *rom_version)
{
        struct raw1394_request req;
        int status;

        CLEAR_REQ(&req);

        req.type = RAW1394_REQ_GET_ROM;
        req.recvb = (unsigned long) buffer;
        req.length = buffersize;
        req.tag = (unsigned long) rom_size;
        req.address = (unsigned long) rom_version;
        req.sendb = (unsigned long) &status;

        if (write(handle->fd, &req, sizeof(req)) < 0) return -8;

        return status;
}

int raw1394_bandwidth_modify (raw1394handle_t handle, unsigned int bandwidth,
	enum raw1394_modify_mode mode)
{
        quadlet_t buffer, compare, swap, new;
        int retry = 3;
        int result;
        
        if (bandwidth == 0)
                return 0;
        
        /* Reading current bandwidth usage from IRM. */
        result = raw1394_read (handle, raw1394_get_irm_id (handle),
                CSR_REGISTER_BASE + CSR_BANDWIDTH_AVAILABLE,
                sizeof (quadlet_t), &buffer);
        if (result < 0)
                return -1;
  
        buffer = ntohl (buffer);
        compare = buffer;
  
        while (retry > 0) {
                if (mode == RAW1394_MODIFY_ALLOC ) {
                        if (compare < bandwidth) {
                                return -1;
                        }
  
                        swap = compare - bandwidth;
                }
                else {
                        swap = compare + bandwidth;
  
                        if( swap > MAXIMUM_BANDWIDTH ) {
                                swap = MAXIMUM_BANDWIDTH;
                        }
                }
                
                result = raw1394_lock (handle, raw1394_get_irm_id (handle),
                                        CSR_REGISTER_BASE + CSR_BANDWIDTH_AVAILABLE, 
                                        RAW1394_EXTCODE_COMPARE_SWAP, ntohl(swap), ntohl(compare),
                                        &new);
                if (result < 0)
                        return -1;
                
                new = ntohl (new);
                
                if (new != compare) {
                        compare = new;
                        retry--;
                        if ( retry == 0 )
                                return -1;
                }
                else {
                        /* Success. */
                        retry = 0;
                        return 0;
                }
        }
  
        return 0;
}

int raw1394_channel_modify (raw1394handle_t handle, unsigned int channel,
	enum raw1394_modify_mode mode)
{
        quadlet_t buffer;
        int result;
        nodeaddr_t addr = CSR_REGISTER_BASE;
        unsigned int c = channel;
        quadlet_t compare, swap = 0, new;
        
        if (c > 31 && c < 64) {
                addr += CSR_CHANNELS_AVAILABLE_LO;
                c -= 32;
        } else if (c < 64)
                addr += CSR_CHANNELS_AVAILABLE_HI;
        else
                return -1;
        c = 31 - c;
  
        result = raw1394_read (handle, raw1394_get_irm_id (handle), addr, 
                sizeof (quadlet_t), &buffer);
        if (result < 0)
                return -1;
        
        buffer = ntohl (buffer);
  
        if ( mode == RAW1394_MODIFY_ALLOC ) {
                if( (buffer & (1 << c)) == 0 )
                        return -1;
                swap = htonl (buffer & ~(1 << c));
        }
        else if ( mode == RAW1394_MODIFY_FREE ) {
                if ( (buffer & (1 << c)) != 0 )
                        return -1;
                swap = htonl (buffer | (1 << c));
        }
  
        compare = htonl (buffer);
  
        result = raw1394_lock (handle, raw1394_get_irm_id (handle), addr,
                                RAW1394_EXTCODE_COMPARE_SWAP, swap, compare, &new);
        if ( (result < 0) || (new != compare) )
                return -1;
  
        return 0;
}

