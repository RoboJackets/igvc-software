/*
 * libraw1394 - library for raw access to the 1394 bus with the Linux subsystem.
 *
 * Copyright (C) 1999,2000,2001,2002 Andreas Bombe
 *        new ISO API by Dan Maas
 *
 * This library is licensed under the GNU Lesser General Public License (LGPL),
 * version 2.1 or later. See the file COPYING.LIB in the distribution for
 * details.
 */

#include <config.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <malloc.h>

#include "raw1394.h"
#include "kernel-raw1394.h"
#include "raw1394_private.h"

/* old ISO API - kept for backwards compatibility */

static int do_iso_listen(struct raw1394_handle *handle, int channel)
{
        struct sync_cb_data sd = { 0, 0 };
        struct raw1394_reqhandle rh = { (req_callback_t)_raw1394_sync_cb, &sd };
        int err;
        struct raw1394_request req;

        CLEAR_REQ(&req);
        req.type = RAW1394_REQ_ISO_LISTEN;
        req.generation = handle->generation;
        req.misc = channel;
        req.tag = ptr2int(&rh);
        req.recvb = ptr2int(handle->buffer);
        req.length = HBUF_SIZE;

        err = write(handle->fd, &req, sizeof(req));
        while (!sd.done) {
                if (err < 0) return err;
                err = raw1394_loop_iterate(handle);
        }

        switch (sd.errcode) {
        case RAW1394_ERROR_ALREADY:
                errno = EALREADY;
                return -1;

        case RAW1394_ERROR_INVALID_ARG:
                errno = EINVAL;
                return -1;

        default:
                errno = 0;
                return sd.errcode;
        }
}

int raw1394_start_iso_rcv(struct raw1394_handle *handle, unsigned int channel)
{
        if (channel > 63) {
                errno = EINVAL;
                return -1;
        }

        return do_iso_listen(handle, channel);
}

int raw1394_stop_iso_rcv(struct raw1394_handle *handle, unsigned int channel)
{
        if (channel > 63) {
                errno = EINVAL;
                return -1;
        }

        return do_iso_listen(handle, ~channel);
}



/* new ISO API */


/* reset the dropped counter each time it is seen */
static unsigned int _raw1394_iso_dropped(raw1394handle_t handle)
{
	unsigned int retval = handle->iso_packets_dropped;
	handle->iso_packets_dropped = 0;
	return retval;
}


/* common code for iso_xmit_init and iso_recv_init */
static int do_iso_init(raw1394handle_t handle,
		       unsigned int buf_packets,
		       unsigned int max_packet_size,
		       int channel,
		       enum raw1394_iso_speed speed,
		       enum raw1394_iso_dma_recv_mode mode,
		       int irq_interval,
		       int cmd)
{
	unsigned int stride;

	/* already initialized? */
	if(handle->iso_mode != ISO_INACTIVE)
		return -1;

	/* choose a power-of-two stride for the packet data buffer,
	   so that an even number of packets fits on one page */
	for(stride = 4; stride < max_packet_size; stride *= 2);

	if(stride > getpagesize()) {
		errno = ENOMEM;
		return -1;
	}

	handle->iso_buf_stride = stride;

	handle->iso_status.config.data_buf_size = stride * buf_packets;
	handle->iso_status.config.buf_packets = buf_packets;
	handle->iso_status.config.channel = channel;
	handle->iso_status.config.speed = speed;
	handle->iso_status.config.irq_interval = irq_interval;
	handle->iso_status.config.dma_mode = mode;

	if(ioctl(handle->fd, cmd, &handle->iso_status))
		return -1;

	/* mmap the DMA buffer */
	/* (we assume the kernel sets buf_size to an even number of pages) */
	handle->iso_buffer = mmap(NULL,
				  handle->iso_status.config.data_buf_size,
				  PROT_READ | PROT_WRITE,
				  MAP_SHARED, handle->fd, 0);

	if(handle->iso_buffer == (unsigned char*) MAP_FAILED) {
		handle->iso_buffer = NULL;
		ioctl(handle->fd, RAW1394_IOC_ISO_SHUTDOWN, 0);
		return -1;
	}

	handle->iso_status.overflows = 0;
	handle->iso_packets_dropped = 0;

	handle->iso_xmit_handler = NULL;
	handle->iso_recv_handler = NULL;

	handle->iso_state = ISO_STOP;

	return 0;			
}

int raw1394_iso_xmit_init(raw1394handle_t handle,
			  raw1394_iso_xmit_handler_t handler,
			  unsigned int buf_packets,
			  unsigned int max_packet_size,
			  unsigned char channel,
			  enum raw1394_iso_speed speed,
			  int irq_interval)
{
	if (do_iso_init(handle, buf_packets, max_packet_size, channel, speed, RAW1394_DMA_DEFAULT,
		       irq_interval, RAW1394_IOC_ISO_XMIT_INIT))
		return -1;

	handle->iso_mode = ISO_XMIT;
	handle->iso_xmit_handler = handler;
	handle->next_packet = 0;

	return 0;
}

int raw1394_iso_recv_init(raw1394handle_t handle,
			  raw1394_iso_recv_handler_t handler,
			  unsigned int buf_packets,
			  unsigned int max_packet_size,
			  unsigned char channel,
		          enum raw1394_iso_dma_recv_mode mode,
			  int irq_interval)
{
	/* any speed will work */
	if (do_iso_init(handle, buf_packets, max_packet_size, channel, RAW1394_ISO_SPEED_100, mode,
		       irq_interval, RAW1394_IOC_ISO_RECV_INIT))
		return -1;

	handle->iso_mode = ISO_RECV;
	handle->iso_recv_handler = handler;
	return 0;
}

int raw1394_iso_multichannel_recv_init(raw1394handle_t handle,
				       raw1394_iso_recv_handler_t handler,
				       unsigned int buf_packets,
				       unsigned int max_packet_size,
				       int irq_interval)
{
	/* any speed will work */
	if (do_iso_init(handle, buf_packets, max_packet_size, -1, RAW1394_ISO_SPEED_100,
			RAW1394_DMA_BUFFERFILL,
		       irq_interval, RAW1394_IOC_ISO_RECV_INIT))
		return -1;

	handle->iso_mode = ISO_RECV;
	handle->iso_recv_handler = handler;
	return 0;
}

int raw1394_iso_recv_listen_channel(raw1394handle_t handle, unsigned char channel)
{
	if (handle->iso_mode != ISO_RECV) {
		errno = EINVAL;
		return -1;
	}

	return ioctl(handle->fd, RAW1394_IOC_ISO_RECV_LISTEN_CHANNEL, channel);
}

int raw1394_iso_recv_unlisten_channel(raw1394handle_t handle, unsigned char channel)
{
	if (handle->iso_mode != ISO_RECV) {
		errno = EINVAL;
		return -1;
	}

	return ioctl(handle->fd, RAW1394_IOC_ISO_RECV_UNLISTEN_CHANNEL, channel);
}

int raw1394_iso_recv_flush(raw1394handle_t handle)
{
	if (handle->iso_mode != ISO_RECV) {
		errno = EINVAL;
		return -1;
	}

	return ioctl(handle->fd, RAW1394_IOC_ISO_RECV_FLUSH, 0);
}

int raw1394_iso_recv_set_channel_mask(raw1394handle_t handle, u_int64_t mask)
{
	if (handle->iso_mode != ISO_RECV) {
		errno = EINVAL;
		return -1;
	}

	return ioctl(handle->fd, RAW1394_IOC_ISO_RECV_SET_CHANNEL_MASK, (void*) &mask);
}

int raw1394_iso_recv_start(raw1394handle_t handle, int start_on_cycle, int tag_mask, int sync)
{
	int args[3];

	if(handle->iso_mode != ISO_RECV) {
		errno = EINVAL;
		return -1;
	}

	args[0] = start_on_cycle;
	args[1] = tag_mask;
	args[2] = sync;

	if(ioctl(handle->fd, RAW1394_IOC_ISO_RECV_START, &args[0]))
		return -1;

	handle->iso_state = ISO_GO;
	return 0;
}


static int _raw1394_iso_xmit_queue_packets(raw1394handle_t handle)
{
	struct raw1394_iso_status *stat = &handle->iso_status;
	struct raw1394_iso_packets packets;
	int retval = -1;
	int stop_sync = 0;

	if(handle->iso_mode != ISO_XMIT) {
		errno = EINVAL;
		goto out;
	}

	/* we could potentially send up to stat->n_packets packets */
	packets.n_packets = 0;
	packets.infos = malloc(stat->n_packets * sizeof(struct raw1394_iso_packet_info));
	if(packets.infos == NULL)
		goto out;

	while(stat->n_packets > 0) {
		enum raw1394_iso_disposition disp;
		unsigned int len;
		
		struct raw1394_iso_packet_info *info = &packets.infos[packets.n_packets];

		info->offset = handle->iso_buf_stride * handle->next_packet;
		
		/* call handler */
		disp = handle->iso_xmit_handler(handle,
						handle->iso_buffer + info->offset,
						&len,
						&info->tag, &info->sy,
						stat->xmit_cycle,
						_raw1394_iso_dropped(handle));
		info->len = len;
		
		/* advance packet cursors and cycle counter */
		stat->n_packets--;
		handle->next_packet = (handle->next_packet + 1) % stat->config.buf_packets;
		if(stat->xmit_cycle != -1)
			stat->xmit_cycle = (stat->xmit_cycle + 1) % 8000;
		packets.n_packets++;

		if(disp == RAW1394_ISO_DEFER) {
			/* queue an event so that we don't hang in the next read() */
			if(ioctl(handle->fd, RAW1394_IOC_ISO_QUEUE_ACTIVITY, 0))
				goto out_produce;
			break;
		} else if(disp == RAW1394_ISO_STOP) {
			stop_sync = 1;
			break;
		} else if(disp == RAW1394_ISO_STOP_NOSYNC) {
			raw1394_iso_stop(handle);
			break;
		} else if(disp == RAW1394_ISO_ERROR) {
			goto out_produce;
		}
	}

	/* success */
	retval = 0;

out_produce:
	if(packets.n_packets > 0) {
		if(ioctl(handle->fd, RAW1394_IOC_ISO_XMIT_PACKETS, &packets))
			retval = -1;
	}
	free(packets.infos);
out:
	if(stop_sync) {
		if(raw1394_iso_xmit_sync(handle))
			return -1;
		raw1394_iso_stop(handle);
	}
	
	return retval;
}

int raw1394_iso_xmit_write(raw1394handle_t handle, unsigned char *data, unsigned int len,
			   unsigned char tag, unsigned char sy)
{
	struct raw1394_iso_status *stat = &handle->iso_status;
	struct raw1394_iso_packets packets;
	struct raw1394_iso_packet_info info;

	if(handle->iso_mode != ISO_XMIT || handle->iso_xmit_handler != NULL) {
		errno = EINVAL;
		return -1;
	}

	/* wait until buffer space is available */
	while(handle->iso_status.n_packets == 0) {
		/* if the file descriptor has been set non-blocking,
		   return immediately */
		if(fcntl(handle->fd, F_GETFL) & O_NONBLOCK) {
			errno = EAGAIN;
			return -1;
		}
			
		if(raw1394_loop_iterate(handle)) {
			return -1;
		}
	}

	/* copy the data to the packet buffer */
	info.offset = handle->next_packet * handle->iso_buf_stride;
	info.len = len;
	info.tag = tag;
	info.sy = sy;
	
	memcpy(handle->iso_buffer + info.offset, data, len);
	
	packets.n_packets = 1;
	packets.infos = &info;

	if(ioctl(handle->fd, RAW1394_IOC_ISO_XMIT_PACKETS, &packets))
		return -1;

	stat->n_packets--;
	handle->next_packet = (handle->next_packet + 1) % stat->config.buf_packets;
	if(stat->xmit_cycle != -1)
		stat->xmit_cycle = (stat->xmit_cycle + 1) % 8000;

	return 0;
}

int raw1394_iso_xmit_start(raw1394handle_t handle, int start_on_cycle, int prebuffer_packets)
{
	int args[2];

	if(handle->iso_mode != ISO_XMIT) {
		errno = EINVAL;
		return -1;
	}

	args[0] = start_on_cycle;
	args[1] = prebuffer_packets;

	if(ioctl(handle->fd, RAW1394_IOC_ISO_XMIT_START, &args[0]))
		return -1;

	handle->iso_state = ISO_GO;
	return 0;
}

int raw1394_iso_xmit_sync(raw1394handle_t handle)
{
	if(handle->iso_mode != ISO_XMIT) {
		errno = EINVAL;
		return -1;
	}
	return ioctl(handle->fd, RAW1394_IOC_ISO_XMIT_SYNC, 0);
}

void raw1394_iso_stop(raw1394handle_t handle)
{
	if(handle->iso_mode == ISO_INACTIVE) {
		return;
	}

	ioctl(handle->fd, RAW1394_IOC_ISO_XMIT_RECV_STOP, 0);
	handle->iso_state = ISO_STOP;
}

void raw1394_iso_shutdown(raw1394handle_t handle)
{
	if(handle->iso_buffer) {
		munmap(handle->iso_buffer, handle->iso_status.config.data_buf_size);
		handle->iso_buffer = NULL;
	}
	
	if(handle->iso_mode != ISO_INACTIVE) {
		raw1394_iso_stop(handle);
		ioctl(handle->fd, RAW1394_IOC_ISO_SHUTDOWN, 0);
	}

	handle->iso_mode = ISO_INACTIVE;
}

static int _raw1394_iso_recv_packets(raw1394handle_t handle)
{
	struct raw1394_iso_status *stat = &handle->iso_status;
	struct raw1394_iso_packets packets;

	int retval = -1, packets_done = 0;

	if(handle->iso_mode != ISO_RECV) {
		errno = EINVAL;
		return -1;
	}
	
	/* ask the kernel to fill an array with packet info structs */
	packets.n_packets = stat->n_packets;
	packets.infos = malloc(packets.n_packets * sizeof(struct raw1394_iso_packet_info));
	if(packets.infos == NULL)
		goto out;

	if(ioctl(handle->fd, RAW1394_IOC_ISO_RECV_PACKETS, &packets) < 0)
		goto out_free;

	while(stat->n_packets > 0) {
		struct raw1394_iso_packet_info *info;
		enum raw1394_iso_disposition disp;

		info = &packets.infos[packets_done];

		/* call handler */
		disp = handle->iso_recv_handler(handle,
						handle->iso_buffer + info->offset,
						info->len, info->channel,
						info->tag, info->sy,
						info->cycle,
						_raw1394_iso_dropped(handle));

		/* advance packet cursors */
		stat->n_packets--;
		packets_done++;
		
		if(disp == RAW1394_ISO_DEFER) {
			/* queue an event so that we don't hang in the next read() */
			if(ioctl(handle->fd, RAW1394_IOC_ISO_QUEUE_ACTIVITY, 0))
				goto out_consume;
			break;
		} else if(disp == RAW1394_ISO_STOP || disp == RAW1394_ISO_STOP_NOSYNC) {
			raw1394_iso_stop(handle);
			break;
		} else if(disp == RAW1394_ISO_ERROR) {
			goto out_consume;
		}
	}

	/* success */
	retval = 0;

out_consume:
	if(packets_done > 0) {
		if(ioctl(handle->fd, RAW1394_IOC_ISO_RECV_RELEASE_PACKETS, packets_done))
			retval = -1;
	}
out_free:
	free(packets.infos);
out:	
	return retval;
}

/* run the ISO state machine; called from raw1394_loop_iterate()  */
int _raw1394_iso_iterate(raw1394handle_t handle)
{
	int err;

	if(handle->iso_mode == ISO_INACTIVE)
		return 0;

	err = ioctl(handle->fd, RAW1394_IOC_ISO_GET_STATUS, &handle->iso_status);
	if(err != 0)
		return err;

	handle->iso_packets_dropped += handle->iso_status.overflows;

	if(handle->iso_state == ISO_GO) {
		if(handle->iso_mode == ISO_XMIT) {
			if(handle->iso_xmit_handler) {
				return _raw1394_iso_xmit_queue_packets(handle);
			}
		}

		if(handle->iso_mode == ISO_RECV) {
			if(handle->iso_recv_handler) {
				return _raw1394_iso_recv_packets(handle);
			}
		}
	}

	return 0;
}
