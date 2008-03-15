
#ifndef _RAW1394_PRIVATE_H
#define _RAW1394_PRIVATE_H

#define HBUF_SIZE      8192
#define ARM_REC_LENGTH 4096 
#define MAXIMUM_BANDWIDTH 4915

struct raw1394_handle {
        int fd;
        int protocol_version;
        unsigned int generation;

        nodeid_t local_id;
        int num_of_nodes;
        nodeid_t irm_id;

        raw1394_errcode_t err;
        void *userdata;

        bus_reset_handler_t bus_reset_handler;
        tag_handler_t     tag_handler;
        arm_tag_handler_t arm_tag_handler;
        fcp_handler_t     fcp_handler;
        iso_handler_t     iso_handler[64];

	/* new ISO API */

	/* memory mapping of the DMA buffer */
	unsigned char *iso_buffer;
	enum { ISO_INACTIVE, ISO_XMIT, ISO_RECV } iso_mode;
	enum { ISO_STOP, ISO_GO } iso_state;

	/* iso XMIT only: */
	unsigned int iso_buf_stride; /* offset between successive packets */
	unsigned int next_packet; /* index of next packet to be transmitted */

	/* status buffer, updated from _raw1394_iso_iterate() */
	struct raw1394_iso_status iso_status;
	unsigned int iso_packets_dropped;

	/* user-supplied handlers */
	raw1394_iso_xmit_handler_t iso_xmit_handler;
	raw1394_iso_recv_handler_t iso_recv_handler;

        quadlet_t buffer[HBUF_SIZE/4]; /* 2048 */
};

struct sync_cb_data {
        int done;
        int errcode;
};

int _raw1394_sync_cb(struct raw1394_handle*, struct sync_cb_data*, int);
int _raw1394_iso_iterate(raw1394handle_t handle);

#define CLEAR_REQ(reqp) memset((reqp), 0, sizeof(struct raw1394_request))

#if SIZEOF_VOID_P == 4
#define int2ptr(x) ((void *)(__u32)x)
#define ptr2int(x) ((__u64)(__u32)x)
#else
#define int2ptr(x) ((void *)x)
#define ptr2int(x) ((__u64)x)
#endif

#endif /* _RAW1394_PRIVATE_H */
