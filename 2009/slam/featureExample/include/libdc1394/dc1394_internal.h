/*
 * 1394-Based Digital Camera Control Library
 * Copyright (C) 2000 SMART Technologies Inc.
 *
 * Written by Gord Peters <GordPeters@smarttech.com>
 * Additions by Chris Urmson <curmson@ri.cmu.edu>
 * Additions by Damien Douxchamps <douxchamps@ieee.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef _DC1394_INTERNAL_H
#define _DC1394_INTERNAL_H

#include <libraw1394/raw1394.h>


/* Definitions which application developers shouldn't care about */
#define CONFIG_ROM_BASE             0xFFFFF0000000ULL

#define ON_VALUE                    0x80000000UL
#define OFF_VALUE                   0x00000000UL

/* Maximum number of write/read retries */
#define MAX_RETRIES                 20

/* A hard compiled factor that makes sure async read and writes don't happen
   too fast */
#define SLOW_DOWN                   20

/* transaction acknowldegements (this should be in the raw1394 headers) */
#define ACK_COMPLETE                0x0001U
#define ACK_PENDING                 0x0002U
#define ACK_LOCAL                   0x0010U

/*Response codes (this should be in the raw1394 headers) */
//not currently used
#define RESP_COMPLETE               0x0000U
#define RESP_SONY_HACK              0x000fU

/* Internal functions required by two different source files */
int
GetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
			 octlet_t offset, quadlet_t *value);

int
GetCameraAdvControlRegister(raw1394handle_t handle, nodeid_t node,
			    octlet_t offset, quadlet_t *value);

int
SetCameraAdvControlRegister(raw1394handle_t handle, nodeid_t node,
			    octlet_t offset, quadlet_t value);

int
_dc1394_dma_basic_setup(int channel, int num_dma_buffers,
                        dc1394_cameracapture *camera);
						
						
typedef struct __dc1394_camerahandle
{
  int       port;
  octlet_t  ccr_base;
  octlet_t  adv_csr;    /* Offset of registers for advanced features */
  int       sw_version;
  octlet_t  format7_csr[NUM_MODE_FORMAT7];

} dc1394_camerahandle;
						
#endif /* _DC1394_INTERNAL_H */
