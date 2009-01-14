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
#ifndef _DC1394_CAMERA_CONTROL_H
#define _DC1394_CAMERA_CONTROL_H

#include <stddef.h>
#include <sys/types.h>
#include <libraw1394/raw1394.h>
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
 
/* Enumeration of data speeds */
enum
{
    SPEED_100= 0,
    SPEED_200,
    SPEED_400,
    SPEED_800,
    SPEED_1600,
    SPEED_3200
};
#define SPEED_MIN                   SPEED_100
#define SPEED_MAX                   SPEED_3200
#define NUM_SPEEDS                  (SPEED_MAX - SPEED_MIN + 1)

/* Enumeration of camera framerates */
enum 
{
    FRAMERATE_1_875= 32,
    FRAMERATE_3_75,
    FRAMERATE_7_5,
    FRAMERATE_15,
    FRAMERATE_30,
    FRAMERATE_60,
    FRAMERATE_120,
    FRAMERATE_240
};
#define FRAMERATE_MIN               FRAMERATE_1_875
#define FRAMERATE_MAX               FRAMERATE_240
#define NUM_FRAMERATES              (FRAMERATE_MAX - FRAMERATE_MIN + 1)

/* Enumeration of camera modes for Format_0 */
enum 
{
    MODE_160x120_YUV444= 64,
    MODE_320x240_YUV422,
    MODE_640x480_YUV411,
    MODE_640x480_YUV422,
    MODE_640x480_RGB,
    MODE_640x480_MONO,
    MODE_640x480_MONO16
};
#define MODE_FORMAT0_MIN	    MODE_160x120_YUV444
#define MODE_FORMAT0_MAX	    MODE_640x480_MONO16
#define NUM_FORMAT0_MODES	    (MODE_FORMAT0_MAX - MODE_FORMAT0_MIN + 1)

/* Enumeration of camera modes for Format_1 */
enum 
{
    MODE_800x600_YUV422= 96,
    MODE_800x600_RGB,
    MODE_800x600_MONO,
    MODE_1024x768_YUV422,
    MODE_1024x768_RGB,
    MODE_1024x768_MONO,
    MODE_800x600_MONO16,
    MODE_1024x768_MONO16
};
#define MODE_FORMAT1_MIN	    MODE_800x600_YUV422
#define MODE_FORMAT1_MAX	    MODE_1024x768_MONO16
#define NUM_FORMAT1_MODES	    (MODE_FORMAT1_MAX - MODE_FORMAT1_MIN + 1)


/* Enumeration of camera modes for Format_2 */
enum 
{
    MODE_1280x960_YUV422= 128,
    MODE_1280x960_RGB,
    MODE_1280x960_MONO,
    MODE_1600x1200_YUV422,
    MODE_1600x1200_RGB,
    MODE_1600x1200_MONO,
    MODE_1280x960_MONO16,
    MODE_1600x1200_MONO16
};
#define MODE_FORMAT2_MIN	    MODE_1280x960_YUV422
#define MODE_FORMAT2_MAX	    MODE_1600x1200_MONO16
#define NUM_FORMAT2_MODES	    (MODE_FORMAT2_MAX - MODE_FORMAT2_MIN + 1)

/* Enumeration of camera modes for Format_6 */
enum 
{
    MODE_EXIF= 256
};
#define MODE_FORMAT6_MIN            MODE_EXIF
#define MODE_FORMAT6_MAX            MODE_EXIF
#define NUM_FORMAT6_MODES           (MODE_FORMAT6_MAX - MODE_FORMAT6_MIN + 1)

/* Enumeration of camera modes for Format_7 */
enum {
    MODE_FORMAT7_0= 288,
    MODE_FORMAT7_1,
    MODE_FORMAT7_2,
    MODE_FORMAT7_3,
    MODE_FORMAT7_4,
    MODE_FORMAT7_5,
    MODE_FORMAT7_6,
    MODE_FORMAT7_7
};
#define MODE_FORMAT7_MIN            MODE_FORMAT7_0
#define MODE_FORMAT7_MAX            MODE_FORMAT7_7
#define NUM_MODE_FORMAT7            (MODE_FORMAT7_MAX - MODE_FORMAT7_MIN + 1)

/* Enumeration of Format_7 color modes */
enum {
    COLOR_FORMAT7_MONO8= 320,
    COLOR_FORMAT7_YUV411,
    COLOR_FORMAT7_YUV422,
    COLOR_FORMAT7_YUV444,
    COLOR_FORMAT7_RGB8,
    COLOR_FORMAT7_MONO16,
    COLOR_FORMAT7_RGB16,
    COLOR_FORMAT7_MONO16S,
    COLOR_FORMAT7_RGB16S,
    COLOR_FORMAT7_RAW8,
    COLOR_FORMAT7_RAW16
};
#define COLOR_FORMAT7_MIN           COLOR_FORMAT7_MONO8
#define COLOR_FORMAT7_MAX           COLOR_FORMAT7_RAW16
#define NUM_COLOR_FORMAT7           (COLOR_FORMAT7_MAX - COLOR_FORMAT7_MIN + 1)

/* Enumeration of trigger modes */
enum {
    TRIGGER_MODE_0= 352,
    TRIGGER_MODE_1,
    TRIGGER_MODE_2,
    TRIGGER_MODE_3
};
#define TRIGGER_MODE_MIN            TRIGGER_MODE_0
#define TRIGGER_MODE_MAX            TRIGGER_MODE_3
#define NUM_TRIGGER_MODE            (TRIGGER_MODE_MAX - TRIGGER_MODE_MIN + 1)

/* Enumeration of camera image formats */
enum {
    FORMAT_VGA_NONCOMPRESSED= 384,
    FORMAT_SVGA_NONCOMPRESSED_1,
    FORMAT_SVGA_NONCOMPRESSED_2,
    /* 3 reserved formats */
    FORMAT_STILL_IMAGE= 390,
    FORMAT_SCALABLE_IMAGE_SIZE
};
#define FORMAT_MIN                  FORMAT_VGA_NONCOMPRESSED
#define FORMAT_MAX                  FORMAT_SCALABLE_IMAGE_SIZE
#define NUM_FORMATS                 (FORMAT_MAX - FORMAT_MIN + 1)

/* Enumeration of camera features */
enum 
{
    FEATURE_BRIGHTNESS= 416,
    FEATURE_EXPOSURE,
    FEATURE_SHARPNESS,
    FEATURE_WHITE_BALANCE,
    FEATURE_HUE,
    FEATURE_SATURATION,
    FEATURE_GAMMA,
    FEATURE_SHUTTER,
    FEATURE_GAIN,
    FEATURE_IRIS,
    FEATURE_FOCUS,
    FEATURE_TEMPERATURE,
    FEATURE_TRIGGER,
    FEATURE_TRIGGER_DELAY,
    FEATURE_WHITE_SHADING,
    FEATURE_FRAME_RATE,
    /* 16 reserved features */
    FEATURE_ZOOM,
    FEATURE_PAN,
    FEATURE_TILT,
    FEATURE_OPTICAL_FILTER,
    /* 12 reserved features */
    FEATURE_CAPTURE_SIZE,
    FEATURE_CAPTURE_QUALITY
    /* 14 reserved features */
};
#define FEATURE_MIN                 FEATURE_BRIGHTNESS
#define FEATURE_MAX                 FEATURE_CAPTURE_QUALITY
#define NUM_FEATURES                (FEATURE_MAX - FEATURE_MIN + 1)

/* Operation modes */
enum 
{
  OPERATION_MODE_LEGACY = 480,
  OPERATION_MODE_1394B
};

/* Format 7 sensor layouts*/
enum
{
  COLOR_FILTER_FORMAT7_RGGB = 512,
  COLOR_FILTER_FORMAT7_GBRG,
  COLOR_FILTER_FORMAT7_GRBG,
  COLOR_FILTER_FORMAT7_BGGR
};
#define COLOR_FILTER_FORMAT7_MIN                 COLOR_FILTER_FORMAT7_RGGB
#define COLOR_FILTER_FORMAT7_MAX                 COLOR_FILTER_FORMAT7_BGGR
#define NUM_COLOR_FILTER_FORMAT7                (COLOR_FILTER_FORMAT7_MAX - COLOR_FILTER_FORMAT7_MIN + 1)

/* IIDC versions*/
enum
{
  IIDC_VERSION_1_04 = 544,
  IIDC_VERSION_1_20,
  IIDC_VERSION_PTGREY,
  IIDC_VERSION_1_30,
  IIDC_VERSION_1_31,
  IIDC_VERSION_1_32,
  IIDC_VERSION_1_33,
  IIDC_VERSION_1_34,
  IIDC_VERSION_1_35,
  IIDC_VERSION_1_36,
  IIDC_VERSION_1_37,
  IIDC_VERSION_1_38,
  IIDC_VERSION_1_39
};
#define IIDC_VERSION_MIN                 IIDC_VERSION_1_04
#define IIDC_VERSION_MAX                 IIDC_VERSION_1_39
#define NUM_IIDC_VERSION                (IIDC_VERSION_MAX - IIDC_VERSION_MIN + 1)

/* Maximum number of characters in vendor and model strings */
#define MAX_CHARS                   32

/* Return values for visible functions*/
#define DC1394_SUCCESS               1
#define DC1394_FAILURE              -1
#define DC1394_NO_FRAME             -2
#define DC1394_NO_CAMERA            0xffff

/* Parameter flags for dc1394_setup_format7_capture() */
#define QUERY_FROM_CAMERA -1
#define USE_MAX_AVAIL     -2
#define USE_RECOMMENDED   -3

/* The video1394 policy: blocking (wait for a frame forever)
   or polling (returns if no frames in buffer */
typedef enum
{ 
  VIDEO1394_WAIT=0,
  VIDEO1394_POLL
} dc1394videopolicy_t;

/* Yet another boolean data type */
typedef enum
{
    DC1394_FALSE= 0,
    DC1394_TRUE
} dc1394bool_t;

/* Camera structure */
typedef struct __dc1394_camerainfo
{
    raw1394handle_t handle;
    nodeid_t id;
    octlet_t ccr_offset;
    u_int64_t euid_64;
    char vendor[MAX_CHARS + 1];
    char model[MAX_CHARS + 1];
} dc1394_camerainfo;

typedef struct __dc1394_cam_cap_struct 
{
    nodeid_t node;
    int channel;
    int frame_rate;
    int frame_width, frame_height;
    int * capture_buffer;
    int quadlets_per_frame;
    int quadlets_per_packet;
    /* components needed for the DMA based video capture */
    const unsigned char * dma_ring_buffer;
    int dma_buffer_size;
    int dma_frame_size;
    int num_dma_buffers;
    int dma_last_buffer;
    int num_dma_buffers_behind;
    const char * dma_device_file;
    int dma_fd;
    int port;
    struct timeval filltime;
    int drop_frames;
} dc1394_cameracapture ;

typedef struct __dc1394_misc_info
{
  unsigned int format;
  unsigned int mode;
  unsigned int framerate;

  dc1394bool_t is_iso_on;
  unsigned int iso_channel;
  unsigned int iso_speed;

  unsigned int mem_channel_number;
  unsigned int save_channel;
  unsigned int load_channel;

  dc1394bool_t bmode_capable;
  dc1394bool_t one_shot_capable;
  dc1394bool_t multi_shot_capable;

} dc1394_miscinfo;

typedef struct __dc1394_feature_info_struct 
{
    unsigned int feature_id;
    dc1394bool_t available;
    dc1394bool_t one_push;
    dc1394bool_t absolute_capable;
    dc1394bool_t readout_capable;
    dc1394bool_t on_off_capable;
    dc1394bool_t auto_capable;
    dc1394bool_t manual_capable;
    dc1394bool_t polarity_capable;
    dc1394bool_t one_push_active;
    dc1394bool_t is_on;
    dc1394bool_t auto_active;
    char trigger_mode_capable_mask;
    int trigger_mode;
    dc1394bool_t trigger_polarity;
    int min;
    int max;
    int value;
    int BU_value;
    int RV_value;
    int B_value;
    int R_value;
    int G_value;
    int target_value;

    dc1394bool_t abs_control;
    float abs_value;
    float abs_max;
    float abs_min;

} dc1394_feature_info;

typedef struct __dc1394_feature_set_struct 
{
  dc1394_feature_info feature[NUM_FEATURES];
} dc1394_feature_set;

/* Feature descriptions */
extern const char *dc1394_feature_desc[NUM_FEATURES];

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************
 Direct register manipulation functions.
 Use with caution, this might wreak your camera.
 *****************************************************/
int
SetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
                         octlet_t offset, quadlet_t value);

int
GetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
                         octlet_t offset, quadlet_t *value);

/*****************************************************
 dc1394_get_camera_feature_set

 Collects the available features for the camera
 described by node and stores them in features.
*****************************************************/  
int 
dc1394_get_camera_feature_set(raw1394handle_t handle, nodeid_t node,
                              dc1394_feature_set *features);

/*****************************************************
 dc1394_get_camera_feature

 Stores the bounds and options associated with the
 feature described by feature->feature_id
*****************************************************/
int 
dc1394_get_camera_feature(raw1394handle_t handle, nodeid_t node,
                          dc1394_feature_info *feature);


/*****************************************************
 dc1394_get_camera_misc_info

 Collects other camera info registers
*****************************************************/
int 
dc1394_get_camera_misc_info(raw1394handle_t handle, nodeid_t node,
                            dc1394_miscinfo *info);


/*****************************************************
 dc1394_print_feature

 Displays the bounds and options of the given feature
*****************************************************/
void 
dc1394_print_feature(dc1394_feature_info *feature);

/*****************************************************
 dc1394_print_feature_set

 Displays the entire feature set stored in features
*****************************************************/
void 
dc1394_print_feature_set(dc1394_feature_set *features);
	

/*****************************************************
 dc1394_create_handle

 This creates a raw1394_handle.  Port should be 0
 unless you have multiple firewire cards in your
 system

 If a handle can't be created, it returns NULL

 IMPORTANT: you should NOT use raw1394_new_handle()
 to create a new handle, even if you use
 raw1394_set_port after. dc1394_create_handle has to
 setup other things and failing to use it will result
 in crashes.

*****************************************************/
raw1394handle_t 
dc1394_create_handle(int port);

/***************************************************
 dc1394_destroy_handle

 Destroy the handle, including the userdata 
***************************************************/
int
dc1394_destroy_handle(raw1394handle_t handle);


/*****************************************************
 dc1394_get_camera_nodes

 This returns the available cameras on the bus.

 If showCameras is set to 1, a description of the
 found cameras is printed.

 Returns -1 in numCameras and NULL from the call if
 there is a problem, otherwise the number of cameras
 and the nodeid_t array from the call
*****************************************************/
nodeid_t* 
dc1394_get_camera_nodes(raw1394handle_t handle, int *numCameras,
                        int showCameras);

#define dc1394_free_camera_nodes free

/*****************************************************
 dc1394_get_sorted_camera_nodes

 This returns the available cameras on the bus.

 It returns the node id's in the same index as the id
 specified the ids array contains a list of the low
 quadlet of the unique camera ids.

 If showCameras is set to 1, a description of the
 found cameras is printed.

 Returns -1 in numCameras and NULL from the call if
 there is a problem, otherwise the number of cameras
 and the nodeid_t array from the call
*****************************************************/
nodeid_t* 
dc1394_get_sorted_camera_nodes(raw1394handle_t handle,int numids, 
                               int *ids,int * numCameras,
                               int showCameras);


/* Initialize camera to factory default settings */
int
dc1394_init_camera(raw1394handle_t handle, nodeid_t node);


/* Determine if the given node is a camera */
int
dc1394_is_camera(raw1394handle_t handle, nodeid_t node, dc1394bool_t *value);

/* Determine the IIDC Specification version */
int
dc1394_get_sw_version(raw1394handle_t handle, nodeid_t node, int *version);

/* Get the camera information and print that structure*/
void 
dc1394_print_camera_info(dc1394_camerainfo *info); 

int
dc1394_get_camera_info(raw1394handle_t handle, nodeid_t node,
                       dc1394_camerainfo *info);


/* Functions for querying camera attributes */
int
dc1394_query_supported_formats(raw1394handle_t handle, nodeid_t node,
                               quadlet_t *value);
int
dc1394_query_supported_modes(raw1394handle_t handle, nodeid_t node,
                             unsigned int format, quadlet_t *value);
int
dc1394_query_supported_framerates(raw1394handle_t handle, nodeid_t node,
                                  unsigned int format, unsigned int mode,
                                  quadlet_t *value);
int
dc1394_query_revision(raw1394handle_t handle, nodeid_t node, int mode,
                      quadlet_t *value);
int
dc1394_query_basic_functionality(raw1394handle_t handle, nodeid_t node,
                                 quadlet_t *value);
/*
int
dc1394_query_feature_control(raw1394handle_t handle, nodeid_t node,
                             unsigned int feature, unsigned int *availability);
*/
int
dc1394_query_advanced_feature_offset(raw1394handle_t handle, nodeid_t node,
                                     quadlet_t *value);
int
dc1394_query_feature_characteristics(raw1394handle_t handle, nodeid_t node,
                                     unsigned int feature, quadlet_t *value);


/* Get/Set the framerate, mode, format, iso channel/speed for the video */
int
dc1394_get_video_framerate(raw1394handle_t handle, nodeid_t node,
                           unsigned int *framerate);
int
dc1394_set_video_framerate(raw1394handle_t handle, nodeid_t node,
                           unsigned int framerate);
int
dc1394_get_video_mode(raw1394handle_t handle, nodeid_t node,
                      unsigned int *mode);
int
dc1394_set_video_mode(raw1394handle_t handle, nodeid_t node,
                      unsigned int mode);
int
dc1394_get_video_format(raw1394handle_t handle, nodeid_t node,
                        unsigned int *format);
int
dc1394_set_video_format(raw1394handle_t handle, nodeid_t node,
                        unsigned int format);
int
dc1394_get_operation_mode(raw1394handle_t handle, nodeid_t node,
			  unsigned int *mode);
int
dc1394_set_operation_mode(raw1394handle_t handle, nodeid_t node,
			  unsigned int mode);
int
dc1394_get_iso_channel_and_speed(raw1394handle_t handle, nodeid_t node,
                                 unsigned int *channel, unsigned int *speed);
int
dc1394_set_iso_channel_and_speed(raw1394handle_t handle, nodeid_t node,
                                 unsigned int channel, unsigned int speed);

/* Turn camera on or off */
int
dc1394_camera_on(raw1394handle_t handle, nodeid_t node);
int
dc1394_camera_off(raw1394handle_t handle, nodeid_t node);


/* Start/stop isochronous data transmission */
int
dc1394_start_iso_transmission(raw1394handle_t handle, nodeid_t node);
int
dc1394_stop_iso_transmission(raw1394handle_t handle, nodeid_t node);
int
dc1394_get_iso_status(raw1394handle_t handle, nodeid_t node,
                      dc1394bool_t *is_on);


/* Turn one shot mode on or off */
int
dc1394_set_one_shot(raw1394handle_t handle, nodeid_t node);
int
dc1394_unset_one_shot(raw1394handle_t handle, nodeid_t node);


/* Turn multishot mode on or off */
int
dc1394_set_multi_shot(raw1394handle_t handle, nodeid_t node,
                      unsigned int numFrames);
int
dc1394_unset_multi_shot(raw1394handle_t handle, nodeid_t node);


int
dc1394_get_one_shot(raw1394handle_t handle, nodeid_t node, dc1394bool_t *is_on);

int
dc1394_get_multi_shot(raw1394handle_t handle, nodeid_t node, dc1394bool_t *is_on,
		      unsigned int *numFrames);

/* Get/Set the values of the various features on the camera */
int
dc1394_get_brightness(raw1394handle_t handle, nodeid_t node,
                      unsigned int *brightness);
int
dc1394_set_brightness(raw1394handle_t handle, nodeid_t node,
                      unsigned int brightness);
int
dc1394_get_exposure(raw1394handle_t handle, nodeid_t node,
                    unsigned int *exposure);
int
dc1394_set_exposure(raw1394handle_t handle, nodeid_t node,
                    unsigned int exposure);
int
dc1394_get_sharpness(raw1394handle_t handle, nodeid_t node,
                     unsigned int *sharpness);
int
dc1394_set_sharpness(raw1394handle_t handle, nodeid_t node,
                     unsigned int sharpness);
int
dc1394_get_white_balance(raw1394handle_t handle, nodeid_t node,
                         unsigned int *u_b_value, unsigned int *v_r_value);
int
dc1394_set_white_balance(raw1394handle_t handle, nodeid_t node,
                         unsigned int u_b_value, unsigned int v_r_value);

int
dc1394_get_hue(raw1394handle_t handle, nodeid_t node,
               unsigned int *hue);
int
dc1394_set_hue(raw1394handle_t handle, nodeid_t node,
               unsigned int hue);
int
dc1394_get_saturation(raw1394handle_t handle, nodeid_t node,
                      unsigned int *saturation);
int
dc1394_set_saturation(raw1394handle_t handle, nodeid_t node,
                      unsigned int saturation);
int
dc1394_get_gamma(raw1394handle_t handle, nodeid_t node,
                 unsigned int *gamma);
int
dc1394_set_gamma(raw1394handle_t handle, nodeid_t node,
                 unsigned int gamma);
int
dc1394_get_shutter(raw1394handle_t handle, nodeid_t node,
                   unsigned int *shutter);
int
dc1394_set_shutter(raw1394handle_t handle, nodeid_t node,
                   unsigned int shutter);
int
dc1394_get_gain(raw1394handle_t handle, nodeid_t node,
                unsigned int *gain);
int
dc1394_set_gain(raw1394handle_t handle, nodeid_t node,
                unsigned int gain);
int
dc1394_get_iris(raw1394handle_t handle, nodeid_t node,
                unsigned int *iris);
int
dc1394_set_iris(raw1394handle_t handle, nodeid_t node,
                unsigned int iris);
int
dc1394_get_focus(raw1394handle_t handle, nodeid_t node,
                 unsigned int *focus);
int
dc1394_set_focus(raw1394handle_t handle, nodeid_t node,
                 unsigned int focus);
int
dc1394_get_temperature(raw1394handle_t handle, nodeid_t node,
                       unsigned int *target_temperature,
		       unsigned int *temperature);
int
dc1394_set_temperature(raw1394handle_t handle, nodeid_t node,
                       unsigned int target_temperature);
int
dc1394_get_white_shading(raw1394handle_t handle, nodeid_t node,
                         unsigned int *r_value, unsigned int *g_value,
			 unsigned int *b_value);
int
dc1394_set_white_shading(raw1394handle_t handle, nodeid_t node,
			 unsigned int r_value, unsigned int g_value,
			 unsigned int b_value);
int
dc1394_get_trigger_delay(raw1394handle_t handle, nodeid_t node,
			 unsigned int *trigger_delay);
int
dc1394_set_trigger_delay(raw1394handle_t handle, nodeid_t node,
			 unsigned int trigger_delay);
int
dc1394_get_frame_rate(raw1394handle_t handle, nodeid_t node,
		      unsigned int *frame_rate);
int
dc1394_set_frame_rate(raw1394handle_t handle, nodeid_t node,
		      unsigned int frame_rate);
int
dc1394_get_trigger_mode(raw1394handle_t handle, nodeid_t node,
                        unsigned int *mode);
int
dc1394_set_trigger_mode(raw1394handle_t handle, nodeid_t node,
                        unsigned int mode);
int
dc1394_get_zoom(raw1394handle_t handle, nodeid_t node,
                unsigned int *zoom);
int
dc1394_set_zoom(raw1394handle_t handle, nodeid_t node,
                unsigned int zoom);
int
dc1394_get_pan(raw1394handle_t handle, nodeid_t node,
               unsigned int *pan);
int
dc1394_set_pan(raw1394handle_t handle, nodeid_t node,
               unsigned int pan);
int
dc1394_get_tilt(raw1394handle_t handle, nodeid_t node,
                unsigned int *tilt);
int
dc1394_set_tilt(raw1394handle_t handle, nodeid_t node,
                unsigned int tilt);
int
dc1394_get_optical_filter(raw1394handle_t handle, nodeid_t node,
                          unsigned int *optical_filter);
int
dc1394_set_optical_filter(raw1394handle_t handle, nodeid_t node,
                          unsigned int optical_filter);
int
dc1394_get_capture_size(raw1394handle_t handle, nodeid_t node,
                        unsigned int *capture_size);
int
dc1394_set_capture_size(raw1394handle_t handle, nodeid_t node,
                        unsigned int capture_size);
int
dc1394_get_capture_quality(raw1394handle_t handle, nodeid_t node,
                           unsigned int *capture_quality);
int
dc1394_set_capture_quality(raw1394handle_t handle, nodeid_t node,
                           unsigned int capture_quality);


/* Convenience functions to query/set based on a variable camera feature */
/* (can't be used for white balance) */
int
dc1394_get_feature_value(raw1394handle_t handle, nodeid_t node,
                         unsigned int feature, unsigned int *value);

int
dc1394_set_feature_value(raw1394handle_t handle, nodeid_t node,
                         unsigned int feature, unsigned int value);


/* Query/set specific feature characteristics */
int
dc1394_is_feature_present(raw1394handle_t handle, nodeid_t node,
                          unsigned int feature, dc1394bool_t *value);
int
dc1394_has_one_push_auto(raw1394handle_t handle, nodeid_t node,
                         unsigned int feature, dc1394bool_t *value);
int
dc1394_is_one_push_in_operation(raw1394handle_t handle, nodeid_t node,
                                unsigned int feature, dc1394bool_t *value);
int
dc1394_start_one_push_operation(raw1394handle_t handle, nodeid_t node,
                                unsigned int feature);
int
dc1394_can_read_out(raw1394handle_t handle, nodeid_t node,
                    unsigned int feature, dc1394bool_t *value);
int
dc1394_can_turn_on_off(raw1394handle_t handle, nodeid_t node,
                       unsigned int feature, dc1394bool_t *value);
int
dc1394_is_feature_on(raw1394handle_t handle, nodeid_t node,
                     unsigned int feature, dc1394bool_t *value);
int
dc1394_feature_on_off(raw1394handle_t handle, nodeid_t node,
                      unsigned int feature, unsigned int value);
                                          /* 0=off, nonzero=on */
int
dc1394_has_auto_mode(raw1394handle_t handle, nodeid_t node,
                     unsigned int feature, dc1394bool_t *value);
int
dc1394_has_manual_mode(raw1394handle_t handle, nodeid_t node,
                       unsigned int feature, dc1394bool_t *value);
int
dc1394_is_feature_auto(raw1394handle_t handle, nodeid_t node,
                       unsigned int feature, dc1394bool_t *value);
int
dc1394_auto_on_off(raw1394handle_t handle, nodeid_t node,
                   unsigned int feature, unsigned int value);
                                          /* 0=off, nonzero=on */
int
dc1394_get_min_value(raw1394handle_t handle, nodeid_t node,
                     unsigned int feature, unsigned int *value);
int
dc1394_get_max_value(raw1394handle_t handle, nodeid_t node,
                     unsigned int feature, unsigned int *value);


/**********************************
 DMA Capture Functions 

 These routines will be much faster
 than the above capture routines.
***********************************/

/*****************************************************
 dc1394_dma_setup_capture

 This sets up the given camera to capture images using 
 the dma engine.  Should be much faster than the above
 routines
******************************************************/
int
dc1394_dma_setup_capture(raw1394handle_t handle, nodeid_t node,
                         unsigned int channel, unsigned int format, unsigned int mode,
                         unsigned int speed, unsigned int frame_rate, 
                         int num_dma_buffers,
                         int drop_frames,
                         const char *dma_device_file,
                         dc1394_cameracapture *camera);

/*****************************************************
 dc1394_dma_release_camera

 This releases memory that was mapped by
 dc1394_dma_setup_camera
*****************************************************/
int 
dc1394_dma_release_camera(raw1394handle_t handle, 
                          dc1394_cameracapture *camera);

/*****************************************************
 dc1394_dma_unlisten

 This tells video1394 to halt iso reception.
*****************************************************/
int 
dc1394_dma_unlisten(raw1394handle_t handle,
                          dc1394_cameracapture *camera);

/****************************************************
 dc1394_dma_single_capture

 This captures a frame from the given camera. Two
 policies are available: wait for a frame or return
 if no frame is available (POLL)
*****************************************************/
int 
dc1394_dma_single_capture(dc1394_cameracapture *camera);

int
dc1394_dma_single_capture_poll(dc1394_cameracapture *camera);

/****************************************************
 dc1394_dma_multi_capture

 This captures a frame from the given camera. Two
 policies are available: wait for a frame or return
 if no frame is available (POLL)
*****************************************************/
int 
dc1394_dma_multi_capture(dc1394_cameracapture *camera, int num);

int
dc1394_dma_multi_capture_poll(dc1394_cameracapture *camera, int num);
/*****************************************************
 dc1394_dma_done_with_buffer

 This allows the driver to use the buffer previously handed
 to the user by dc1394_dma_*_capture
*****************************************************/
int 
dc1394_dma_done_with_buffer(dc1394_cameracapture * camera);


/********************************
 Non DMA Capture Functions 

 These functions use libraw
 to grab frames from the cameras,
 the dma routines are faster, and 
 should be used instead.
*********************************/

/***********************************************************
 dc1394_setup_capture

 Sets up both the camera and the cameracapture structure
 to be used other places.

 Returns DC1394_SUCCESS on success, DC1394_FAILURE otherwise

 NOTE: it is important to call dc1394_release_camera 
       to free memory allocated by this routine- if you
       don't, your application WILL leak memory
************************************************************/
int 
dc1394_setup_capture(raw1394handle_t handle, nodeid_t node, 
                     unsigned int channel, unsigned int format, unsigned int mode, 
                     unsigned int speed, unsigned int frame_rate, 
                     dc1394_cameracapture * camera);

/***********************************************************
 dc1394_release_camera

 Frees buffer space contained in the cameracapture structure
************************************************************/
int 
dc1394_release_camera(raw1394handle_t handle,
                      dc1394_cameracapture *camera);

/*****************************************************
 dc1394_single_capture

 Captures a frame of video from the camera specified
*****************************************************/
int 
dc1394_single_capture(raw1394handle_t handle,
                      dc1394_cameracapture *camera);

/********************************************************
 dc1394_multi_capture

 This routine captures a frame from each camera specified
 in the cams array.  Cameras must be set up first using
 dc1394_setup_camera

 Returns DC1394_FAILURE if it fails, DC1394_SUCCESS if it
 succeeds
*********************************************************/
int 
dc1394_multi_capture(raw1394handle_t handle, dc1394_cameracapture *cams,
                     int num);


/**************************************************
 Functions to read and write camera setups on/in
 memory channels
***************************************************/

int 
dc1394_get_memory_load_ch(raw1394handle_t handle, nodeid_t node,
                          unsigned int *channel);

int 
dc1394_get_memory_save_ch(raw1394handle_t handle, nodeid_t node,
                          unsigned int *channel);

int 
dc1394_is_memory_save_in_operation(raw1394handle_t handle, nodeid_t node,
                                   dc1394bool_t *value);

int 
dc1394_set_memory_save_ch(raw1394handle_t handle, nodeid_t node,
                          unsigned int channel);

int
dc1394_memory_save(raw1394handle_t handle, nodeid_t node);

int
dc1394_memory_load(raw1394handle_t handle, nodeid_t node,
                   unsigned int channel);


/*************************************************
 Functions to control the trigger feature.
**************************************************/

int
dc1394_set_trigger_polarity(raw1394handle_t handle, nodeid_t node,
                            dc1394bool_t polarity);

int
dc1394_get_trigger_polarity(raw1394handle_t handle, nodeid_t node,
                            dc1394bool_t *polarity);

int
dc1394_trigger_has_polarity(raw1394handle_t handle, nodeid_t node,
                            dc1394bool_t *polarity);

int
dc1394_set_trigger_on_off(raw1394handle_t handle, nodeid_t node,
                          dc1394bool_t on_off);

int
dc1394_get_trigger_on_off(raw1394handle_t handle, nodeid_t node,
                          dc1394bool_t *on_off);

/* Turn one software trigger on or off and get state */
int
dc1394_set_soft_trigger(raw1394handle_t handle, nodeid_t node);

int
dc1394_unset_soft_trigger(raw1394handle_t handle, nodeid_t node);

int
dc1394_get_soft_trigger(raw1394handle_t handle, nodeid_t node,
			dc1394bool_t *is_on);

 
/*************************************************
  FORMAT_7 access functions
**************************************************/

/*======================================================================*/
/*! 
 *   setup capture for format7 (FORMAT_SCALABLE_IMAGE_SIZE) mode. For
 *   some parameters you may pass QUERY_FROM_CAMERA (which means query
 *   this value from the camera and maybe adjust it to the new
 *   conditions) or USE_MAX_AVAIL (which means query the maximum
 *   availible value for this parameter from camera and use this)
 *
 *   \param handle   handle for raw1394 port
 *   \param node     node of the camera
 *   \param channel  iso channel for data transmission (0 ... 15)
 *   \param mode     mode for camera operation 
 *                   (MODE_FORMAT7_0 ... MODE_FORMAT7_7)
 *   \param speed    transmission speed (SPEED_100 ... SPEED_400)
 *   \param bytes_per_packet number of bytes per packet can be used to
 *                   control the framerate or QUERY_FROM_CAMERA or
 *                   USE_MAX_AVAIL
 *   \param left     area of interest start column or QUERY_FROM_CAMERA
 *   \param top      area of interest start row or QUERY_FROM_CAMERA
 *   \param width    area of interest width or QUERY_FROM_CAMERA
 *                   or USE_MAX_AVAIL
 *   \param height   area of interest height or QUERY_FROM_CAMERA
 *                   or USE_MAX_AVAIL
 *   \param camera   (out) structure containing the returned parameters
 *                   from the camera. Memory for this struct must be
 *                   allocated by the calling  function
 *
 *   \return DC1394_SUCCESS or DC1394_FAILURE
 */
/*======================================================================*/
int
dc1394_setup_format7_capture(raw1394handle_t handle, nodeid_t node,
                             unsigned int channel, unsigned int mode, unsigned int speed,
                             unsigned int bytes_per_packet,
                             unsigned int left, unsigned int top,
                             unsigned int width, unsigned int height, 
                             dc1394_cameracapture * camera);
  

/*======================================================================*/
/*!
 *   setup capture for format7 (FORMAT_SCALABLE_IMAGE_SIZE) mode using
 *   the dma engine.  Should be much faster than the above
 *   routines. For some parameters you may pass QUERY_FROM_CAMERA
 *   (which means query this value from the camera and maybe adjust it
 *   to the new conditions) or USE_MAX_AVAIL (which means query the
 *   maximum availible value for this parameter from camera and use
 *   this)
 *
 *   \param handle   handle for raw1394 port
 *   \param node     node of the camera
 *   \param channel  iso channel for data transmission (0 ... 15)
 *   \param mode     mode for camera operation 
 *                   (MODE_FORMAT7_0 ... MODE_FORMAT7_7)
 *   \param speed    transmission speed (SPEED_100 ... SPEED_400)
 *   \param bytes_per_packet number of bytes per packet can be used to
 *                   control the framerate or QUERY_FROM_CAMERA or
 *                   USE_MAX_AVAIL
 *   \param left     area of interest start column or QUERY_FROM_CAMERA
 *   \param top      area of interest start row or QUERY_FROM_CAMERA
 *   \param width    area of interest width or QUERY_FROM_CAMERA
 *                   or USE_MAX_AVAIL
 *   \param height   area of interest height or QUERY_FROM_CAMERA
 *                   or USE_MAX_AVAIL
 *   \param num_dma_buffers number of buffers for the dma ring buffer
 *   \param camera   (out) structure containing the returned parameters
 *                   from the camera. Memory for this struct must be
 *                   allocated by the calling  function
 *
 *   \return DC1394_SUCCESS or DC1394_FAILURE
 */
/*======================================================================*/
int
dc1394_dma_setup_format7_capture(raw1394handle_t handle, nodeid_t node,
                                 unsigned int channel, unsigned int mode, unsigned int speed,
                                 unsigned int bytes_per_packet,
                                 unsigned int left, unsigned int top,
                                 unsigned int width, unsigned int height,
                                 int num_dma_buffers,
				 int drop_frames,
				 const char *dma_device_file,
                                 dc1394_cameracapture *camera);
  

int
dc1394_query_format7_max_image_size(raw1394handle_t handle, nodeid_t node,
				    unsigned int mode,
                                    unsigned int *horizontal_size,
				    unsigned int *vertical_size);

int
dc1394_query_format7_unit_size(raw1394handle_t handle, nodeid_t node,
			       unsigned int mode,
                               unsigned int *horizontal_unit,
			       unsigned int *vertical_unit);

int
dc1394_query_format7_image_position(raw1394handle_t handle, nodeid_t node,
				    unsigned int mode,
                                    unsigned int *left_position,
				    unsigned int *top_position);

int
dc1394_query_format7_image_size(raw1394handle_t handle, nodeid_t node,
				unsigned int mode, unsigned int *width,
				unsigned int *height);

int
dc1394_query_format7_color_coding_id(raw1394handle_t handle, nodeid_t node,
				     unsigned int mode, unsigned int *color_id);

int
dc1394_query_format7_color_coding(raw1394handle_t handle, nodeid_t node,
				  unsigned int mode, quadlet_t *value);

int
dc1394_query_format7_pixel_number(raw1394handle_t handle, nodeid_t node,
				  unsigned int mode, unsigned int *pixnum);

int
dc1394_query_format7_total_bytes(raw1394handle_t handle, nodeid_t node,
				 unsigned int mode, unsigned long long int *total_bytes);

int
dc1394_query_format7_packet_para(raw1394handle_t handle, nodeid_t node,
				 unsigned int mode, unsigned int *min_bytes,
				 unsigned int *max_bytes);

int
dc1394_query_format7_byte_per_packet(raw1394handle_t handle, nodeid_t node,
				     unsigned int mode,
                                     unsigned int *packet_bytes);

int
dc1394_set_format7_image_position(raw1394handle_t handle, nodeid_t node,
				  unsigned int mode, unsigned int left,
				  unsigned int top);

int
dc1394_set_format7_image_size(raw1394handle_t handle, nodeid_t node,
			      unsigned int mode, unsigned int width,
			      unsigned int height);

int
dc1394_set_format7_color_coding_id(raw1394handle_t handle, nodeid_t node,
				   unsigned int mode, unsigned int color_id);

int
dc1394_set_format7_byte_per_packet(raw1394handle_t handle, nodeid_t node,
				   unsigned int mode,
                                   unsigned int packet_bytes);

int
dc1394_query_format7_value_setting(raw1394handle_t handle, nodeid_t node,
				   unsigned int mode,
				   unsigned int *present,
				   unsigned int *setting1,
				   unsigned int *err_flag1,
				   unsigned int *err_flag2);
int
dc1394_set_format7_value_setting(raw1394handle_t handle, nodeid_t node,
				 unsigned int mode);

int
dc1394_query_format7_recommended_byte_per_packet(raw1394handle_t handle, nodeid_t node,
						  unsigned int mode,
						  unsigned int *bpp);

int
dc1394_query_format7_packet_per_frame(raw1394handle_t handle, nodeid_t node,
				      unsigned int mode,
				      unsigned int *ppf);

int
dc1394_query_format7_unit_position(raw1394handle_t handle, nodeid_t node,
				   unsigned int mode,
				   unsigned int *horizontal_pos,
				   unsigned int *vertical_pos);
int
dc1394_query_format7_data_depth(raw1394handle_t handle, nodeid_t node,
				unsigned int mode,
				unsigned int *data_depth);

int
dc1394_query_format7_frame_interval(raw1394handle_t handle, nodeid_t node,
				    unsigned int mode,
				    float *interval);

int
dc1394_query_format7_color_filter_id(raw1394handle_t handle, nodeid_t node,
				     unsigned int mode,
				     unsigned int *filter_id);

/**********************************
 *   ABSOLUTE SETTING FUNCTIONS   *		     
 **********************************/


int
dc1394_query_absolute_feature_min_max(raw1394handle_t handle, nodeid_t node,
				      unsigned int feature,
				      float *min, float *max);

int
dc1394_query_absolute_feature_value(raw1394handle_t handle, nodeid_t node,
				    int feature, float *value);

int
dc1394_set_absolute_feature_value(raw1394handle_t handle, nodeid_t node,
				  int feature, float value);

int
dc1394_query_absolute_control(raw1394handle_t handle, nodeid_t node,
			    unsigned int feature, dc1394bool_t *value);

int
dc1394_absolute_setting_on_off(raw1394handle_t handle, nodeid_t node,
			       unsigned int feature, unsigned int value);

int
dc1394_has_absolute_control(raw1394handle_t handle, nodeid_t node,
			    unsigned int feature, dc1394bool_t *value);


int
dc1394_get_bandwidth_usage(raw1394handle_t handle, nodeid_t node, unsigned int *bandwidth);

int
dc1394_get_camera_port(raw1394handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _DC1394_CAMERA_CONTROL_H */
