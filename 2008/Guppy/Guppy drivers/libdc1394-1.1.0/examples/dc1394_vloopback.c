/**************************************************************************
**       Title: Turn a Digital Camera into a V4L device using vloopback
**    $RCSfile: dc1394_vloopback.c,v $
**   $Revision: 1.11 $$Name:  $
**       $Date: 2004/08/10 07:57:22 $
**   Copyright: LGPL $Author: ddouxchamps $
** Description:
**
**    Sends format0 640x480 RGB to the vloopback input device so that it
**    can be consumed by V4L applications on the vloopback output device.
**    Get vloopback 0.90 from http://motion.technolust.cx/vloopback/
**    Apply the patch from effectv http://effectv.sf.net/
**    It has been tested with EffecTV (exciting!), GnomeMeeting, ffmpeg,
**    camsource, Kino, and veejay.
**
** TODO:
**    - Support video controls
**    - do cropping on v4l window offset
**    - supply audio from sound card?
**
**-------------------------------------------------------------------------
**
**  $Log: dc1394_vloopback.c,v $
**  Revision 1.11  2004/08/10 07:57:22  ddouxchamps
**  Removed extra buffering (Johann Schoonees)
**
**  Revision 1.10  2004/03/09 08:41:44  ddouxchamps
**  patch from Johann Schoonees for extra buffering
**
**  Revision 1.9  2004/01/20 04:15:34  ddennedy
**  workaround some potential V4L compilation error
**
**  Revision 1.8  2004/01/20 04:12:27  ddennedy
**  added dc1394_free_camera_nodes and applied to examples
**
**  Revision 1.7  2004/01/14 22:20:43  ddennedy
**  enhanced dc1394_vloopback
**
**  Revision 1.6  2004/01/04 21:27:56  ddennedy
**  simplify video1394 includes and other minor corrections
**
**  Revision 1.5  2003/09/23 13:44:12  ddennedy
**  fix camera location by guid for all ports, add camera guid option to vloopback, add root detection and reset to vloopback
**
**  Revision 1.4  2003/09/15 17:21:27  ddennedy
**  add features to examples
**
**  Revision 1.1  2002/10/27 04:21:54  ddennedy
**  added v4l utility using vloopback
**
**
**************************************************************************/

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <unistd.h>
#include <signal.h>
#define _LINUX_TIME_H
#define _DEVICE_H_
#include <linux/videodev.h>
#include <getopt.h>

#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>
#include "affine.h"

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

/* uncomment the following to drop frames to prevent delays */
#define DROP_FRAMES 1
#define MAX_PORTS 4
#define DC1394_BUFFERS 8
#define DC1394_WIDTH 640
#define DC1394_HEIGHT 480
#define MAX_WIDTH 1024
#define MAX_HEIGHT 768
#define MIN_WIDTH 160
#define MIN_HEIGHT 120
#define MAX_BPP 3
#define V4L_BUFFERS 2
#define MAX_RESETS 5

/* declarations for libdc1394 */
raw1394handle_t handle = NULL;
dc1394_cameracapture camera;
nodeid_t *camera_nodes;
dc1394_feature_set features;

/* Video4Linux globals */
int v4l_dev = -1;
unsigned char *out_pipe = NULL, *out_mmap = NULL;
#define MAXIOCTL 1024
char ioctlbuf[MAXIOCTL];

/* cmdline options */
int g_daemon = 0;
enum v4l_modes {
	V4L_MODE_NONE,
	V4L_MODE_PIPE,
	V4L_MODE_MMAP
};
enum v4l_modes g_v4l_mode = V4L_MODE_MMAP;
char *dc_dev_name = NULL;
char *v4l_dev_name = NULL;
int g_v4l_fmt = VIDEO_PALETTE_YUV422;
int g_width = DC1394_WIDTH;
int g_height = DC1394_HEIGHT;
u_int64_t g_guid = 0;

static struct option long_options[]={
	{"daemon",0,NULL,0},
	{"pipe",0,NULL,0},
	{"guid",1,NULL,0},
	{"video1394",1,NULL,0},
	{"vloopback",1,NULL,0},
	{"palette",1,NULL,0},
	{"width",1,NULL,0},
	{"height",1,NULL,0},
	{"help",0,NULL,0},
	{NULL,0,0,0}
	};

void get_options(int argc,char *argv[])
{
	int option_index=0;
	
	while(getopt_long(argc,argv,"",long_options,&option_index)>=0){
			switch(option_index){ 
				/* case values must match long_options */
				case 0:
					g_daemon = 1;
					break;
				case 1:
					g_v4l_mode = V4L_MODE_PIPE;
					break;
				case 2:
					sscanf(optarg, "%llx", &g_guid);
					break;
				case 3:
					dc_dev_name=strdup(optarg);
					break;
				case 4:
					v4l_dev_name=strdup(optarg);
					break;
				case 5:
					if (strcasecmp(optarg, "rgb24") == 0)
						g_v4l_fmt = VIDEO_PALETTE_RGB24;
					else if (strcasecmp(optarg, "yuv422p") == 0)
						g_v4l_fmt = VIDEO_PALETTE_YUV422P;
					else if (strcasecmp(optarg, "yuv420p") == 0)
						g_v4l_fmt = VIDEO_PALETTE_YUV420P;
					break;
				case 6:
					g_width = atoi(optarg);
					break;
				case 7:
					g_height = atoi(optarg);
					break;
				default:
					printf( "\n"
						"%s - send video from dc1394 through vloopback\n\n"
						"Usage:\n"
						"    %s [--daemon] [--pipe] [--guid=camera-euid]\n"
						"       [--video1394=/dev/video1394/x] [--vloopback=/dev/video0]\n"
						"       [--palette=format] [--width=n] [--height=n]\n\n"
						"    --daemon    - run as a daemon, detached from console (optional)\n"
						"    --pipe      - write images to vloopback device instead of using\n"
						"                  zero-copy mmap mode (optional)\n"
						"    --guid      - select camera to use (optional)\n"
						"                  default is first camera on any port\n"
						"    --video1394 - specifies video1394 device to use (optional)\n"
						"                  default = /dev/video1394/<port#>\n"
						"    --vloopback - specifies video4linux device to use (optional)\n"
						"                  by default, this is automatically determined!\n"
						"    --palette   - specified the video palette to use (optional)\n"
						"                  yuv422 (default), yuv422p, yuv420p, or rgb24\n"
						"    --width     - set the initial width (default=640)\n"
						"    --height    - set the initial height (default=480)\n"
						"    --help      - prints this message\n\n"
						,argv[0],argv[0]);
					exit(0);
			}
	}
	
}

/***** IMAGE PROCESSING *******************************************************/

typedef void (*affine_transform_cb)(const unsigned char *src, unsigned char *dest);

void transform_yuv422(const unsigned char *src, unsigned char *dest)
{
	swab(src, dest, 4);
}

void transform_rgb24(const unsigned char *src, unsigned char *dest)
{
	dest[2] = src[0]; //b
	dest[1] = src[1]; //g
	dest[0] = src[2]; //r
}

void affine_scale( const unsigned char *src, int src_width, int src_height, unsigned char *dest, int dest_width, int dest_height,
                   int bpp, affine_transform_cb transform )
{	
	affine_transform_t affine;
	double scale_x = (double) dest_width / (double) src_width;
	double scale_y = (double) dest_height / (double) src_height;
	register unsigned char *d = dest;
    register const unsigned char  *s = src;
	register int i, j, x, y;

	/* optimization for no scaling required */
	if (scale_x == 1.0 && scale_y == 1.0) {
		for (i = 0; i < src_width*src_height; i++) {
			transform(s, d);
			s += bpp;
			d += bpp;
		}
		return;
	}
	
	affine_transform_init( &affine );
	
	if ( scale_x <= 1.0 && scale_y <= 1.0 )
	{
		affine_transform_scale( &affine, scale_x, scale_y );

		for( j = 0; j < src_height; j++ )
			for( i = 0; i < src_width; i++ )
			{
				x = (int) ( affine_transform_mapx( &affine, i - src_width/2, j - src_height/2 ) );
				y = (int) ( affine_transform_mapy( &affine, i - src_width/2, j - src_height/2 ) );
				x += dest_width/2;
				x = CLAMP( x, 0, dest_width);
				y += dest_height/2;
				y = CLAMP( y, 0, dest_height);
				s = src + (j*src_width*bpp) + i*bpp;
				d = dest + y*dest_width*bpp + x*bpp;
				transform(s, d);
				d += bpp;
				s += bpp;
			}
	}
	else if ( scale_x >= 1.0 && scale_y >= 1.0 )
	{
		affine_transform_scale( &affine, 1.0/scale_x, 1.0/scale_y );
	
		for( y = 0; y < dest_height; y++ )
			for( x = 0; x < dest_width; x++ )
			{
				i = (int) ( affine_transform_mapx( &affine, x - dest_width/2, y - dest_height/2 ) );
				j = (int) ( affine_transform_mapy( &affine, x - dest_width/2, y - dest_height/2 ) );
				i += src_width/2;
				i = CLAMP( i, 0, dest_width);
				j += src_height/2;
				j = CLAMP( j, 0, dest_height);
				s = src + (j*src_width*bpp) + i*bpp;
				d = dest + y*dest_width*bpp + x*bpp;
				transform(s, d);
				d += bpp;
				s += bpp;
			}
	}
}

inline void
yuy2_to_yv16( const unsigned char *src, unsigned char *dest, int width, int height)
{
	register int total = (width * height) >> 1;
	register int i;
	register unsigned char *img_y = dest;
	register unsigned char *img_cb = img_y + (width * height);
	register unsigned char *img_cr = img_cb + total;

	for (i = 0; i < total; i++) {
		*img_y++  = *src++;
		*img_cb++ = *src++;
		*img_y++  = *src++;
		*img_cr++ = *src++;
	}
}

inline void
yuy2_to_yv12( const unsigned char *src, unsigned char *dest, int width, int height)
{
	register int stride = width >> 1;
	register int i, j;
	register unsigned char *img_y = dest;
	register unsigned char *img_cb = img_y + (width * height);
	register unsigned char *img_cr = img_cb + stride * (height >> 1);

	for (i = 0; i < height; i++) {
		for (j = 0; j < stride; j++) {
			if (i%2) {
				*img_y++  = *src++;
				src++;
				*img_y++  = *src++;
				src++;
			} else {
				*img_y++  = *src++;
				*img_cb++ = (src[width << 1] + src[0]) >> 1;
				src++;
				*img_y++  = *src++;
				*img_cr++ = (src[width << 1] + src[0]) >> 1;
				src++;
			}
		}
	}
}

/***** IMAGE CAPTURE **********************************************************/

int capture_pipe(int dev, const unsigned char *image_in)
{
	int size = g_width * g_height;
	int bpp = 0;
	int ppp = 1; /* pixels per pixel! */
	affine_transform_cb transform = NULL;

	switch (g_v4l_fmt) {
		case VIDEO_PALETTE_RGB24:
			bpp = 3;
			ppp = 1;
			transform = transform_rgb24;
			break;

		case VIDEO_PALETTE_YUV422:
		case VIDEO_PALETTE_YUV422P:
		case VIDEO_PALETTE_YUV420P:
			bpp = 2;
			ppp = 2;
			transform = transform_yuv422;
			break;

		default:
			return 0;
	}
	
	affine_scale( image_in, DC1394_WIDTH/ppp, DC1394_HEIGHT,
		out_pipe, g_width/ppp, g_height,
		ppp * bpp, transform);
		
	if (g_v4l_fmt == VIDEO_PALETTE_YUV422P && out_pipe != NULL) {
		size_t memsize = (g_width * g_height) << 1;
		unsigned char *buffer = malloc(memsize);
		if (buffer) {
			memcpy( buffer, out_pipe, memsize);
			yuy2_to_yv16( buffer, out_pipe, g_width, g_height);
			free(buffer);
		}
	}
	else if (g_v4l_fmt == VIDEO_PALETTE_YUV420P && out_pipe != NULL) {
		size_t memsize = (g_width * g_height) << 1;
		unsigned char *buffer = malloc(memsize);
		if (buffer) {
			memcpy( buffer, out_pipe, memsize);
			yuy2_to_yv12( buffer, out_pipe, g_width, g_height);
			free(buffer);
		}
		size = g_width * g_height * 3 / 2;
		bpp = 1;
	}
	
	if (write(dev, out_pipe, size*bpp) != (size*bpp)) {
		perror("Error writing image to pipe");
		return 0;
	}
	return 1;
}

int capture_mmap(int frame)
{
	int bpp = 0;
	int ppp = 1; /* pixels per pixel! */
	affine_transform_cb transform = NULL;
	
	switch (g_v4l_fmt) {
		case VIDEO_PALETTE_RGB24:
			bpp = 3;
			ppp = 1;
			transform = transform_rgb24;
			break;

		case VIDEO_PALETTE_YUV422:
		case VIDEO_PALETTE_YUV422P:
		case VIDEO_PALETTE_YUV420P:
			bpp = 2;
			ppp = 2;
			transform = transform_yuv422;
			break;

		default:
			return 0;
	}
	
	if (g_v4l_fmt == VIDEO_PALETTE_YUV422P && out_pipe != NULL) {
		if (dc1394_dma_single_capture(&camera) == DC1394_SUCCESS) {
			affine_scale( (unsigned char *) camera.capture_buffer, DC1394_WIDTH/ppp, DC1394_HEIGHT,
				out_pipe, g_width/ppp, g_height,
				ppp * bpp, transform);
			dc1394_dma_done_with_buffer(&camera);
		}
		yuy2_to_yv16( out_pipe, out_mmap + (MAX_WIDTH * MAX_HEIGHT * 3 * frame), g_width, g_height);
	}
	else if (g_v4l_fmt == VIDEO_PALETTE_YUV420P && out_pipe != NULL) {
		if (dc1394_dma_single_capture(&camera) == DC1394_SUCCESS) {
			affine_scale( (unsigned char *) camera.capture_buffer, DC1394_WIDTH/ppp, DC1394_HEIGHT,
				out_pipe, g_width/ppp, g_height,
				ppp * bpp, transform);
			dc1394_dma_done_with_buffer(&camera);
		}
		yuy2_to_yv12( out_pipe, out_mmap + (MAX_WIDTH * MAX_HEIGHT * 3 * frame), g_width, g_height);
	}
	else if (dc1394_dma_single_capture(&camera) == DC1394_SUCCESS) {
		affine_scale( (unsigned char *) camera.capture_buffer, DC1394_WIDTH/ppp, DC1394_HEIGHT,
			out_mmap + (MAX_WIDTH * MAX_HEIGHT * 3 * frame), g_width/ppp, g_height,
			ppp * bpp, transform);
		dc1394_dma_done_with_buffer(&camera);
	}

	
	return 1;
}

/***** DIGITAL CAMERA *********************************************************/

int dc_init()
{
	struct raw1394_portinfo ports[MAX_PORTS];
	int numPorts = MAX_PORTS;
	int port, reset;
	int camCount = 0;
	int found = 0;

	/* get the number of ports (cards) */
	handle = raw1394_new_handle();
    if (handle==NULL) {
        perror("Unable to aquire a raw1394 handle\n");
        perror("did you load the drivers?\n");
        exit(-1);
    }
	
	numPorts = raw1394_get_port_info(handle, ports, numPorts);
	raw1394_destroy_handle(handle);
	handle = NULL;

	for (reset = 0; reset < MAX_RESETS && !found; reset++) {
		/* get dc1394 handle to each port */
		for (port = 0; port < numPorts && !found; port++) {
			if (handle != NULL)
				dc1394_destroy_handle(handle);
			handle = dc1394_create_handle(port);
			if (handle==NULL) {
				perror("Unable to aquire a raw1394 handle\n");
				perror("did you load the drivers?\n");
				exit(-1);
			}
	
			/* get the camera nodes and describe them as we find them */
			camCount = 0;
			camera_nodes = dc1394_get_camera_nodes(handle, &camCount, 0);
			
			if (camCount > 0) {
				if (g_guid == 0) {
					dc1394_camerainfo info;
					/* use the first camera found */
					camera.node = camera_nodes[0];
					if (dc1394_get_camera_info(handle, camera_nodes[0], &info) == DC1394_SUCCESS)
						dc1394_print_camera_info(&info);
					found = 1;
				}
				else {
					/* attempt to locate camera by guid */
					int cam;
					for (cam = 0; cam < camCount && found == 0; cam++) {
						dc1394_camerainfo info;
						if (dc1394_get_camera_info(handle, camera_nodes[cam], &info) == DC1394_SUCCESS)	{
							if (info.euid_64 == g_guid)	{
								dc1394_print_camera_info(&info);
								camera.node = camera_nodes[cam];
								found = 1;
							}
						}
					}
				}
				if (found == 1)	{
					/* camera can not be root--highest order node */
					if (camera.node == raw1394_get_nodecount(handle)-1)	{
						/* reset and retry if root */
						raw1394_reset_bus(handle);
						sleep(2);
						found = 0;
					}
				}
				dc1394_free_camera_nodes(camera_nodes);
			} /* camCount > 0 */
		} /* next port */
	} /* next reset retry */
	if (found) {
		/*have the camera start sending us data*/
		if (dc1394_start_iso_transmission(handle, camera.node) !=DC1394_SUCCESS) {
			perror("unable to start camera iso transmission\n");
			exit(-1);
		}
	}
	if (found == 0 && g_guid != 0) {
		fprintf( stderr, "Unable to locate camera node by guid\n");
		exit(-1);
	}
	else if (camCount == 0) {
		fprintf( stderr, "no cameras found :(\n");
		exit(-1);
	}
	if (reset == MAX_RESETS) {
		fprintf( stderr, "failed to not make camera root node :(\n");
		exit(-1);
	}
	return found;
}

int dc_start(int palette)
{
	unsigned int channel;
	unsigned int speed;
	int mode;

	switch (palette) {
		case VIDEO_PALETTE_RGB24:
			mode = MODE_640x480_RGB;
			break;
			
		case VIDEO_PALETTE_YUV422:
		case VIDEO_PALETTE_YUV422P:
		case VIDEO_PALETTE_YUV420P:
			mode = MODE_640x480_YUV422;
			break;

		default:
			return 0;
	}
	
	if (dc1394_get_iso_channel_and_speed(handle, camera.node,
								 &channel, &speed) !=DC1394_SUCCESS) 
	{
		printf("unable to get the iso channel number\n");
		return 0;
	}
	 
	if (dc1394_dma_setup_capture(handle, camera.node,  channel,
							FORMAT_VGA_NONCOMPRESSED, mode,
							speed, FRAMERATE_15, DC1394_BUFFERS, DROP_FRAMES,
							dc_dev_name, &camera) != DC1394_SUCCESS) 
	{
		fprintf(stderr, "unable to setup camera- check line %d of %s to make sure\n",
			   __LINE__,__FILE__);
		perror("that the video mode,framerate and format are supported\n");
		fprintf(stderr, "is one supported by your camera\n");
		return 0;
	}
	return 1;
}

void dc_stop()
{
	if (handle && g_v4l_mode != V4L_MODE_NONE) {
		dc1394_dma_unlisten( handle, &camera );
		dc1394_dma_release_camera( handle, &camera);
	}
}
		
/***** VIDEO4LINUX ***********************************************************/

int v4l_open(char *device)
{
	int pipe = -1;
	FILE *vloopbacks;
	char pipepath[255];
	char buffer[255];
	char *loop;
	char *input;
	char *istatus;
	char *output;
	char *ostatus;

	if (device == NULL) {
		vloopbacks=fopen("/proc/video/vloopback/vloopbacks", "r");
		if (!vloopbacks) {
			perror ("Failed to open '/proc/video/vloopback/vloopbacks");
			return -1;
		}
		/* Read vloopback version */
		fgets(buffer, 255, vloopbacks);
		printf("%s", buffer);
		/* Read explanation line */
		fgets(buffer, 255, vloopbacks);
		while (fgets(buffer, 255, vloopbacks)) {
			if (strlen(buffer)>1) {
				buffer[strlen(buffer)-1]=0;
				loop=strtok(buffer, "\t");
				input=strtok(NULL, "\t");
				istatus=strtok(NULL, "\t");
				output=strtok(NULL, "\t");
				ostatus=strtok(NULL, "\t");
				if (istatus[0]=='-') {
					sprintf(pipepath, "/dev/%s", input);
					pipe=open(pipepath, O_RDWR);
					if (pipe>=0) {
						printf("Input: /dev/%s\n", input);
						printf("Output: /dev/%s\n", output);
						break;
					}
				}
			} 
		}
	} else {
		pipe=open(device, O_RDWR);
	}
	return pipe;
}

int v4l_start_pipe (int width, int height, int format)
{
	struct video_capability vid_caps;
	struct video_window vid_win;
	struct video_picture vid_pic;
	size_t memsize = width * height;

	switch (format) {
		case VIDEO_PALETTE_RGB24:
			memsize *= 3;
			break;

		case VIDEO_PALETTE_YUV422:
		case VIDEO_PALETTE_YUV422P:
		case VIDEO_PALETTE_YUV420P:
			memsize *= 2;
			break;

		default:
			return 0;
	}

	if (out_pipe)
		free(out_pipe);
	out_pipe = (unsigned char *) malloc(memsize);
	
	if (ioctl (v4l_dev, VIDIOCGCAP, &vid_caps) == -1) {
		perror ("ioctl (VIDIOCGCAP)");
		return 0;
	}
	if (ioctl (v4l_dev, VIDIOCGPICT, &vid_pic)== -1) {
		perror ("ioctl VIDIOCGPICT");
		return 0;
	}
	vid_pic.palette = format;
	if (ioctl (v4l_dev, VIDIOCSPICT, &vid_pic)== -1) {
		perror ("ioctl VIDIOCSPICT");
		return 0;
	}
	if (ioctl (v4l_dev, VIDIOCGWIN, &vid_win)== -1) {
		perror ("ioctl VIDIOCGWIN");
		return 0;
	}
	vid_win.width=width;
	vid_win.height=height;
	if (ioctl (v4l_dev, VIDIOCSWIN, &vid_win)== -1) {
		perror ("ioctl VIDIOCSWIN");
		return 0;
	}
	//g_v4l_mode = V4L_MODE_PIPE;
	return 1;
}

int v4l_start_mmap (int memsize)
{
	if (out_pipe)
		free(out_pipe);
	out_pipe = (unsigned char *) malloc(MAX_WIDTH * MAX_HEIGHT * 2);

	out_mmap = mmap(0, memsize, PROT_READ|PROT_WRITE, MAP_SHARED, v4l_dev, 0);
	if ( out_mmap == (unsigned char *) -1 )
		return 0;
	//g_v4l_mode = V4L_MODE_MMAP;
	return 1;	
}

int v4l_ioctl(unsigned long int cmd, void *arg)
{
	//printf("ioctl %d\n", cmd & 0xff);
	switch (cmd) {
		case VIDIOCGCAP:
		{
			struct video_capability *vidcap=arg;

			sprintf(vidcap->name, "IEEE 1394 Digital Camera");
			vidcap->type = VID_TYPE_CAPTURE | VID_TYPE_SCALES;
			vidcap->channels = 1;
			vidcap->audios = 0;
			vidcap->maxwidth = MAX_WIDTH;
			vidcap->maxheight = MAX_HEIGHT;
			vidcap->minwidth = MIN_WIDTH;
			vidcap->minheight = MIN_HEIGHT;
			break;
		}
		case VIDIOCGTUNER:
		{
			struct video_tuner *vidtune=arg;
			
			sprintf(vidtune->name, "IEEE 1394 Digital Camera");
			vidtune->tuner = 0;
			vidtune->rangelow = 0;
			vidtune->rangehigh = 0;
			vidtune->flags = VIDEO_TUNER_PAL | VIDEO_TUNER_NTSC;
			vidtune->mode = g_height > 480 ? VIDEO_MODE_PAL : VIDEO_MODE_NTSC;
			vidtune->signal = 0;
			break;
		}
		case VIDIOCGCHAN:
		{
			struct video_channel *vidchan=arg;
			
			vidchan->channel = 0;
			vidchan->flags = 0;
			vidchan->tuners = 0;
			vidchan->type = VIDEO_TYPE_CAMERA;
			strcpy(vidchan->name, "Dummy channel");
			break;
		}
		case VIDIOCGPICT:
		{
			struct video_picture *vidpic=arg;

			/* TODO: we might be able to support these */
			vidpic->colour = 0xffff;
			vidpic->hue = 0xffff;
			vidpic->brightness = 0xffff;
			vidpic->contrast = 0xffff;
			vidpic->whiteness = 0xffff;
			
			vidpic->palette = g_v4l_fmt;
			switch (g_v4l_fmt) {
				case VIDEO_PALETTE_RGB24:
					vidpic->depth = 24;
					break;
					
				case VIDEO_PALETTE_YUV422:
				case VIDEO_PALETTE_YUV422P:
					vidpic->depth = 16;
					break;

				case VIDEO_PALETTE_YUV420P:
					vidpic->depth = 12;
					break;

				default:
					return 1;
			}
			break;
		}
		case VIDIOCSPICT:
		{
			struct video_picture *vidpic=arg;
			
			switch (vidpic->palette)
			{
				case VIDEO_PALETTE_YUV422:
				case VIDEO_PALETTE_YUV422P:
				case VIDEO_PALETTE_YUV420P:
					printf("VIDIOCSPICT: video palette set to YUV42%d%s\n",
						vidpic->palette == VIDEO_PALETTE_YUV420P ? 0 : 2,
						vidpic->palette == VIDEO_PALETTE_YUV422P ? "P" : "");
					dc_stop();
					g_v4l_fmt = vidpic->palette;
					dc_start(g_v4l_fmt);
					break;
					
				case VIDEO_PALETTE_RGB24:
					printf("VIDIOCSPICT: video palette set to RGB24\n");
					dc_stop();
					g_v4l_fmt = vidpic->palette;
					dc_start(g_v4l_fmt);
					break;

				default:
					printf("VIDIOCSPICT: unsupported video palette %d\n", vidpic->palette);
					return 1;
			}
			break;
		}
		case VIDIOCCAPTURE:
		{
			break;
		}
		case VIDIOCGWIN:
		{
			struct video_window *vidwin=arg;

			vidwin->x=0;
			vidwin->y=0;
			vidwin->width=g_width;
			vidwin->height=g_height;
			vidwin->chromakey=0;
			vidwin->flags=0;
			vidwin->clipcount=0;
			break;
		}
		case VIDIOCSWIN:
		{
			struct video_window *vidwin=arg;
				
			/* TODO: support cropping */
			
			if (vidwin->width > MAX_WIDTH ||
			    vidwin->height > MAX_HEIGHT )
				return 1;
			if (vidwin->flags)
				return 1;
			g_width = vidwin->width;
			g_height = vidwin->height;
			printf("VIDIOCSWIN: size set to %dx%d\n", g_width, g_height);
			break;
		}
		case VIDIOCGMBUF:
		{
			struct video_mbuf *vidmbuf=arg;
			int i;
			
			vidmbuf->size = MAX_WIDTH * MAX_HEIGHT * MAX_BPP;
			vidmbuf->frames = V4L_BUFFERS;
			for (i=0; i < V4L_BUFFERS; i++)
				vidmbuf->offsets[i] = i * vidmbuf->size;
			vidmbuf->size *= vidmbuf->frames;
			break;
		}
		case VIDIOCMCAPTURE:
		{
			struct video_mmap *vidmmap=arg;

			if ( vidmmap->format != g_v4l_fmt )
			switch (vidmmap->format)
			{
				case VIDEO_PALETTE_YUV422:
				case VIDEO_PALETTE_YUV422P:
				case VIDEO_PALETTE_YUV420P:
					printf("VIDIOCMCAPTURE: video palette set to YUV42%d%s\n",
						vidmmap->format == VIDEO_PALETTE_YUV420P ? 0 : 2,
						vidmmap->format == VIDEO_PALETTE_YUV422P ? "P" : "");
					dc_stop();
					g_v4l_fmt = vidmmap->format;
					dc_start(g_v4l_fmt);
					break;

				case VIDEO_PALETTE_RGB24:
					printf("VIDIOCMCAPTURE: video palette set to RGB24\n");
					dc_stop();
					g_v4l_fmt = vidmmap->format;
					dc_start(g_v4l_fmt);
					break;

				default:
					printf("VIDIOCMCAPTURE: unsupported video palette %d\n", vidmmap->format);
					return 1;
			}
				
			if (vidmmap->height > MAX_HEIGHT ||
			    vidmmap->width > MAX_WIDTH) {
				printf("VIDIOCMCAPTURE: invalid size %dx%d\n", vidmmap->width, vidmmap->height );
				return 1;
			}
			if (vidmmap->height != g_height ||
			    vidmmap->width != g_width) {
				g_height = vidmmap->height;
				g_width = vidmmap->width;
				printf("VIDIOCMCAPTURE: new size %dx%d\n", g_width, g_height);
			}
			break;
		}
		case VIDIOCSYNC:
		{
			struct video_mmap *vidmmap=arg;
			if ( !capture_mmap(vidmmap->frame) ) {
				printf("VIDIOCSYNC: failed frame %d\n", vidmmap->frame);
				return 1;
			}
			break;
		}
		default:
		{
			printf("ioctl %ld unhandled\n", cmd & 0xff);
			break;
		}
	}
	return 0;
}

void v4l_sighandler(int signo)
{
	int size, ret;
	unsigned long int cmd;
	struct pollfd ufds;

	if (signo!=SIGIO)
		return;
	ufds.fd=v4l_dev;
	ufds.events=POLLIN;
	ufds.revents=0;
	poll(&ufds, 1, 1000);
	if (!ufds.revents & POLLIN) {
		printf("Received signal but got negative on poll?!?!?!?\n");
		return;
	}
	size=read(v4l_dev, ioctlbuf, MAXIOCTL);
	if (size >= sizeof(unsigned long int)) {
		memcpy(&cmd, ioctlbuf, sizeof(unsigned long int));
		if (cmd==0) {
			printf("Client closed device\n");
			return;
		}
		ret=v4l_ioctl(cmd, ioctlbuf+sizeof(unsigned long int));
		if (ret) {
			memset(ioctlbuf+sizeof(unsigned long int), MAXIOCTL-sizeof(unsigned long int), 0xff);
			printf("ioctl %ld unsuccesfull\n", cmd & 0xff);
		}
		ioctl(v4l_dev, cmd, ioctlbuf+sizeof(unsigned long int));
	}
	return;
}

/***** MAIN *******************************************************************/

void signal_handler( int sig) {
	exit(0);
}

void cleanup(void) {
	if (v4l_dev)
		close(v4l_dev);
	if (handle) {
		if (g_v4l_mode != V4L_MODE_NONE) {
			dc1394_dma_unlisten( handle, &camera );
			dc1394_dma_release_camera( handle, &camera);
		}
		dc1394_destroy_handle(handle);
	}
	if (out_pipe)
		free(out_pipe);
	if (out_mmap)
		munmap(out_mmap, MAX_WIDTH * MAX_HEIGHT * MAX_BPP );
}

int main(int argc,char *argv[])
{
	get_options(argc,argv);
	
	if (g_daemon) {
		if (fork() == 0)
			setsid();
		else
			exit(0);
	}
	
	atexit(cleanup);
	
	if (dc_init() < 1) {
		fprintf(stderr, "no cameras found :(\n");
		exit(-1);
	}

	if ( !dc_start(g_v4l_fmt) ) {
		fprintf(stderr, "Failed to start dc1394 capture");
		exit(-1);
	}

	v4l_dev = v4l_open(v4l_dev_name);
	if (v4l_dev < 0) {
		perror ("Failed to open Video4Linux device");
		exit(-1);
	}
	
	if (g_v4l_mode == V4L_MODE_PIPE) {		
		if ( !v4l_start_pipe(g_width, g_height, g_v4l_fmt) ) {
			fprintf(stderr, "Failed to start vloopback pipe provider");
			exit(-1);
		}
	} else {
		if ( !v4l_start_mmap(MAX_WIDTH * MAX_HEIGHT * MAX_BPP * V4L_BUFFERS) ) {
			fprintf(stderr, "Failed to start vloopback mmap provider");
			exit(-1);
		}
	}
	
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGIO, v4l_sighandler);
	
	while (1) {
		if (g_v4l_mode == V4L_MODE_PIPE) {
			if (dc1394_dma_single_capture(&camera) == DC1394_SUCCESS) {
				capture_pipe( v4l_dev, (char *) camera.capture_buffer );
				dc1394_dma_done_with_buffer(&camera);
			}
		} else {
			pause();
		}
	}
	exit(0);
}
