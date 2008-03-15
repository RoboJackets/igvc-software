/**************************************************************************
**       Title: display video from multiple cameras/multiple cards
**    $RCSfile: dc1394_multiview.c,v $
**   $Revision: 1.9 $$Name:  $
**       $Date: 2004/08/10 07:57:22 $
**   Copyright: LGPL $Author: ddouxchamps $
** Description:
**
**    View format0-only camera video from one camera on one card,
**    muliple cameras on one card, or multiple cameras on multiple cards.
**
** TODO:
**    - Option to tile displays instead vertical stacking.
**
**-------------------------------------------------------------------------
**
**  $Log: dc1394_multiview.c,v $
**  Revision 1.9  2004/08/10 07:57:22  ddouxchamps
**  Removed extra buffering (Johann Schoonees)
**
**  Revision 1.8  2004/03/09 08:41:44  ddouxchamps
**  patch from Johann Schoonees for extra buffering
**
**  Revision 1.7  2004/01/20 16:14:01  ddennedy
**  fix segfault in dc1394_multiview
**
**  Revision 1.6  2004/01/20 04:12:27  ddennedy
**  added dc1394_free_camera_nodes and applied to examples
**
**  Revision 1.5  2003/09/02 23:42:36  ddennedy
**  cleanup handle destroying in examples; fix dc1394_multiview to use handle per camera; new example
**
**  Revision 1.4  2002/07/27 21:24:51  ddennedy
**  just increase buffers some to reduce chance of hangs
**
**  Revision 1.3  2002/07/27 04:45:07  ddennedy
**  added drop_frames option to dma capture, prepare versions/NEWS for 0.9 release
**
**  Revision 1.2  2002/07/24 02:22:40  ddennedy
**  cleanup, add drop frame support to dc1394_multiview
**
**  Revision 1.1  2002/07/22 02:57:02  ddennedy
**  added examples/dc1394_multiview to test/demonstrate dma multicapture over multiple ports
**
**
**************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/Xvlib.h>
#include <X11/keysym.h>
#define _GNU_SOURCE
#include <getopt.h>

#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>


/* uncomment the following to drop frames to prevent delays */
#define DROP_FRAMES 1
#define MAX_PORTS   4
#define MAX_CAMERAS 8
#define NUM_BUFFERS 8

/* ok the following constant should be by right included thru in Xvlib.h */
#ifndef XV_YV12
#define XV_YV12 0x32315659
#endif

#ifndef XV_YUY2
#define XV_YUY2 0x32595559
#endif

#ifndef XV_UYVY
#define XV_UYVY 0x59565955
#endif


/* declarations for libdc1394 */
int numPorts = MAX_PORTS;
raw1394handle_t handles[MAX_CAMERAS];
int numCameras = 0;
dc1394_cameracapture cameras[MAX_CAMERAS];
nodeid_t *camera_nodes;
dc1394_feature_set features;

/* declarations for video1394 */
char *device_name=NULL;

/* declarations for Xwindows */
Display *display=NULL;
Window window=(Window)NULL;
long width,height;
long device_width,device_height;
int connection=-1;
XvImage *xv_image=NULL;
XvAdaptorInfo *info;
long format=0;
GC gc;


/* Other declarations */
long frame_length;
long frame_free;
int frame=0;
int adaptor=-1;

int freeze=0;
int average=0;
int fps;
int res;
unsigned char *frame_buffer=NULL;


static struct option long_options[]={
	{"device",1,NULL,0},
	{"fps",1,NULL,0},
	{"res",1,NULL,0},
	{"help",0,NULL,0},
	{NULL,0,0,0}
	};

void get_options(int argc,char *argv[])
{
	int option_index=0;
	fps=7;
	res=0;
	
	while(getopt_long(argc,argv,"",long_options,&option_index)>=0){
		if(optarg){
			switch(option_index){ 
				/* case values must match long_options */
				case 0:
					device_name=strdup(optarg);
					break;
				case 1:
					fps=atoi(optarg);
					break;
				case 2: 
					res=atoi(optarg);
					break;
				}
			}
		if(option_index==3){
			printf( "\n"
			        "        %s - multi-cam monitor for libdc1394 and XVideo\n\n"
				"Usage:\n"
				"        %s [--fps=[1,3,7,15,30]] [--res=[0,1,2]] [--device=/dev/video1394/x]\n"
				"             --fps    - frames per second. default=7,\n"
				"                        30 not compatible with --res=2\n"
				"             --res    - resolution. 0 = 320x240 (default),\n"
				"                        1 = 640x480 YUV4:1:1, 2 = 640x480 RGB8\n"
				"             --device - specifies video1394 device to use (optional)\n"
				"                        default = /dev/video1394/<port#>\n"
				"             --help   - prints this message\n\n"
				"Keyboard Commands:\n"
				"        q = quit\n"
				"        < -or- , = scale -50%%\n"
				"        > -or- . = scale +50%%\n"
				"        0 = pause\n"
				"        1 = set framerate to 1.875 fps\n"
				"        2 = set framerate tp 3.75 fps\n"
				"        3 = set framerate to 7.5 fps\n"
				"        4 = set framerate to 15 fps\n"
			    "        5 = set framerate to 30 fps\n"
				,argv[0],argv[0]);
			exit(0);
			}
		}
	
}

/* image format conversion functions */

static inline
void iyu12yuy2 (unsigned char *src, unsigned char *dest, int NumPixels) {
  int i=0,j=0;
  register int y0, y1, y2, y3, u, v;
  while (i < NumPixels*3/2)
    {
      u = src[i++];
      y0 = src[i++];
      y1 = src[i++];
      v = src[i++];
      y2 = src[i++];
      y3 = src[i++];

      dest[j++] = y0;
      dest[j++] = u;
      dest[j++] = y1;
      dest[j++] = v;

      dest[j++] = y2;
      dest[j++] = u;
      dest[j++] = y3;
      dest[j++] = v;
    }
}


/* macro by Bart Nabbe */
#define RGB2YUV(r, g, b, y, u, v)\
  y = (9798*r + 19235*g + 3736*b)  / 32768;\
  u = (-4784*r - 9437*g + 14221*b)  / 32768 + 128;\
  v = (20218*r - 16941*g - 3277*b) / 32768 + 128;\
  y = y < 0 ? 0 : y;\
  u = u < 0 ? 0 : u;\
  v = v < 0 ? 0 : v;\
  y = y > 255 ? 255 : y;\
  u = u > 255 ? 255 : u;\
  v = v > 255 ? 255 : v

static inline
void rgb2yuy2 (unsigned char *RGB, unsigned char *YUV, int NumPixels) {
  int i, j;
  register int y0, y1, u0, u1, v0, v1 ;
  register int r, g, b;

  for (i = 0, j = 0; i < 3 * NumPixels; i += 6, j += 4)
    {
      r = RGB[i + 0];
      g = RGB[i + 1];
      b = RGB[i + 2];
      RGB2YUV (r, g, b, y0, u0 , v0);
      r = RGB[i + 3];
      g = RGB[i + 4];
      b = RGB[i + 5];
      RGB2YUV (r, g, b, y1, u1 , v1);
      YUV[j + 0] = y0;
      YUV[j + 1] = (u0+u1)/2;
      YUV[j + 2] = y1;
      YUV[j + 3] = (v0+v1)/2;
    }
}

/* helper functions */

void set_frame_length(long size, int numCameras)
{
	frame_length=size;
	fprintf(stderr,"Setting frame size to %ld kb\n",size/1024);
	frame_free=0;
  	frame_buffer = malloc( size * numCameras);
}

void display_frames()
{
	int i;
	
	if(!freeze && adaptor>=0){
		for (i = 0; i < numCameras; i++)
		{
			switch (res) {
			case MODE_640x480_YUV411:
				iyu12yuy2( (unsigned char *) cameras[i].capture_buffer,
					frame_buffer + (i * frame_length),
					(device_width*device_height) );
				break;
				
			case MODE_320x240_YUV422:
			case MODE_640x480_YUV422:
				memcpy( frame_buffer + (i * frame_length),
					cameras[i].capture_buffer, device_width*device_height*2);
				break;
					
			case MODE_640x480_RGB:
				rgb2yuy2( (unsigned char *) cameras[i].capture_buffer,
					frame_buffer + (i * frame_length),
					(device_width*device_height) );
				break;
			}
		}
		
		xv_image=XvCreateImage(display,info[adaptor].base_id,format,frame_buffer,
			device_width,device_height * numCameras);
		XvPutImage(display,info[adaptor].base_id,window,gc,xv_image,
			0,0,device_width,device_height * numCameras,
			0,0,width,height);

		xv_image=NULL;
	}
}

void QueryXv()
{
	int num_adaptors;
	int num_formats;
	XvImageFormatValues *formats=NULL;
	int i,j;
	char xv_name[5];
	
	XvQueryAdaptors(display,DefaultRootWindow(display),&num_adaptors,&info);
	
	for(i=0;i<num_adaptors;i++) {
		formats=XvListImageFormats(display,info[i].base_id,&num_formats);
		for(j=0;j<num_formats;j++) {
			xv_name[4]=0;
			memcpy(xv_name,&formats[j].id,4);
			if(formats[j].id==format) {
				fprintf(stderr,"using Xv format 0x%x %s %s\n",formats[j].id,xv_name,(formats[j].format==XvPacked)?"packed":"planar");
				if(adaptor<0)adaptor=i;
			}
		}
	}
		XFree(formats);
	if(adaptor<0)
		fprintf(stderr,"No suitable Xv adaptor found");	
	
}


void cleanup(void) {
	int i;
	for (i=0; i < numCameras; i++)
	{
		dc1394_dma_unlisten( handles[i], &cameras[i] );
		dc1394_dma_release_camera( handles[i], &cameras[i]);
		dc1394_destroy_handle(handles[i]);
	}
	if ((void *)window != NULL)
		XUnmapWindow(display,window);
	if (display != NULL)
		XFlush(display);
	if (frame_buffer != NULL)
		free( frame_buffer );
}


/* trap ctrl-c */
void signal_handler( int sig) {
	signal( SIGINT, SIG_IGN);
	cleanup();
	exit(0);
}

int main(int argc,char *argv[])
{
	XEvent xev;
	XGCValues xgcv;
	long background=0x010203;
	unsigned int channel;
	unsigned int speed;
	int i, p;
	raw1394handle_t raw_handle;
	struct raw1394_portinfo ports[MAX_PORTS];

	get_options(argc,argv);
	/* process options */
	switch(fps) {
		case 1: fps =	FRAMERATE_1_875; break;
		case 3: fps =	FRAMERATE_3_75; break;
		case 15: fps = FRAMERATE_15; break;
		case 30: fps = FRAMERATE_30; break;
		case 60: fps = FRAMERATE_60; break;
		default: fps = FRAMERATE_7_5; break;
	}
	switch(res) {
		case 1: 
			res = MODE_640x480_YUV411; 
			device_width=640;
			device_height=480;
			format=XV_YUY2;
			break;
		case 2: 
			res = MODE_640x480_RGB; 
			device_width=640;
			device_height=480;
			format=XV_YUY2;
			break;
		default: 
			res = MODE_320x240_YUV422;
			device_width=320;
			device_height=240;
			format=XV_UYVY;
			break;
	}

	/* get the number of ports (cards) */
	raw_handle = raw1394_new_handle();
    if (raw_handle==NULL) {
        perror("Unable to aquire a raw1394 handle\n");
        perror("did you load the drivers?\n");
        exit(-1);
    }
	
	numPorts = raw1394_get_port_info(raw_handle, ports, numPorts);
	raw1394_destroy_handle(raw_handle);
	printf("number of ports = %d\n", numPorts);

	/* get dc1394 handle to each port */
	for (p = 0; p < numPorts; p++)
	{
		int camCount;
		
		/* get the camera nodes and describe them as we find them */
		raw_handle = raw1394_new_handle();
		raw1394_set_port( raw_handle, p );
		camera_nodes = dc1394_get_camera_nodes(raw_handle, &camCount, 1);
		raw1394_destroy_handle(raw_handle);

		/* setup cameras for capture */
		for (i = 0; i < camCount; i++)
		{	
			handles[numCameras] = dc1394_create_handle(p);
			if (handles[numCameras]==NULL) {
				perror("Unable to aquire a raw1394 handle\n");
				perror("did you load the drivers?\n");
				cleanup();
				exit(-1);
			}

			cameras[numCameras].node = camera_nodes[i];
		
			if(dc1394_get_camera_feature_set(handles[numCameras], cameras[numCameras].node, &features) !=DC1394_SUCCESS) 
			{
				printf("unable to get feature set\n");
			} else {
				dc1394_print_feature_set(&features);
			}
		
			if (dc1394_get_iso_channel_and_speed(handles[numCameras], cameras[numCameras].node,
										 &channel, &speed) != DC1394_SUCCESS) 
			{
				printf("unable to get the iso channel number\n");
				cleanup();
				exit(-1);
			}
			 
			if (dc1394_dma_setup_capture(handles[numCameras], cameras[numCameras].node, i+1 /*channel*/,
									FORMAT_VGA_NONCOMPRESSED, res,
									SPEED_400, fps, NUM_BUFFERS, DROP_FRAMES,
									device_name, &cameras[numCameras]) != DC1394_SUCCESS) 
			{
				fprintf(stderr, "unable to setup camera- check line %d of %s to make sure\n",
					   __LINE__,__FILE__);
				perror("that the video mode,framerate and format are supported\n");
				printf("is one supported by your camera\n");
				cleanup();
				exit(-1);
			}
		
		
			/*have the camera start sending us data*/
			if (dc1394_start_iso_transmission(handles[numCameras], cameras[numCameras].node) !=DC1394_SUCCESS) 
			{
				perror("unable to start camera iso transmission\n");
				cleanup();
				exit(-1);
			}
			numCameras++;
		}
		dc1394_free_camera_nodes(camera_nodes);
	}

	fflush(stdout);
	if (numCameras < 1) {
		perror("no cameras found :(\n");
		cleanup();
		exit(-1);
	}


	switch(format){
		case XV_YV12:
			set_frame_length(device_width*device_height*3/2, numCameras);
			break;
		case XV_YUY2:
		case XV_UYVY:
			set_frame_length(device_width*device_height*2, numCameras);
			break;
		default:
			fprintf(stderr,"Unknown format set (internal error)\n");
			exit(255);
	}

	/* make the window */
	display=XOpenDisplay(getenv("DISPLAY"));
	if(display==NULL)
	{
		fprintf(stderr,"Could not open display \"%s\"\n",getenv("DISPLAY"));
		cleanup();
		exit(-1);
	}
	
	QueryXv();
	
	if ( adaptor < 0 )
	{
		cleanup();
		exit(-1);
	}
	
	width=device_width;
	height=device_height * numCameras;

	window=XCreateSimpleWindow(display,DefaultRootWindow(display),0,0,width,height,0,
		WhitePixel(display,DefaultScreen(display)),
		background);
		
	XSelectInput(display,window,StructureNotifyMask|KeyPressMask);
	XMapWindow(display,window);
	connection=ConnectionNumber(display);
	
	gc=XCreateGC(display,window,0,&xgcv);
	
	
	/* main event loop */	
	while(1){

		dc1394_dma_multi_capture(cameras, numCameras);
		
		display_frames();
		XFlush(display);

		while(XPending(display)>0){
			XNextEvent(display,&xev);
			switch(xev.type){
				case ConfigureNotify:
					width=xev.xconfigure.width;
					height=xev.xconfigure.height;
					display_frames();
					break;
				case KeyPress:
					switch(XKeycodeToKeysym(display,xev.xkey.keycode,0)){
						case XK_q:
						case XK_Q:
							cleanup();
							exit(0);
							break;
						case XK_comma:
						case XK_less:
							width=width/2;
							height=height/2;
							XResizeWindow(display,window,width,height);
							display_frames();
							break;
						case XK_period:
						case XK_greater:
							width=2*width;
							height=2*height;
							XResizeWindow(display,window,width,height);
							display_frames();
							break;
						case XK_0:
							freeze = !freeze;
							break;
						case XK_1:
							fps =	FRAMERATE_1_875; 
							for (i = 0; i < numCameras; i++)
								dc1394_set_video_framerate(handles[i], cameras[i].node, fps);
							break;
						case XK_2:
							fps =	FRAMERATE_3_75; 
							for (i = 0; i < numCameras; i++)
								dc1394_set_video_framerate(handles[i], cameras[i].node, fps);
							break;
						case XK_4:
							fps = FRAMERATE_15; 
							for (i = 0; i < numCameras; i++)
								dc1394_set_video_framerate(handles[i], cameras[i].node, fps);
							break;
						case XK_5: 
							fps = FRAMERATE_30;
							for (i = 0; i < numCameras; i++)
								dc1394_set_video_framerate(handles[i], cameras[i].node, fps);
							break;
						case XK_3:
							fps = FRAMERATE_7_5; 
							for (i = 0; i < numCameras; i++)
								dc1394_set_video_framerate(handles[i], cameras[i].node, fps);
							break;
						}
					break;
				}
			} /* XPending */

			for (i = 0; i < numCameras; i++)
			{
				dc1394_dma_done_with_buffer(&cameras[i]);
			}
		
		} /* while not interrupted */

	exit(0);
}
