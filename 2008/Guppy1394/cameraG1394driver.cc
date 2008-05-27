/*
 * This is a generic driver for the Guppy camera over the Firewire (IEEE 1394) protocol.
 * 
 * By ctm
 */
/*
 * This code is based on:
 		the cameradv1394 folder
 		and the grab_color_image.c example
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>

#include "raw1394.h"
#include <stdio.h>

#include "dc1394_control.h"
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#define _GNU_SOURCE_2
#include <getopt.h>

#define MAX_PORTS 4
#define MAX_RESETS 10
u_int64_t g_guid = 0;
// -----------------------------------------------------------------------



// -----------------------------------------------------------------------

class CameraG1394
{
public:
	CameraG1394(const char* camDeviceFilepath);
	~CameraG1394();
	
	/** Connects to the camera. Returns true if successful. */
	bool Connect();
	/** Disconnects from the camera. */
	void Disconnect();
	/** Returns whether this driver is connected to the camera. */
	bool IsConnected() { return this->valid; };
	
	/**
	 * Grabs a frame from the camera.
	 * 
	 * @return	a frame in RGB888 format, or <tt>null</tt> if an error occurred.
	 */
	unsigned char* GrabFrame();
	
	int GetFrameWidth() { return camera.frame_width; }
	int GetFrameHeight() { return camera.frame_height; }

private:
	void Resize(int width, int height);
	bool initG();

private:
	// This is the path of the camera device in the filesystem.
	const char* camDeviceFilepath;
	
	bool valid;

	
	///////
  FILE* imagefile;
  dc1394_cameracapture camera;
  struct raw1394_portinfo ports[MAX_PORTS];
  int numPorts;
  int numCameras;
  int i, j;
  int found;
  raw1394handle_t handle;
  nodeid_t * camera_nodes;

	///////
	
};

// -----------------------------------------------------------------------
#pragma mark -

CameraG1394::CameraG1394(const char* camDeviceFilepath) 
	: valid(false)

{
	this->camDeviceFilepath = camDeviceFilepath;
	

	///////
	//initG();

	///////
	
	
}

CameraG1394::~CameraG1394() {
	// Nothing to uninit
}

// from grab_color_image.c example
bool CameraG1394::initG(){
  numPorts = 0;
  numCameras = 0;
  found = 0;
  nodeid_t * camera_nodes = NULL;

  /*-----------------------------------------------------------------------
   *  Open ohci and asign handle to it
   *-----------------------------------------------------------------------*/
  handle = raw1394_new_handle();
  if (handle==NULL)
  {
    fprintf( stderr, "Unable to aquire a raw1394 handle\n\n"
             "Please check \n"
	     "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
	     "  - if you have read/write access to /dev/raw1394\n\n");
    return false;
  }
	/* get the number of ports (cards) */
  numPorts = raw1394_get_port_info(handle, ports, numPorts);
  raw1394_destroy_handle(handle);
  handle = NULL;
  
  for (j = 0; j < MAX_RESETS && found == 0; j++)
  {
    /* look across all ports for cameras */
    for (i = 0; i < numPorts && found == 0; i++)
    {
      if (handle != NULL)
        dc1394_destroy_handle(handle);
      handle = dc1394_create_handle(i);
      if (handle == NULL)
      {
        fprintf( stderr, "Unable to aquire a raw1394 handle for port %i\n", i);
        return false;
      }
      numCameras = 0;
      camera_nodes = dc1394_get_camera_nodes(handle, &numCameras, 0);
      if (numCameras > 0)
      {
        if (g_guid == 0)
        {
          dc1394_camerainfo info;
          /* use the first camera found */
          camera.node = camera_nodes[0];
          if (dc1394_get_camera_info(handle, camera_nodes[0], &info) == DC1394_SUCCESS)
            dc1394_print_camera_info(&info);
          found = 1;
        }
        else
        {
          /* attempt to locate camera by guid */
          int k;
          for (k = 0; k < numCameras && found == 0; k++)
          {
            dc1394_camerainfo info;
            if (dc1394_get_camera_info(handle, camera_nodes[k], &info) == DC1394_SUCCESS)
            {
              if (info.euid_64 == g_guid)
              {
                dc1394_print_camera_info(&info);
                camera.node = camera_nodes[k];
                found = 1;
              }
            }
          }
        }
        if (found == 1)
        {
          /* camera can not be root--highest order node */
          if (camera.node == raw1394_get_nodecount(handle)-1)
          {
            /* reset and retry if root */
            raw1394_reset_bus(handle);
            sleep(2);
            found = 0;
          }
        }
        dc1394_free_camera_nodes(camera_nodes);
      } /* cameras >0 */
    } /* next port */
  } /* next reset retry */
  
  if (found == 0 && g_guid != 0)
  {
    fprintf( stderr, "Unable to locate camera node by guid\n");
    return false;
  }
  else if (numCameras == 0)
  {
    fprintf( stderr, "no cameras found :(\n");
    dc1394_destroy_handle(handle);
    return false;
  }
  if (j == MAX_RESETS)
  {
    fprintf( stderr, "failed to not make camera root node :(\n");
    dc1394_destroy_handle(handle);
    return false;
  }
  
  /*-----------------------------------------------------------------------
   *  setup capture
   *-----------------------------------------------------------------------*/
  if (dc1394_setup_capture(handle, camera.node,
                           0, /* channel */ 
                           FORMAT_VGA_NONCOMPRESSED,
                           MODE_640x480_MONO,
                           SPEED_400,
                           FRAMERATE_15,
                           &camera)!=DC1394_SUCCESS) 
  {
    fprintf( stderr,"unable to setup camera-\n"
             "check line %d of %s to make sure\n"
             "that the video mode,framerate and format are\n"
             "supported by your camera\n",
             __LINE__,__FILE__);
    dc1394_release_camera(handle,&camera);
    dc1394_destroy_handle(handle);
    return false;
  }
  
  /*-----------------------------------------------------------------------
   *  have the camera start sending us data
   *-----------------------------------------------------------------------*/
  if (dc1394_start_iso_transmission(handle,camera.node)
      !=DC1394_SUCCESS) 
  {
    fprintf( stderr, "unable to start camera iso transmission\n");
    dc1394_release_camera(handle,&camera);
    dc1394_destroy_handle(handle);
    return false;
  }
  
	  	valid=true;
	  	return true;
}

bool CameraG1394::Connect() {

	return initG();
}

void CameraG1394::Disconnect() {
    dc1394_release_camera(handle,&camera);
    dc1394_destroy_handle(handle);
}

unsigned char* CameraG1394::GrabFrame() {
  if (dc1394_single_capture(handle,&camera)!=DC1394_SUCCESS) 
  {
    fprintf( stderr, "unable to capture a frame\n");

    return false;
  }
	return (unsigned char *)camera.capture_buffer;
}

void CameraG1394::Resize(int width, int height)
{

}
