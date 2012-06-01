#include "DCam.h"
#include "DCam_Config.h"

#include <stdlib.h>
#include <stdexcept>
#include <dc1394/conversions.h>

#include <QMutexLocker>
#include <QVBoxLayout>
#include <QLabel>

#include <iostream>

using namespace std;
using namespace Camera;

//static vars//
std::vector<DCam*> DCam::cameras;

dc1394_t *DCam::_dc1394 = 0;

DCam::DCam(uint64_t guid)
		: Base(),resetCallback(NULL)
{
	_initialized = false;
	_config = 0;
	_name = "DCam";

	if (!_dc1394)
	{
		init();
	}

	_camera = dc1394_camera_new(_dc1394, guid);
	if (!_camera)
	{
		throw runtime_error("Failed to create camera");
	}

	DCam::cameras.push_back(this);

	_image = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
}

DCam::~DCam()
{
	if (_camera)
	{
		close();

		dc1394_camera_free(_camera);
		_camera = 0;
	}
}

void DCam::reset1394Bus()
{
	if (_camera)
	{
		dc1394_reset_bus(_camera);
	}
}

void DCam::open()
{
	if (!_initialized && _camera)
	{
		// 15fps causes data to be dropped on the old laptop, resulting in bad colors.  Don't use it.
		if (dc1394_video_set_framerate(	_camera,
										//DC1394_FRAMERATE_60
										DC1394_FRAMERATE_30
										//DC1394_FRAMERATE_15
									  ))
		{
			throw runtime_error("Unable to set framerate.");
		}

		if (dc1394_video_set_mode(_camera, DC1394_VIDEO_MODE_640x480_MONO8))
		{
			throw runtime_error("Unable to set video mode.");
		}

		//if (dc1394_capture_setup(_camera, 4, DC1394_CAPTURE_FLAGS_DEFAULT))
		if (dc1394_capture_setup(_camera, 1, DC1394_CAPTURE_FLAGS_DEFAULT))//the 1 is frames in ring buffer
		{
			throw runtime_error("Unable to setup capture.");
		}

		if (dc1394_video_set_transmission(_camera, DC1394_ON))
		{
			throw runtime_error("Unable to start ISO transmission.");
		}

		printf("Initialized camera %016llx\n", (long long unsigned int)_camera->guid);

		_initialized = true;
	}
	else
	{
		printf("Tried to open already open camera %016llx\n", (long long unsigned int)_camera->guid);
	}
}

void DCam::close()
{
	if (_initialized && _camera)
	{
		//turn off ISO in case it is on
		dc1394_video_set_transmission(_camera, DC1394_OFF);

		//stop capture
		dc1394_capture_stop(_camera);

		_initialized = false;
	}
}

bool DCam::is_open()
{
	return _camera != 0;
}

QWidget *DCam::configuration()
{
	if (!_config)
	{
		_config = new DCam_Config(this);
	}

	return _config;
}

QSize DCam::size()
{
	return QSize(_image->width, _image->height);
}

IplImage *DCam::read_frame()
{
	if (!_camera)
	{
		return _image;
	}

	try
	{
		dc1394error_t ec;
		if( (ec=dc1394_capture_dequeue(_camera, DC1394_CAPTURE_POLICY_WAIT, &_frame)) )
		{
			std::cout<<"libdc1394 error reading frame from dc1394_capture_dequeue, error code"<< ec << endl;
			return 0;
		};
		//dc1394_capture_dequeue(_camera, DC1394_CAPTURE_POLICY_POLL, &_frame);

		if (_frame)
		{
			//QMutexLocker ml(&_camera_thread->mutex);
			// The camera is actually BGGR, but OpenCV expects BGR while libdc1394 provides RGB,
			// so swap the R and B channels here.
			dc1394_bayer_decoding_8bit(	_frame->image,
										(uint8_t *)_image->imageData,
										_image->width,
										_image->height,
										DC1394_COLOR_FILTER_RGGB,
										DC1394_BAYER_METHOD_SIMPLE
									  );
		}
		else
		{
			printf("no frame\n");
			return 0;
		}

		//release the buffer for next frame
		dc1394_capture_enqueue(_camera, _frame);

	}
	catch (const char* err)
	{
		printf("Error: %s\n", err);
	}

	return _image;
}

int DCam::resetCamera(){
//Forces libdc1394 to reset, for when camera temporarily dies.
//
//In order to avoid possible segfaults we don't try to clean 
//up old stuff, so there is a memory leak on calling this.
	int error =1;
	while(error){
		//reset libdc
		dc1394_free(_dc1394);
		_dc1394 = dc1394_new();
		cout<<"_dc1394 "<< _dc1394 << endl;
		if(!_dc1394)continue;
		
		//list cameras
		dc1394camera_list_t *list = 0;
		if(DC1394_SUCCESS!=dc1394_camera_enumerate(_dc1394, &list))continue;
		cout<<"list*  "<< list << endl;
		if(!list)continue;
		
		//check num cams, must be > 0
		cout<<"list->num  "<< list->num << endl;
		if(!list->num)continue;
		
		//connect to first camera
		_camera = dc1394_camera_new(_dc1394, list->ids[0].guid);
		cout<<"_camera "<< _camera << endl;
		if(!_camera)continue;
		
		//open camera stream
		_initialized=false;
		try{
			open();
		}
		catch(std::exception)
		{
			cout<<"Camera stream opening failed!"<< endl;
			continue;
		}
		
		//call reset callback if one has been registered
		if (resetCallback)
		{
			if(!resetCallback(this))continue;
		}
		
		//it all worked! we're back up
		error=0;
	}
	return 1;
}

int DCam::registerResetCallback(int (*callback)(DCam*)){
	this->resetCallback=callback;
	return 1;
}


void DCam::exposure(unsigned int level)
{
	dc1394_feature_set_value(_camera,  DC1394_FEATURE_EXPOSURE, level);
}

void DCam::init()
{
	_dc1394 = dc1394_new();

	//cleanup on exit
	atexit(DCam::destroy);
}

void DCam::scanCameras()
{
	if (!_dc1394)
	{
		init();
	}

	dc1394camera_list_t *list = 0;

	if (dc1394_camera_enumerate(_dc1394, &list) || !list)
	{
		throw runtime_error("Unable to get raw 1394 handle.");
	}

	for (unsigned int i = 0; i < list->num; ++i)
	{
		new DCam(list->ids[i].guid);
	}

	dc1394_camera_free_list(list);
}

void DCam::destroy()
{
	for (unsigned int i = 0; i < cameras.size(); ++i)
	{
		delete cameras[i];
	}

	cameras.clear();
}
