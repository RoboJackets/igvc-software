#include "GuppyCam.h"
#include "XmlConfiguration.h"
#include "image_buffers.h"


GuppyCam::GuppyCam() {}

GuppyCam::~GuppyCam() {}

int GuppyCam::connect()
{
	try
	{
		Camera::DCam::scanCameras();
	}
	catch (std::exception)
	{
		camconnected=0;
		//return 0;
		printf("Error connecting to camera \n");
		exit(-1);
	}

	std::vector<Camera::DCam*> cam = Camera::DCam::availableCameras();
	if (cam.size()==0)
	{
		printf("Error connecting to camera \n");
		return 0;
	}
	_dcam = cam[0];
	_dcam->open();
	_camera = _dcam->camera();

	if (_dcam->is_open())
	{
		camconnected=1;
		return 1;
	}
	else
	{
		camconnected=0;
		//return 0;
		printf("Error connecting to camera \n");
		exit(-1);
	}
}

int GuppyCam::isValid()
{
	return camconnected;
}

bool GuppyCam::GrabCvImage()
{
	ImageBufferManager::getInstance().visCvRaw = _dcam->read_frame();
	return true;
}

IplImage * GuppyCam::ReturnFrame()
{
	return _dcam->read_frame();
}


void GuppyCam::setGain(int value)
{
	_feature = DC1394_FEATURE_GAIN;
	if ( DC1394_SUCCESS != dc1394_feature_set_value(_camera, _feature, value))
		printf("error setting gain \n");
}

void GuppyCam::setShutter(int value)
{
	_feature = DC1394_FEATURE_SHUTTER;
	if ( DC1394_SUCCESS != dc1394_feature_set_value(_camera, _feature, value))
		printf("error setting shutter \n");
}

void GuppyCam::setGamma(int value)
{
	_feature = DC1394_FEATURE_GAMMA;
	if ( DC1394_SUCCESS != dc1394_feature_set_value(_camera, _feature, value))
		printf("error setting gamma \n");
}

void GuppyCam::setWhiteBalance(uint32_t red, uint32_t blue)
{
	_feature = DC1394_FEATURE_WHITE_BALANCE;
	if ( DC1394_SUCCESS !=  dc1394_feature_whitebalance_set_value(_camera, blue, red))
		printf("error setting whitebalance \n");

}

void GuppyCam::setAllAuto(bool useauto)
{
	dc1394feature_mode_t mode;
	if (useauto)	mode = DC1394_FEATURE_MODE_AUTO;
	else mode = DC1394_FEATURE_MODE_MANUAL;

	_feature =  DC1394_FEATURE_SHUTTER;
	if ( DC1394_SUCCESS !=  dc1394_feature_set_mode(_camera, _feature, mode))
		printf("error setting auto shutter \n");

	_feature =  DC1394_FEATURE_GAIN;
	if ( DC1394_SUCCESS !=  dc1394_feature_set_mode(_camera, _feature, mode))
		printf("error setting auto gain \n");

}

void GuppyCam::loadSettings()
{

//	setAllAuto(); // for now use auto
//	setWhiteBalance( 523, 646 );


	// load xml settings
	XmlConfiguration cfg("Config.xml");
	int gain = cfg.getInt("gain");
	int shutter = cfg.getInt("shutter");
	int gamma = cfg.getInt("gamma");
	int red 	= cfg.getInt("red");
	int blue = cfg.getInt("blue");
	int use_auto = cfg.getInt("auto_gain_and_shutter");

	// sanity check
	if (gain==-1 || shutter==-1)
	{
		// load failed
		use_auto = 1;
	}

	// auto adjust gain/shutter check
	if (use_auto)
	{
		setAllAuto(true);
		printf("Using auto adjust camera settings \n");
	}
	else
	{
		setAllAuto(false);
		// set values
		setGain( gain );
		setShutter( shutter );
		//setGamma( gamma );
		//setWhiteBalance( red, blue );
	}

	// set other values
	setGamma( gamma );
	setWhiteBalance( red, blue );

}






