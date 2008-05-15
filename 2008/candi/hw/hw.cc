#include "hw.h"
#include <libplayerc++/playerc++.h>


using namespace PlayerCc;

static bool isInitialConnect;

static PlayerClient* localRC = NULL;		// NULL if unavailable
static PlayerClient* remoteRC = NULL;		// NULL if unavailable

static CameraProxy* camera = NULL;			// NULL if unavailable
static Position2dProxy* motors = NULL;		// NULL if unavailable
static LaserProxy* laser = NULL;			// NULL if unavailable

// -----

static PlayerClient* GetLocalRC();
static PlayerClient* GetRemoteRC();

static CameraProxy* GetCamera();
static Position2dProxy* GetMotors();
static LaserProxy* GetLaser();

static void InitCameraFrame();

// -----

void InitHW() {
	isInitialConnect = TRUE;
	{
		// Connect to the robot controllers
		GetLocalRC();
		GetRemoteRC();
		
		// Connect to the devices
		GetCamera();
		GetMotors();
		GetLaser();
	}
	isInitialConnect = FALSE;
}

void UpdateSensors() {
	//PlayerClient* localRC = GetLocalRC();
	//PlayerClient* remoteRC = GetRemoteRC();
	
	if (localRC != NULL) localRC->Read();
	if (remoteRC != NULL) remoteRC->Read();
}

// -----

static PlayerClient* GetLocalRC() {
	if (localRC == NULL && (isInitialConnect || SUPPORT_PLUG_AND_PLAY)) {
		printf("Connecting to local robot controller... ");
		fflush(stdout);
		try {
			localRC = new PlayerClient("localhost", PLAYER_PORTNUM);
			localRC->SetDataMode(PLAYER_DATAMODE_PULL);
			printf("ok\n");
		} catch (PlayerError e) {
			printf("error\n");
			if (isInitialConnect) {
				printf("ERROR: Unable to connect to local robot controller:\n");
				printf("       %s\n", (char*) e.GetErrorStr().c_str());
			}
		}
	}
	return localRC;
}

static PlayerClient* GetRemoteRC() {
	if (remoteRC == NULL && (isInitialConnect || SUPPORT_PLUG_AND_PLAY)) {
#if LOOK_FOR_REMOTE_RC
		printf("Connecting to remote robot controller... ");
		fflush(stdout);
		try {
			remoteRC = new PlayerClient("192.168.10.100", PLAYER_PORTNUM);
			remoteRC->SetDataMode(PLAYER_DATAMODE_PULL);
			printf("ok\n");
		} catch (PlayerError e) {
			printf("error\n");
			if (isInitialConnect) {
				printf("ERROR: Unable to connect to remote robot controller:\n");
				printf("       %s\n", (char*) e.GetErrorStr().c_str());
			}
		}
#endif
	}
	return remoteRC;
}

// -----

static CameraProxy* GetCamera() {
	if (camera == NULL && (isInitialConnect || SUPPORT_PLUG_AND_PLAY)) {
		InitCameraFrame();
		
		PlayerClient* localRC = GetLocalRC();
		if (localRC != NULL) {
			try {
				camera = new PlayerCc::CameraProxy(localRC,0);	// use camera:0
				localRC->SetReplaceRule((bool) 1, -1, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_CODE);
				printf("Connected to camera.\n");
			} catch (PlayerError e) {
				if (isInitialConnect) {
					printf("ERROR: Unable to connect to camera:\n");
					printf("       %s\n", (char*) e.GetErrorStr().c_str());
				}
			}
		}
	}
	return camera;
}

static Position2dProxy* GetMotors() {
	if (motors == NULL && (isInitialConnect || SUPPORT_PLUG_AND_PLAY)) {
		PlayerClient* localRC = GetLocalRC();
		if (localRC != NULL) {
			try {
				motors = new Position2dProxy(localRC,0);		// use position2d:0
				printf("Connected to motors.\n");
			} catch (PlayerError e) {
				if (isInitialConnect) {
					printf("ERROR: Unable to connect to motors:\n");
					printf("       %s\n", (char*) e.GetErrorStr().c_str());
				}
			}
		}
	}
	return motors;
}

static LaserProxy* GetLaser() {
	if (laser == NULL && (isInitialConnect || SUPPORT_PLUG_AND_PLAY)) {
		PlayerClient* remoteRC = GetRemoteRC();
		if (remoteRC != NULL) {
			try {
				laser = new LaserProxy(remoteRC,0);				// use laser:0
				printf("Connected to LIDAR.\n");
			} catch (PlayerError e) {
				if (isInitialConnect) {
					printf("ERROR: Unable to connect to LIDAR:\n");
					printf("       %s\n", (char*) e.GetErrorStr().c_str());
				}
			}
		}
	}
	return laser;
}

// -----

static Image curFrame;

static void InitCameraFrame() {
	curFrame.data=NULL;
	curFrame.width=0;
	curFrame.height=0;
}

// Resizes the image buffer that received images are stored into
static void resizeImage(uint width, uint height) {
	if ((width == curFrame.width) && (height==curFrame.height)) return;
	curFrame.width=width;
	curFrame.height=height;
	
	if (curFrame.data) free(curFrame.data);
	if (width != 0 || height != 0)
		curFrame.data = (char *) malloc(width*height*3);
}

Image* GetCameraFrame() {
	CameraProxy* camera = GetCamera();
	if (camera == NULL) {
		resizeImage(0,0);
	} else {
		/* Block until we have a new camera image */
#if 1
		// XXX: This function is non-standard. It needs to be added to the official
		//      Player codebase at some point.
		while (!camera->HasNewImage()) {
			UpdateSensors();
		}
#else
		// XXX: This function is non-standard. It needs to be added to the official
		//      Player codebase at some point.
		camera->WaitForNewImage();		// blocks until a new camera image has been received
#endif
		
		/* Get the camera image */
		resizeImage(camera->GetWidth(), camera->GetHeight());
		camera->GetImage((uint8_t*) curFrame.data);
	}
	
	return &curFrame;
}

// -----

void SetMotorOutput(MotorOutput motorOutput) {
	Position2dProxy* motors = GetMotors();
	if (motors != NULL) {
		motors->SetSpeed(
			(double) motorOutput.leftSpeed*2,		// output: -255 to 255
			(double) motorOutput.rightSpeed*2);		// output: -255 to 255
	}
}

// -----

/**
 * Returns the range (in meters) of the obstacle at the specified
 * bearing (in radians), or -1 if an error occurred.
 */
double GetLaserRangeAtBearing(double bearing) {
	LaserProxy* laser = GetLaser();
	if (laser == NULL) return -1;
	
	long aIndex = (long) ((bearing - laser->GetMinAngle()) / laser->GetScanRes());
	if ((aIndex < 0) || (aIndex >= (long) laser->GetCount())) return -1;
	return laser->GetRange(aIndex);
}
