#include <stdlib.h>

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QGridLayout>
#include <QFrame>
#include <QPainter>
#include <QSize>
#include <QPoint>
#include <QTimer>
#include <QImage>

#include <libplayerc++/playerc++.h>

// -------------------------------------------------------------------------------------------

extern int gCameraID;
extern const char* gPlayerServerHostname;
extern int gPlayerServerPort;

// -------------------------------------------------------------------------------------------

class RenderArea : public QWidget
{
	Q_OBJECT

public:
	RenderArea(QWidget *parent = 0) : QWidget(parent), imageRGB24(NULL), imageRGB32(NULL) {
		setAutoFillBackground(true);
		
		// Initialize random seed
		srand(time(NULL));
		
		printf("Connecting to robot...\n");
		printf("  Connecting to '%s' on port %d...\n", gPlayerServerHostname, gPlayerServerPort);
		this->robot = new PlayerCc::PlayerClient(gPlayerServerHostname, gPlayerServerPort);
		printf("  Connecting to 'camera:%d'...\n", gCameraID);
		this->camera = new PlayerCc::CameraProxy(robot, gCameraID);
		printf("Done\n");
		
		// Set data to PULL mode
		// (i.e., always get the *latest* camera images, even if some frames are dropped)
		//robot->SetDataMode(PLAYER_DATAMODE_PULL);
		robot->SetReplaceRule(1, -1, PLAYER_MSGTYPE_DATA);
	}
	
	~RenderArea() {
		delete(this->camera);
		delete(this->robot);
		delete(this->imageRGB24);
		delete(this->imageRGB32);
	}
	
	QSize minimumSizeHint() const { return QSize(640,480); }
	QSize sizeHint() const { return QSize(640,480); }

protected:
	void paintEvent(QPaintEvent *event) {
		//QPainter painter(this);
		//painter.drawLine(QPoint(10 + (rand() % 100),10), QPoint(100,100));
		
		robot->Read();
		//camera->Decompress();
		printf("Image: %d x %d\n", camera->GetWidth(), camera->GetHeight());
		//camera->SaveFrame("camera");
		
		resizeImage(camera->GetWidth(), camera->GetHeight());
		camera->GetImage(imageRGB24);
		calculateRGB32();
		QImage theImage(
			(unsigned char*) imageRGB32,
			camera->GetWidth(),
			camera->GetHeight(),
			QImage::Format_RGB32
			/*32, 0, 0, QImage::LittleEndian*/);	// NOTE: try BigEndian if this doesn't work
		
		QPainter painter(this);
		painter.drawImage(0, 0, theImage);
	}

private:
	unsigned char* imageRGB24;
	unsigned char* imageRGB32;
	
	void resizeImage(int width, int height) {
		static int lastWidth = -1;
		static int lastHeight = -1;
		if ((width == lastWidth) && (height == lastHeight)) return;
		lastWidth = width;
		lastHeight = height;
		
		if (imageRGB24) delete[] imageRGB24;
		if (imageRGB32) delete[] imageRGB32;
		imageRGB24 = new unsigned char[width*height*3];
		imageRGB32 = new unsigned char[width*height*4];
	}

	// Converts RGB -> BGR0
	void calculateRGB32() {
		unsigned char* end = imageRGB24 + (camera->GetWidth()*camera->GetHeight()*3);
		for (unsigned char *src=imageRGB24, *dst=imageRGB32; src<end; src += 3, dst += 4) {
			dst[0] = src[2];
			dst[1] = src[1];
			dst[2] = src[0];
			dst[3] = 0;
		}
	}
	
private:
	PlayerCc::PlayerClient *robot;
	PlayerCc::CameraProxy *camera;
};

// -------------------------------------------------------------------------------------------

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0) : QMainWindow(parent) {
		RenderArea *canvas = new RenderArea();
		
		QFrame *content = new QFrame();
		QGridLayout *layout = new QGridLayout();
		content->setLayout(layout);
		layout->addWidget(canvas,0,0);
		
		this->setCentralWidget(content);
		this->setWindowTitle("Main Window");

		// Arrange for update() to be called periodically on the RenderArea object
		updateScheduler = new QTimer(this);
		connect(updateScheduler, SIGNAL(timeout()), canvas, SLOT(update()));
		updateScheduler->start(34);
	}
	
	~MainWindow() {
		updateScheduler->stop();
		
		delete updateScheduler;
	}

private:
	QTimer *updateScheduler;
};

// -------------------------------------------------------------------------------------------
