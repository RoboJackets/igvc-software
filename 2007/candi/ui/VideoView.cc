#include "VideoView.h"
#include "vision/vision.h"	// for visMutex

QList<VideoView *> *VideoView::viewList = 0;

VideoView::VideoView()
{
	if (!viewList)
	{
		viewList = new QList<VideoView *>;
	}

	viewList->append(this);
}

VideoView::~VideoView()
{
	viewList->removeAll(this);
}

void VideoView::draw(QPainter &p)
{
	QImage image = createImage();
	{
		// Fill background with solid color
		p.fillRect(p.window(), Qt::darkGray);	// Qt::black
		
		// Draw the image
		p.drawImage(0, 0, image);
	}
	freeImage();
}

QImage VideoView::createImage()
{
	return QImage();
}

void VideoView::freeImage()
{
}

void VideoView::save(const QString &filename)
{
	QImage image = createImage();
	image.save(filename, "PNG");
}

////////

#if 0
//QRgb *GrayscaleView::colortable = NULL;
QVector<QRgb>* GrayscaleView::colortable = NULL;
#endif

GrayscaleView::GrayscaleView()
{
#if 0
	if (!colortable)
	{
		/*
		colortable = new QRgb[256];
		for (int i = 0; i < 256; i++)
			colortable[i] = qRgb(i, i, i);
		*/
		colortable = new QVector<QRgb>(256);
		for (int i = 0; i < 256; i++)
			colortable->append(qRgb(i, i, i));
	}
#endif

	image = 0;
	width = 0;
	height = 0;
}

QImage GrayscaleView::createImage()
{
	// Acquire values for the "image", "width", and "height" fields
	this->prepareToCreateImage();
	
	if (image && width && height) {
		if ((long(image) % 4) != 0) {
			printf("GrayscaleView: image is not 4-byte aligned\n");
		}
		if ((width % 4) != 0) {
			printf("GrayscaleView: width of %d is not divisible by 4!\n", (int) width);
		}
		
		QImage im = QImage(
			(uchar*) image,
			width, height,
			//8, colortable, 256, QImage::LittleEndian);
			QImage::Format_Indexed8);
		
		// Set the image's color table
		im.setNumColors(256);
		for (int i=0; i<256; i++)
			im.setColor(i, qRgb(i, i, i));
		
		return im;
	} else {
		return QImage();
	}
}

////////

#if 0
//QRgb *BlackAndWhiteView::colortable = NULL;
QVector<QRgb>* BlackAndWhiteView::colortable = NULL;
#endif

BlackAndWhiteView::BlackAndWhiteView()
{
#if 0
	if (!colortable)
	{
		// NOTE: assumes that sizeof(bool) == sizeof(unsigned char) == 1
		/*
		colortable = new QRgb[256];
		colortable[0] = qRgb(0, 0, 0);			// 0          = black
		for (int i = 1; i < 256; i++)
			colortable[i] = qRgb(255, 255, 255);	// <non-zero> = white
		*/
		colortable = new QVector<QRgb>(256);
		colortable->append(qRgb(0, 0, 0));				// 0          = black
		for (int i = 1; i < 256; i++)
			colortable->append(qRgb(255, 255, 255));	// <non-zero> = white
	}
#endif

	image = 0;
	width = 0;
	height = 0;
}

QImage BlackAndWhiteView::createImage()
{
	// Acquire values for the "image", "width", and "height" fields
	this->prepareToCreateImage();
	
	if (image && width && height) {
		// NOTE: Assumes that sizeof(bool) == sizeof(uchar))
		if (sizeof(bool) != sizeof(uchar)) {
			printf("BlackAndWhiteView: sizeof(bool) != sizeof(uchar)!\n");
			return QImage();
		}
		
		if ((long(image) % 4) != 0) {
			printf("BlackAndWhiteView: image is not 4-byte aligned\n");
		}
		if ((width % 4) != 0) {
			printf("BlackAndWhiteView: width of %d is not divisible by 4!\n", (int) width);
		}
		
		QImage im = QImage(
			(uchar*) image,
			width, height,
			//8, colortable, 256, QImage::LittleEndian
			QImage::Format_Indexed8);
		
		// Set the image's color table
		im.setNumColors(256);
		im.setColor(0, qRgb(0, 0, 0));					// 0          = black
		for (int i=1; i<256; i++)
			im.setColor(i, qRgb(255, 255, 255));		// <non-zero> = white
		
		return im;
	} else {
		return QImage();
	}
}

////////

QImage VisColorView::createImage() {
	visMutexLock();
	return QImage(
		(unsigned char*) visBuffer->data,
		visBuffer->width, visBuffer->height,
		//32, 0, 0, QImage::LittleEndian);
		QImage::Format_RGB32);	// BGR0
}

void VisColorView::freeImage() {
	visMutexUnlock();
}

////////

void VisGrayView::prepareToCreateImage()
{
	visMutexLock();
	if (visBuffer) {
		this->image = visBuffer->data;
		this->width = visBuffer->width;
		this->height = visBuffer->height;
	} else {
		this->image = NULL;
		this->width = 0;
		this->height = 0;
	}
}

void VisGrayView::freeImage()
{
	visMutexUnlock();
}

////////

void VisBlackAndWhiteView::prepareToCreateImage()
{
	visMutexLock();
	if (visBuffer) {
		this->image = visBuffer->data;
		this->width = visBuffer->width;
		this->height = visBuffer->height;
	} else {
		this->image = NULL;
		this->width = 0;
		this->height = 0;
	}
}

void VisBlackAndWhiteView::freeImage()
{
	visMutexUnlock();
}
