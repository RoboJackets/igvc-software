#ifndef _VIDEO_VIEW_H_
#define _VIDEO_VIEW_H_

// To make a view that will show up on the list in the GUI (in a VideoWidget),
// make a subclass of VideoView.  Implement at least getName() and draw().
// Statically declare an object of the subclass.
// The VideoView constructor will initialize the videoViews list and
// add each object to it.
//
// The GUI code will automatically populate the view type list, so other code
// needs to be changed to add a view type.  Ain't that cool?
// 
// SEE ALSO: gui/views

#include <QPainter>
#include <QList>
#include <QImage>

#include "vision/Buffer2D.h"
#include "vision/Pixel.h"

// The base class for all views
class VideoView
{
public:
	VideoView();
	virtual ~VideoView();

	virtual QString getName() const = 0;

	virtual void draw(QPainter &p);
	virtual void save(const QString &filename);

	static const QList<VideoView *> *getViewList() { return viewList; }

protected:
	// Override this to return the image to be drawn/saved.
	virtual QImage createImage();

	// Called after the image is drawn.  Subclasses may use this to release
	// data used to generate the image.
	virtual void freeImage();

private:
	// This list must be created at run-time because we cannot guarantee that
	// its constructor would be called before the constructors of all the VideoView
	// subclass objects (which add themselves to this list).
	static QList<VideoView *> *viewList;
};

////////

// Abstract base class for views that show an 8 bits-per-pixel (bpp) grayscale image.
class GrayscaleView: public VideoView
{
public:
	GrayscaleView();
	
protected:
	QImage createImage();
	
	// This must set the "image", "width", and "height" fields of this class
	virtual void prepareToCreateImage() = 0;

	// Subclasses must initialize these fields when
	// prepareToCreateImage() is invoked
	unsigned char *image;
	int width, height;

private:
#if 0
	// Color table for use with QImage.  Created the first time
	// this class' constructor is called.
	// Entry i is color (i, i, i).
	//static QRgb *colortable;
	static QVector<QRgb>* colortable;
#endif
};

////////

// Abstract base class for views that show an 8 bits-per-pixel (bpp) black & white image,
// where 0=black and <non-zero>=white.
class BlackAndWhiteView: public VideoView
{
public:
	BlackAndWhiteView();
	
protected:
	QImage createImage();
	
	// This must set the "image", "width", and "height" fields of this class
	virtual void prepareToCreateImage() = 0;

	// Subclasses must initialize these fields when
	// prepareToCreateImage() is invoked
	bool *image;
	int width, height;

private:
#if 0
	// Color table for use with QImage.  Created the first time
	// this class' constructor is called.
	//static QRgb *colortable;
	static QVector<QRgb>* colortable;
#endif
};

////////

// Generic class for color images coming from the vision code.
// 
// This view locks visMutex while the image is being drawn.
class VisColorView: public VideoView
{
private:
	QString name;
	Buffer2D<Pixel>* visBuffer;

public:
	VisColorView(QString name, Buffer2D<Pixel>* visBuffer) {
		this->name = name;
		this->visBuffer = visBuffer;
	}
	
public:
	QString getName() const {
		return name;
	}

protected:
	QImage createImage();
	void freeImage();
};

////////

// Generic class for grayscale images coming from the vision code.
// 
// This view locks visMutex while the image is being drawn.
class VisGrayView: public GrayscaleView
{
private:
	QString name;
	Buffer2D<unsigned char>* visBuffer;

public:
	VisGrayView(QString name, Buffer2D<unsigned char>* visBuffer) {
		this->name = name;
		this->visBuffer = visBuffer;
	}
	
public:
	QString getName() const {
		return name;
	}

protected:
	void prepareToCreateImage();
	void freeImage();
};

////////

// Generic class for black & white images coming from the vision code.
// 
// This view locks visMutex while the image is being drawn.
class VisBlackAndWhiteView: public BlackAndWhiteView
{
private:
	QString name;
	Buffer2D<bool>* visBuffer;

public:
	VisBlackAndWhiteView(QString name, Buffer2D<bool>* visBuffer) {
		this->name = name;
		this->visBuffer = visBuffer;
	}
	
public:
	QString getName() const {
		return name;
	}

protected:
	void prepareToCreateImage();
	void freeImage();
};

#endif // _VIDEO_VIEW_H_

