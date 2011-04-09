//g++ lidartestgui.cc -g3 -O0 -lcv -lhighgui -lNAV200

#include <iostream>
#include <limits>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>

#include "NAV200.hpp"
#include "lidarProc.hpp"

static const std::string winname = "lidartest";
static const std::string trackname = "lidartestbar";
boost::mutex winMutex;

const static int imw = 625;
const static int imh = 625;
cv::Mat rawlidar(imh, imw, CV_8UC(1));
boost::mutex rawImageMutex;
cv::Mat runavglidar(imh, imw, CV_8UC(1));
boost::mutex runavgMutex;
cv::Mat rmisoLidar(imh, imw, CV_8UC(1));
boost::mutex rmisoLidarMutex;
cv::Mat morphLidar(imh, imw, CV_8UC(1));
boost::mutex morphLidarMutex;
cv::Mat linesLidar(imh, imw, CV_8UC(1));
boost::mutex linesLidarMutex;

float minrange = 0, maxrange = 0;
const int numpos = 4;

void drawimage(int pos)
{
	boost::mutex::scoped_lock lock(winMutex);
	switch(pos)
	{
		case 0:
		{
				boost::mutex::scoped_lock lock(rawImageMutex);
				cv::imshow(winname, rawlidar);
				break;
		}
		case 1:
		{
				boost::mutex::scoped_lock lock(runavgMutex);
				cv::imshow(winname, runavglidar);
				break;
		}
		case 2:
		{
				boost::mutex::scoped_lock lock(rmisoLidarMutex);
				cv::imshow(winname, rmisoLidar);
				break;
		}
		case 3:
		{
				boost::mutex::scoped_lock lock(morphLidarMutex);
				cv::imshow(winname, morphLidar);
				break;
		}
		case 4:
		{
				boost::mutex::scoped_lock lock(linesLidarMutex);
				cv::imshow(winname, linesLidar);
				break;
		}
		default:
		{
				break;
		}
	}
}

void trackbarcallback(int pos, void* data)
{
	drawimage(pos);
}

void updategui()
{
	int pos = cv::getTrackbarPos(trackname, winname);
	drawimage(pos);
}

//http://arduino.cc/en/Reference/Map
template<typename T>
T linmap(const T& x, const T& in_min, const T& in_max, const T& out_min, const T& out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void plotNAV200Pts(NAV200::Point pts[NAV200::Num_Points], cv::Mat& im, size_t len)
{
	im.setTo(0);

/*
	maxrange = -1*std::numeric_limits<float>::max();
	for(size_t i = 0; i < len; i++)
	{
		if(pts[i].valid)
		{
			maxrange = std::max(maxrange, pts[i].distance);
		}
	}
	minrange = -maxrange;
*/

	maxrange = 5;
	minrange = -maxrange;

	cv::Size sz = im.size();

	for(size_t i = 0; i < len; i++)
	{
		if(pts[i].valid)
		{
			float x, y;
			NAV200::polar2cart(pts[i], x, y);

			int xpx = linmap<float>(x, minrange, maxrange, 0, sz.width);
			int ypx = sz.height - linmap<float>(y, minrange, maxrange, 0, sz.height);

			//std::cout << "min: " << minrange << "max: " << maxrange <<  " theta: " << pts[i].angle << " distance: " << pts[i].distance << " x: " << x << " y: " << y << " xpx: " << xpx << " ypx: " << ypx <<std::endl;
			im.at<uint8_t>(ypx, xpx) = 255;
		}
	}

}

void plotNAV200cart(float* x, float* y, cv::Mat& im, size_t len)
{
	im.setTo(0);

/*
	maxrange = -1*std::numeric_limits<float>::max();
	for(size_t i = 0; i < len; i++)
	{
		float distance = sqrt(x[i]*x[i] + y[i]*y[i]);
		maxrange = std::max(maxrange, distance);
	}
	minrange = -maxrange;
*/

	maxrange = 5;
	minrange = -maxrange;
	cv::Size sz = im.size();

	for(size_t i = 0; i < len; i++)
	{
		int xpx = linmap<float>(x[i], minrange, maxrange, 0, sz.width);
		int ypx = sz.height - linmap<float>(y[i], minrange, maxrange, 0, sz.height);

		im.at<uint8_t>(ypx, xpx) = 255;
	}

}

void plotline200cart(float* x, float* y, cv::Mat& im, float startx, float starty, float endx, float endy, size_t len)
{
/*
	maxrange = -1*std::numeric_limits<float>::max();
	for(size_t i = 0; i < len; i++)
	{
		float distance = sqrt(x[i]*x[i] + y[i]*y[i]);
		maxrange = std::max(maxrange, distance);
	}
	minrange = -maxrange;
*/
	maxrange = 5;
	minrange = -maxrange;

	cv::Size sz = im.size();


	int startxpx = linmap<float>(startx, minrange, maxrange, 0, sz.width);
	int startypx = sz.height - linmap<float>(starty, minrange, maxrange, 0, sz.height);

	int endxpx = linmap<float>(endx, minrange, maxrange, 0, sz.width);
	int endypx = sz.height - linmap<float>(endy, minrange, maxrange, 0, sz.height);

	cv::Point a(startxpx, startypx);
	cv::Point b(endxpx, endypx);
	cv::line(im, a, b, cv::Scalar(255,255,255));
}

void array2NAV200(const float* x, const float* y, NAV200::Point* points, size_t len)
{
	for(size_t i = 0; i < len; i++)
	{
		points[i].angle = x[i];
		points[i].distance = y[i];
		points[i].valid = true;
	}
}

int main()
{
	//setup gui
	//cv::namedWindow(winname, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(winname);
	cv::createTrackbar(trackname, winname, NULL, numpos, trackbarcallback, NULL);

	NAV200 lidar;

	for(;;)
	{
		if(lidar.read())
		{
			{
			boost::mutex::scoped_lock lock(linesLidarMutex);
			linesLidar.setTo(0);
			}

			// Draw Raw points
			boost::mutex::scoped_lock lock(rawImageMutex);
			plotNAV200Pts(lidar.points, rawlidar, NAV200::Num_Points);

			// Get the valid raw points in polar
			float goodt[NAV200::Num_Points];
			float goodr[NAV200::Num_Points];
			size_t numgoodpts = lidar.getValidData(goodt,goodr);

			// Get the valid raw points in cartesian
			float goodx[NAV200::Num_Points];
			float goody[NAV200::Num_Points];
			NAV200::polar2cart(goodt, goodr, goodx, goody, numgoodpts);

			//running average the radius
			NAV200::Point avgpoints[NAV200::Num_Points];
			float ravg[NAV200::Num_Points];
			lidarProc::runavg(goodr, ravg, numgoodpts, 10);

			// plot the running average
			array2NAV200(goodt, ravg, avgpoints, numgoodpts);
			plotNAV200Pts(avgpoints, runavglidar, numgoodpts);

			// get the polar slope
			float derivR[NAV200::Num_Points];
			float derivT[NAV200::Num_Points];
			lidarProc::takeDerivative(goodr, goodt, derivR, derivT, numgoodpts);

			// get the polar accel
			float ddR[NAV200::Num_Points];
			float ddT[NAV200::Num_Points];
			lidarProc::takeDerivative(derivR, derivT, ddR, ddT, numgoodpts);

			//remove points w/ large slope
			float x_avg[NAV200::Num_Points];
			float y_avg[NAV200::Num_Points];
			float x_dense[NAV200::Num_Points];
			float y_dense[NAV200::Num_Points];
			NAV200::polar2cart(goodt, ravg, x_avg, y_avg, numgoodpts);
	
			size_t denselen = 0;
			lidarProc::removeIsolatedPoints(x_avg, y_avg, numgoodpts, x_dense, y_dense, denselen, .1);
			plotNAV200cart(x_dense, y_dense, rmisoLidar, denselen);

			//morphological operations
			//runavglidar.copyTo(morphLidar);			
			rmisoLidar.copyTo(morphLidar);
			cv::morphologyEx(morphLidar, morphLidar, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), 10);
			cv::dilate(morphLidar, morphLidar, cv::Mat());

			#if 1
			//find lines with polar
			std::deque< boost::tuple<size_t,size_t> > lines;
			lidarProc::findLinearRuns(goodt, goodr, numgoodpts, 5, lines);

			std::deque< boost::tuple<size_t,size_t> > grow_lines;
			lidarProc::collectNearRuns(goodt, goodr, numgoodpts, .02, lines, grow_lines);

			boost::tuple<size_t,size_t> pt;
			BOOST_FOREACH(pt, grow_lines)
			{
				float startt = goodt[pt.get<0>()];
				float startr = goodr[pt.get<0>()];

				float stopt = goodt[pt.get<1>()];
				float stopr = goodr[pt.get<1>()];

				//float stopt = goodt[pt.get<1>()];
				//float stopr = ravg[pt.get<1>()];

				float startx, starty, stopx, stopy;
				NAV200::polar2cart(startt, startr, startx, starty);
				NAV200::polar2cart(stopt, stopr, stopx, stopy);

				std::cout << "line at pos: (" << startx << ", " << starty << "), (" << stopx << ", " << stopy << ")\n";

				boost::mutex::scoped_lock lock(linesLidarMutex);
				plotline200cart(goodx, goody, linesLidar, startx, starty, stopx, stopy, numgoodpts);
			}
			#else
			std::deque< boost::tuple<size_t,size_t> > lines;
			lidarProc::findLinearRuns(x_dense, y_dense, denselen, 7, lines);

			boost::tuple<size_t,size_t> pt;
			BOOST_FOREACH(pt, lines)
			{
				float startx = x_dense[pt.get<0>()];
				float starty = y_dense[pt.get<0>()];

				float stopx = x_dense[pt.get<1>()];
				float stopy = y_dense[pt.get<1>()];

				std::cout << "line at pos: (" << startx << ", " << starty << "), (" << stopx << ", " << stopy << ")\n";

				boost::mutex::scoped_lock lock(linesLidarMutex);
				plotline200cart(x_dense, y_dense, linesLidar, startx, starty, stopx, stopy, numgoodpts);
			}
			#endif
		}
		updategui();

		cv::waitKey(125);
	}
}
