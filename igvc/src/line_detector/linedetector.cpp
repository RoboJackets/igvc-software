#include "linedetector.h"
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <queue>

using namespace std;
using namespace cv;
using namespace pcl;

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

constexpr double radians(double degrees)
{
    return degrees / 180.0 * M_PI;
}

void LineDetector::img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cerr << "CALLBACK CALLED" << endl;
    cv_ptr = cv_bridge::toCvCopy(msg, "");

	// What separates lines from other objects?
    // 1. Lines are whiter than their surroundings
    // 2. Lines are on the ground
    // 3. A section of a line has two other line sections adjascent to it
    // 4. We know roughly how wide a line is going to be

    const int linewidthpixels = 13;

    Mat grnd = (cv_ptr->image).clone();
    Mat dview_dst = (cv_ptr->image).clone();
    Mat squish_dst = (cv_ptr->image).clone();

// Squish
    resize(grnd, squish_dst, Size(3*grnd.cols/linewidthpixels, 3*grnd.rows/linewidthpixels), 0, 0, INTER_LANCZOS4);


    // TODO: move these constant definitions outside of the callback.
    float karray[3][9][9] = {
            {
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0}
            }, {
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    {-1, -1, -1, -1, -1, -1, -1,  0,  1},
                    {-1, -1, -1, -1, -1,  0,  1,  1,  1},
                    {-1, -1, -1,  0,  1,  1,  1,  1,  1},
                    {-1,  0,  1,  1,  1,  1,  1, .5,  0},
                    { 1,  1,  1,  1,  1, .5,  0,  0,  0},
                    { 1,  1,  1, .5,  0,  0,  0,  0,  0},
                    { 1, .5,  0,  0,  0,  0,  0,  0,  0},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0}
            },  {
                    {-.89,-.89,-.89,-.89,-.89,-.89,-.89,   1,   1},
                    {-.89,-.89,-.89,-.89,-.89,-.89,   1,   1,   1},
                    {-.89,-.89,-.89,-.89,-.89,   1,   1,   1,   0},
                    {-.89,-.89,-.89,-.89,   1,   1,   1,   0,   0},
                    {-.89,-.89,-.89,   1,   1,   1,   0,   0,   0},
                    {-.89,-.89,   1,   1,   1,   0,   0,   0,   0},
                    {-.89,   1,   1,   1,   0,   0,   0,   0,   0},
                    {   1,   1,   1,   0,   0,   0,   0,   0,   0},
                    {   1,   1,   0,   0,   0,   0,   0,   0,   0}
            }
    };

    // TODO: Should also move these constant operations outside of the callback
    vector<Mat> kernals;
    Mat kernal1(9, 9, CV_32FC1, karray[0]);
    kernal1 /= 27;
    Mat kernal2(9, 9, CV_32FC1, karray[1]);
    kernal2 /= 25;
    Mat kernal3(9, 9, CV_32FC1, karray[2]);
    kernal3 /= 25;

    /*
     * Why transpose kernel 2?
     */
    Mat kernal4 = kernal2.t();
    Mat kernal5 = kernal1.t();

    Mat kernal6;
    Mat kernal7;
    Mat kernal8;

    flip(kernal4, kernal6, 0);
    flip(kernal3, kernal7, 0);
    flip(kernal2, kernal8, 0);

    kernals.push_back(kernal1);
    kernals.push_back(kernal2);
    kernals.push_back(kernal3);
    kernals.push_back(kernal4);
    kernals.push_back(kernal5);
    kernals.push_back(kernal6);
    kernals.push_back(kernal7);
    kernals.push_back(kernal8);

    Mat testimage = squish_dst;

    vector<Mat> results = getGeometricMean(testimage, kernals);

    subtractOrthog(results);
    Mat fin_img = results[0].clone();

    RemoveNonMax(results);

    int threshval = 3;
    for(size_t i = 0; i < results.size(); i++) {
        Mat bgr[3];
        split(results[i], bgr);
        threshold(bgr[0], results[i], threshval, 255, CV_THRESH_BINARY);
        cvtColor(results[i], results[i], CV_GRAY2BGR);
    }

    EnforceContinuity(results, fin_img);

    resize(fin_img, fin_img, Size(dview_dst.cols, dview_dst.rows), 0, 0, INTER_LANCZOS4);
    cloud = toPointCloud(fin_img);
    _line_cloud.publish(cloud);
}

void LineDetector::RemoveNonMax(vector<Mat>& images) {
    Mat maxRes = images[0].clone();
    for(Mat& r : images) {
        maxRes = max(maxRes, r);
    }
    for(Mat& r : images) {
        r = r - (r != maxRes); // FICME: What? Image = Image - bool?
    }
}

typedef struct Node {
    shared_ptr<struct Node> prev;
    Point point;
    int length;

    Node(shared_ptr<struct Node> pr, Point po, int l)
    {
        prev = pr;
        point = po;
        length = l;
    }
} Node;

void LineDetector::EnforceContinuity(vector<Mat>& directions, Mat& out) {
    cerr << "Enforcing Continuity" << endl;
    unsigned int iterations = 300;
    int minpathlength = 20;
    Mat gradient = Mat::zeros(directions[0].rows, directions[0].cols, CV_8UC1);
    Mat fin_img = Mat::zeros(directions[0].rows, directions[0].cols, CV_8UC3);
    for(size_t i = 0; i < directions.size(); i++) {
        Mat bgr[3];
        split(directions[i], bgr);
        bitwise_and(bgr[0], (0b00000001 << i), bgr[0]);
        bitwise_or(gradient, bgr[0], gradient);
    }

    auto getneighbors = [](shared_ptr<Node> n, Mat& gradient) {
        auto similarbits = [](unsigned char a, unsigned char b) {
            return ((a == b) | (a == b << 1) | (a == b >> 1) | (129 == (a | b)));
        };
        vector <shared_ptr<Node> > neighbors;
        if(!n) return neighbors;
        Point p = n->point;
        const unsigned char g = gradient.at<uchar>(p);
        bool forward = true;
        int px = n->prev ? (n->point.x > n->prev->point.x ? 1 : -1) : n->length;
        int py = n->prev ? (n->point.y > n->prev->point.y ? 1 : -1) : n->length;

        if (g & 0b00010000) {           // forward -> y new < y old
            if (py > 0) forward = false;
            px = 0;
            py = -2;
        } else if (g & 0b00000001) {    // forward -> x new > x old
            if(px < 0) forward = false;
            px = 2;
            py = 0;
        } else if (g & 0b00000010) {    // forward -> x new > x old
            if(px < 0) forward = false;
            px = 2;
            py = -1;
        } else if (g & 0b00000100) {    // forward -> x new > x old
            if(px < 0 || py > 0) forward = false;
            px = 2;
            py = -2;
        } else if (g & 0b00001000) {    // forward -> y new < y old
            if(py > 0) forward = false;
            px = 1;
            py = -2;
        } else if (g & 0b00100000) {    // forward -> y new < y old
            if(py > 0) forward = false;
            px = -1;
            py = -2;
        } else if (g & 0b01000000) {    // forward -> y new < y old
            if(px > 0 || py > 0) forward = false;
            px = -2;                   //            x new < x old
            py = -2;
        } else if (g & 0b10000000) {    // forward -> x new < x old
            if(px > 0) forward = false;
            px = -2;
            py = -1;
        }
        if(!forward) {
            px *= -1;
            py *= -1;
        }
        if (g) {
            for (int dx = -1; dx < 2; dx++) {
                for (int dy = -1; dy < 2; dy++) {
                    Point newp(p.x + px + dx, p.y + py + dy);
                    if (newp.x < gradient.cols &&
                        newp.x >= 0 &&
                        newp.y < gradient.rows &&
                        newp.y >= 0)
                        if (similarbits(gradient.at<uchar>(newp), g)) {
                            neighbors.push_back(make_shared<Node>(n, newp, (n->length > 0) ? (n->length + 1) : (n->length - 1)));
                        }

                }
            }
        }
        gradient.at<uchar>(p) = 0;

        return neighbors;
    };

    Mat randlineinds;

    findNonZero(gradient, randlineinds);

    randShuffle(randlineinds);

    for(size_t i = 0; i < iterations && i < randlineinds.total(); i++) {

        Mat gradientcopy = gradient.clone();
        vector<shared_ptr<Node> > frontier;
        shared_ptr<Node> start = make_shared<Node>(nullptr, randlineinds.at<Point>(i), 1);
        shared_ptr<Node> trats = make_shared<Node>(nullptr, randlineinds.at<Point>(i), -1);
        frontier.push_back(trats);
        frontier.push_back(start);

        shared_ptr<Node> current = start;
        shared_ptr<Node> bestpathforward = start;
        shared_ptr<Node> bestpathbackward = trats;

        while(!frontier.empty()) {
            current = frontier.back();
            frontier.pop_back();

            if (current->length > 0) {
                if (current->length > bestpathforward->length)
                    bestpathforward = current;
            } else {
                if (current->length < bestpathbackward->length)
                    bestpathbackward = current;
            }

            vector<shared_ptr<Node>> neighbors = getneighbors(current, gradientcopy);
            if(neighbors.empty()) continue;
            for(shared_ptr<Node>& n : neighbors)
                frontier.push_back(n);
        }

        if(bestpathforward->length - bestpathbackward->length > minpathlength) {

            current = bestpathforward;
            while (current) {
                fin_img.at<Vec3b>(current->point) = Vec3b(255, 255, 255);
                current = current->prev;
            }
            current = bestpathbackward;
            while (current) {
                fin_img.at<Vec3b>(current->point) = Vec3b(255, 255, 255);
                current = current->prev;
            }
        }
        cv_ptr->image = fin_img;
        cerr << "Publishing to filt_img" << endl;
        _filt_img.publish(cv_ptr->toImageMsg());
        Mat channels[3];
        split(fin_img, channels);
        out = channels[0];
    }
}

vector<Mat> LineDetector::getGeometricMean(Mat& image, vector<Mat> &kernals) {
    Mat filtered1, filtered2;
    vector<Mat> results;

    for(size_t i = 0; i < kernals.size(); i++) {
        Mat kernal2;
        flip(kernals[i], kernal2, -1);

        filtered1 = applyFilter(image, kernals[i]);
        filtered2 = applyFilter(image, kernal2);


        filtered1.convertTo(filtered1, CV_16UC3, 1);
        filtered2.convertTo(filtered2, CV_16UC3, 1);

        results.push_back(filtered1.mul(filtered2));
        results[i].convertTo(results[i], CV_8UC3, 1.0 / 256);
    }

    return results;
};

Mat LineDetector::applyFilter(Mat &image, const Mat &kernal) {
    Mat fin_img;
    vector<Mat> newchannel(3);
    vector<Mat> oldchannel(3);
    split(image, oldchannel);
    for(int i = 0; i < 3; i++) {
        filter2D(oldchannel[i], newchannel[i], -1, kernal);
    }

    merge(newchannel, fin_img);
    return fin_img;
}

bool operator<(const Vec3f& s1, const Vec3f& s2) {
    return s1[0] < s2[0];
}

void LineDetector::subtractOrthog(vector<Mat>& images) {
    for (int r = 0; r < images[0].rows; r++) {
        for(int c = 0; c < images[0].cols; c++) {
            Vec3b blackpixel(0, 0, 0);
            Vec3b maxdiff(0, 0, 0);
            Vec3b smax(0, 0, 0);
            for(size_t i = 0; i < images.size(); i++){
                Vec3b s1 = images[i].at<Vec3b>(r, c);
                Vec3b s2 = images[(i+images.size()/2) % images.size()].at<Vec3b>(r, c);
                images[i].at<Vec3b>(r, c) = s1 - s2;
            }
        }
    }
}

LineDetector::LineDetector(ros::NodeHandle &handle)
    : max_elem(2),
      max_kernel_size(2),
      gaussian_size(7),
      _it(handle),
      tf_listener(handle)
{
    _src_img = _it.subscribe("/left/image_rect_color", 1, &LineDetector::img_callback, this);
	  _filt_img = _it.advertise("/filt_img", 1);
    _line_cloud = handle.advertise<PCLCloud>("/line_cloud", 100);

}

PointCloud<PointXYZ>::Ptr LineDetector::toPointCloud(Mat src){
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    tf::StampedTransform transform;
    tf_listener.lookupTransform("/base_footprint", "/camera", ros::Time(0), transform);
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    auto origin_z = transform.getOrigin().getZ();
    auto origin_y = transform.getOrigin().getY();
    auto HFOV = radians(66.0);
    auto VFOV = radians(47.6);
    pitch = -roll; // Because conventions are different and I'm in the middle of comp, and give me a break.
    for(int r = src.rows/2; r < src.rows; r++)
    {
        uchar *row = src.ptr<uchar>(r);
        for(int c = 0; c < src.cols; c++)
        {
            if(row[c] > 0)
            {
                auto pitch_offset = ((float)(r-src.rows/2) / src.rows) * VFOV;
                auto y = origin_z /tan(pitch + pitch_offset) + origin_y;

                auto theta = ((float)(c-src.cols/2) / src.cols) * HFOV;
                auto x = y * tan(theta);

                cloud->points.push_back(PointXYZ(x, y, 0));
            }
        }
    }
	cloud->header.frame_id = "base_footprint";
	return cloud;
}

void LineDetector::helperSearch(void *totvis, void *input, int i, int j, cv::Mat *imgmat, int thresh) {
    vector< tuple<int, int> > *totalvisited = (vector< tuple<int, int> >*)totvis;
    vector< tuple<int, int> > *visited = (vector< tuple<int, int> >*)input;
    if (!helperContains(tuple<int, int>(i, j), visited) && !helperContains(tuple<int, int>(i, j), totalvisited)) {
		visited->push_back( tuple<int, int>(i, j) );
		totalvisited->push_back( tuple<int, int>(i, j) );
	} else {
		return;
	}
	if (visited->size() > 500) {
		return;
	}
    for (int k=0; k<5; k+=4) {
        if (i-2+k < 0 || i-2+k >= imgmat->rows || j < 0 || j >= imgmat->cols) {
            continue;
        }
        if (adjWhite(i-2+k, j, imgmat, thresh)) {
            helperSearch(totalvisited, visited, i-2+k, j, imgmat, thresh);
        }
	}
	for (int k=0; k<3; k+=2) {
        if (i-2+k < 0 || i-2+k >= imgmat->rows || j+2 < 0 || j+2 >= imgmat->cols) {
            continue;
        }
        if (adjWhite(i-2+k, j+2, imgmat, thresh)) {
            helperSearch(totalvisited, visited, i-2+k, j+2, imgmat, thresh);
        }
	}
}

bool LineDetector::adjWhite(int i, int j, cv::Mat *img, int thresh) {
	int grey;
	int scal = 1;
	for (int k = -1; k < 2; k++) {
		int n = i+(k*scal);
		int m = j;
        if (n < 0 || n >= img->rows || m < 0 || m >= img->cols) {
            continue;
        }
        grey = img->at<Vec3b>(n, m)[0];
        if (grey >= thresh) {
            return true;
        }
	}
	for (int k = -1; k < 2; k++) {
		int n = i+(k*scal);
		int m = j+scal;
        if (n < 0 || n >= img->rows || m < 0 || m >= img->cols) {
            continue;
        }
        grey = img->at<Vec3b>(n, m)[0];
        if (grey >= thresh) {
            return true;
        }
	}
	for (int k = -1; k < 2; k++) {
		int n = i+(k*scal);
		int m = j-scal;
        if (n < 0 || n >= img->rows || m < 0 || m >= img->cols) {
            continue;
        }
        grey = img->at<Vec3b>(n, m)[0];
        if (grey >= thresh) {
            return true;
        }
	}
	return false;
}

bool LineDetector::helperContains(tuple<int, int> pos, void *vis) {
    vector< tuple<int, int> > *visited = (vector< tuple<int, int> >*)vis;
    for (size_t i=0; i<visited->size(); i++) {
        if (visited->at(i) == pos) {
            return true;
        }
	}
    return false;
}
