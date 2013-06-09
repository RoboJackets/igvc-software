#include <pcl/kdtree/kdtree_flann.h>
#include "CompetitionController.h"

using namespace IGVC::Sensors;
using namespace std;

namespace IGVC
{
namespace Control
{

CompetitionController::CompetitionController(IGVC::Sensors::GPS* gps,
                                            Event<pcl::PointCloud<pcl::PointXYZ> >* mapSource,
                                            WaypointReader* waypointReader,
                                            MotorDriver* driver)
    : _hasAllData(false),
      GPS_BUFFER_SIZE(5),
      LOnNewGPSData(this),
      LOnNewIMUData(this),
      LOnNewMapFrame(this),
      _viewer("Map and Path")
{
    _gps = gps;
    _gps->onNewData += &LOnNewGPSData;
    _waypointReader = waypointReader;
    _waypointReader->Next();
    _currentHeading = 0;
    _driver = driver;

    (*mapSource) += &LOnNewMapFrame;

    MaxW = 0.8;
    DeltaT = 1.5
}

bool CompetitionController::isRunning()
{
    return true;
}

void CompetitionController::OnNewGPSData(GPSData data)
{
    std::cout << "Compcon gps" << std::endl;
    if(_gpsBuffer.size() >= GPS_BUFFER_SIZE)
    {
        _gpsBuffer.push_back(data);
        GPSData last = _gpsBuffer.back();
        _currentAvgGPS.Lat(_currentAvgGPS.Lat() - ( last.Lat() / GPS_BUFFER_SIZE ));
        _currentAvgGPS.Long(_currentAvgGPS.Long() - ( last.Long() / GPS_BUFFER_SIZE ));
        _currentAvgGPS.Heading(_currentAvgGPS.Heading() - ( last.Heading() / GPS_BUFFER_SIZE ));
        _currentAvgGPS.Speed(_currentAvgGPS.Speed() - ( last.Speed() / GPS_BUFFER_SIZE ));
        _gpsBuffer.erase(_gpsBuffer.begin());
    }
    else
    {
        _gpsBuffer.push_back(data);
    }
    _currentAvgGPS.Lat(_currentAvgGPS.Lat() + ( data.Lat() / GPS_BUFFER_SIZE ));
    _currentAvgGPS.Long(_currentAvgGPS.Long() + ( data.Long() / GPS_BUFFER_SIZE ));
    _currentAvgGPS.Heading(_currentAvgGPS.Heading() + ( data.Heading() / GPS_BUFFER_SIZE ));
    _currentAvgGPS.Speed(_currentAvgGPS.Speed() + ( data.Speed() / GPS_BUFFER_SIZE ));
    _hasAllData = true;
}

void CompetitionController::OnNewIMUData(IMURawData data)
{
    std::cout << "Compcon imu" << std::endl;
    _currentHeading = data.heading;
}

/*
void CompetitionController::OnNewMapFrame(pcl::PointCloud<pcl::PointXYZ> mapFrame)
{
    cout << mapFrame.points.size() << " points recieved." << endl;
    if(!_hasAllData)
        return;
    _viewer.removeAllPointClouds(0);
    _viewer.removeAllShapes();
    _viewer.addPointCloud(mapFrame.makeShared(), "map");
    _viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "map");
    _viewer.spin();
    std::cout << "Compcon newMap" << std::endl;
    vector< pair<double, double> > available_actions;

    using namespace std;
    cout << "Loading possible actions" << endl;

    double v = 0.4;
    for(double w = -MaxW; w <= MaxW; w += 0.5)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(mapFrame.makeShared());
        pair<double, double> end = result(w,v);
        pcl::PointXYZ searchPoint(end.first, end.second, 0);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if(kdtree.radiusSearch(searchPoint, 0.01, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0)
        {
            available_actions.push_back(pair<double,double>(v, w));
        }
    }

    cout << "Checking possible actions..." << endl;

    pair<double, double> minPair;
    double minDist = -1;

    for(vector< pair<double, double> >::iterator iter = available_actions.begin(); iter != available_actions.end(); iter++)
    {
        pair<double, double> action = (*iter);

        cout << "Action done" << endl;

        pair<double, double> waypointCoords;

        cout << _waypointReader->Current().Lat() << endl;

        waypointCoords.first = GPSdX(_currentAvgGPS, _waypointReader->Current());
        waypointCoords.second = GPSdY(_currentAvgGPS, _waypointReader->Current());

        cout << "loaded" << endl;

        pair<double, double> endLoc = result(action.second, action.first);

        cout << "resulted" << endl;

        double dist = distBetween(waypointCoords, endLoc);

        cout << "dist" << endl;

        if(minDist == -1 || dist < minDist)
        {
            minPair = action;
            minDist = dist;
        }
    }

    cout << "Computing velocities..." << endl;

    double Baseline = Robot::CurrentRobot().Baseline();
    double Vr = minPair.first + (Baseline/2.0)*minPair.second;
    double Vl = minPair.first - (Baseline/2.0)*minPair.second;

    cout << "Decided on Vl=" << Vl << " and Vr=" << Vr << endl;

    _driver->setVelocities(Vl, Vr);

}
*/


double CompetitionController::weighting(double delta)
{
  if (delta !=0 )
  {
    //return 1/pow(delta,2);
    return 1/delta;
  }
  else
  {
    std::cout << "input to weighting function was 0. This should not have happened" << std::endl;
    return 0;
  }
}

void CompetitionController::OnNewMapFrame(pcl::PointCloud<pcl::PointXYZ> mapFrame)
{
    //cout << mapFrame.points.size() << " points recieved." << endl;
    //Bunch of Visualization stuff
    if(!_hasAllData)
        return;
    _viewer.removeAllPointClouds(0);
    _viewer.removeAllShapes();
    _viewer.addPointCloud(mapFrame.makeShared(), "map");
    _viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "map");
    _viewer.spin();
    std::cout << "Compcon newMap" << std::endl;
    vector< pair<double, double> > available_actions;

    //cout << "Loading possible actions" << endl;

    //Find Potential Routes
    double v = 0.4; //meters per second

    double wInc = 0.05; //step size between possible omegas
    int i=0; //servers as an iterator for scores so the desired index doesnt have to be rederied from w
    double deltaDeltaT=0.1;

    const int nPaths = floor((2*MaxW)/wInc);
    std::vector<double> scores;
    scores.assign(nPaths,0);


    double minDeltaT = Robot::CurrentRobot().Dist2Front()/v;

    for(double w = -MaxW; w <= MaxW; w += wInc)
    {

      for(double T = deltaDeltaT;T<=DeltaT;T+=deltaDeltaT)
      {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(mapFrame.makeShared());
        pair<double, double> end = result(w,v,T);
        pcl::PointXYZ searchPoint(end.first, end.second, 0);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if(kdtree.radiusSearch(searchPoint, 0.1, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0)
        {
        }
        else
        {
          scores[i]+=weighting(T);
        }
      }
      i++;
    }

    int minArg = 0;
    int secondMinArg = 0;
    int min = 50000;
    int currentScore;
    for(int j=0;j<nPaths;j++)
    {
      currentScore = scores[j];
      if (currentScore<min)
      {
        secondMinArg = minArg;
        minArg = secondMinArg;
        min = currentScore;
      }
    }

    cout << "minArg is" << endl;
    //available_actions.push_back(pair<double,double>(v, w));

    pair<double, double> minPair;
    minPair.first = v;
    minPair.second = -MaxW+wInc*minArg;
    /*
    double minDist = -1;

    for(vector< pair<double, double> >::iterator iter = available_actions.begin(); iter != available_actions.end(); iter++)
    {
        pair<double, double> action = (*iter);

        cout << "Action done" << endl;

        pair<double, double> waypointCoords;

        cout << _waypointReader->Current().Lat() << endl;

        waypointCoords.first = GPSdX(_currentAvgGPS, _waypointReader->Current());
        waypointCoords.second = GPSdY(_currentAvgGPS, _waypointReader->Current());

        cout << "loaded" << endl;

        pair<double, double> endLoc = result(action.second, action.first);

        cout << "resulted" << endl;

        double dist = distBetween(waypointCoords, endLoc);

        cout << "dist" << endl;

        if(minDist == -1 || dist < minDist)
        {
            minPair = action;
            minDist = dist;
        }
    }

    cout << "Computing velocities..." << endl;

    double Baseline = Robot::CurrentRobot().Baseline();
    double Vr = minPair.first + (Baseline/2.0)*minPair.second;
    double Vl = minPair.first - (Baseline/2.0)*minPair.second;

    cout << "Decided on Vl=" << Vl << " and Vr=" << Vr << endl;

    _driver->setVelocities(Vl, Vr);
    */
}

double CompetitionController::headingFromAToB(GPSData A, GPSData B)
{
    double dy = B.Lat() - A.Lat();
    double dx = cos(M_PI/180.0*A.Lat())*(B.Long() - A.Long());
    return atan2(dy, dx);
}

double CompetitionController::distBetween(GPSData A, GPSData B)
{
    double dy = B.Lat() - A.Lat();
    double dx = cos(M_PI/180.0*A.Lat())*(B.Long() - A.Long());
    return sqrt(dx*dx + dy*dy);
}

double CompetitionController::distBetween(pair<double, double> A, pair<double, double> B)
{
    double dy = B.second - A.second;
    double dx = B.first - A.first;
    return sqrt(dx*dx + dy*dy);
}

double CompetitionController::GPSdX(GPSData A, GPSData B)
{
    return cos(M_PI/180.0*A.Lat())*(B.Long() - A.Long());
}

double CompetitionController::GPSdY(GPSData A, GPSData B)
{
    return B.Lat() - A.Lat();
}

pair<double, double> CompetitionController::result(double W, double V)
{
    Eigen::Vector3d endLocation;
    if(W != 0)
    {
        double R = V / W;
        double ICCx = cos(- M_PI/2.0) * R;
        double ICCy = sin(- M_PI/2.0) * R;
        using namespace Eigen;
        Matrix3d T;
        double wdt = W*DeltaT;
        T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
        Vector3d a(-ICCx, -ICCy, 0);
        Vector3d b(ICCx, ICCy, wdt);
        endLocation = T * a + b;
    }
    else
    {
        endLocation[0] = 0;
        endLocation[1] = V * DeltaT;
    }
    return pair<double, double> (endLocation[0], endLocation[1]);
}

pair<double, double> CompetitionController::result(double W, double V, double dt)
{
    Eigen::Vector3d endLocation;
    if(W != 0)
    {
        double R = V / W;
        double ICCx = cos(- M_PI/2.0) * R;
        double ICCy = sin(- M_PI/2.0) * R;
        using namespace Eigen;
        Matrix3d T;
        double wdt = W*dt;
        T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
        Vector3d a(-ICCx, -ICCy, 0);
        Vector3d b(ICCx, ICCy, wdt);
        endLocation = T * a + b;
    }
    else
    {
        endLocation[0] = 0;
        endLocation[1] = V * dt;
    }
    return pair<double, double> (endLocation[0], endLocation[1]);
}
CompetitionController::~CompetitionController()
{
    if(_gps)
        _gps->onNewData -= &LOnNewGPSData;
}

}
}
