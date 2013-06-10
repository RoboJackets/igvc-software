
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <mapping/PointCloudAdder.hpp>

class AdderListener
{
public:
    AdderListener(Event<PC>* evt)
        : _viewer("Say cheese!"),
          LOnNewCloud(this)
    {
        (*evt) += &LOnNewCloud;
    }

    void OnNewCloud(PC cloud)
    {
        cout << cloud.points.size() << endl;
        _viewer.removeAllPointClouds(0);
        _viewer.addPointCloud(cloud.makeShared(), "cloud");
        _viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    }
    LISTENER(AdderListener, OnNewCloud, PC);

    pcl::visualization::PCLVisualizer _viewer;
};

int main()
{
    Event<PC> firstEvent;
    Event<PC> secondEvent;

    PointCloudAdder pcAdder(&firstEvent, &secondEvent);

    AdderListener listener(&pcAdder.onBothData);

    PC cloud1;
    for(double theta = 0; theta < M_PI * 2; theta += M_PI / 12.0)
    {
        cloud1.points.push_back(pcl::PointXYZ(cos(theta)*1, sin(theta)*1, 0));
    }

    PC cloud2;
    for(double theta = M_PI * (4.0/3.0); theta < M_PI * (5.0/3.0); theta += M_PI / 12.0)
    {
        cloud2.points.push_back(pcl::PointXYZ(cos(theta)*0.75, sin(theta)*0.75, 0));
    }
    cloud2.points.push_back(pcl::PointXYZ(cos(M_PI/3.0)*0.75,       sin(M_PI/3.0)*0.75,       0));
    cloud2.points.push_back(pcl::PointXYZ(cos(M_PI*(2.0/3.0))*0.75, sin(M_PI*(2.0/3.0))*0.75, 0));

    firstEvent(cloud1);
    secondEvent(cloud2);

    while(!listener._viewer.wasStopped())
    {
        listener._viewer.spinOnce();
    }

    return 0;
}
