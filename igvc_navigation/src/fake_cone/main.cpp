//
// Created by aaronmao on 1/20/20.
//

#include "fake_cone.h"
#include <igvc_msgs/fake_cone.h>
#include <geometry_msgs/Point.h>


bool scanAndGenerate(igvc_msgs::fake_cone::Request &req,
                    igvc_msgs::fake_cone::Response &res) {
    geometry_msgs::Point point;
    point.x = req.x_pos;
    point.y = req.y_pos;

    fake_cone::FakeConeService fakeConeService;
    fakeConeService.linearProbe(req.costMap, point);
}

bool serviceCallBack() {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_cone_service");
    ros::NodeHandle nh;
    //ros::ServiceServer service = nh.advertiseService("fake_cone_service", serviceCallBack);
    ros::spin();
}