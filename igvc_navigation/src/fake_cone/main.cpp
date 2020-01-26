//
// Created by aaronmao on 1/20/20.
//

#include "fake_cone.h"
#include <igvc_msgs/fake_cone.h>
#include <geometry_msgs/Point.h>


bool scanAndGenerate(igvc_msgs::fake_coneRequest& req,
                    igvc_msgs::fake_coneResponse& res) {
    
    fake_cone::FakeConeService fakeConeService;

    geometry_msgs::Point leftContact = fakeConeService.findLine(req.costMap, req.currentPos, true);
    geometry_msgs::Point rightContact = fakeConeService.findLine(req.costMap, req.currentPos, false);
    
    std::vector<geometry_msgs::Point> leftLine = fakeConeService.linearProbe(req.costMap, leftContact);
    std::vector<geometry_msgs::Point> rightLine = fakeConeService.linearProbe(req.costMap, rightContact);

    geometry_msgs::Point leftEndPoint = fakeConeService.findEndpoint(leftLine, leftContact, req.currentPos);
    geometry_msgs::Point rightEndPoint = fakeConeService.findEndpoint(rightLine, rightContact, req.currentPos);

    std::vector<geometry_msgs::Point> fakeConeLine = fakeConeService.connectEndpoints(leftEndPoint, rightEndPoint);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_cone_service");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("fake_cone_service", scanAndGenerate);
    ros::spin();
}