//
// Created by aaronmao on 1/20/20.
//

#include "fake_cone.h";

bool serviceCallBack() {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_cone_service");
    ros::NodeHandle nh;
    //fake_cone::FakeConeService fakeConeService;
    //ros::ServiceServer service = nh.advertiseService("fake_cone_service", serviceCallBack);
    ros::spin();
}