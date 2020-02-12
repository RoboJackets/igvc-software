//
// Created by pcapple on 2/2/20.
//

#include <ros/ros.h>
#include <parameter_assertions/assertions.h>
#include "barrel_config.h"

namespace line_layer
{
    BarrelConfig::BarrelConfig(const ros::NodeHandle& parent_nh)
    {
        ros::NodeHandle nh {parent_nh, "barrel"};

        assertions::getParam(nh, "blur_size", blur_size);

        assertions::getParam(nh, "barrel_value", barrel_value);

        assertions::getParam(nh, "min_h", min_h);
        assertions::getParam(nh, "min_s", min_s);
        assertions:: getParam(nh, "min_v", min_v);

        assertions::getParam(nh, "max_h", max_h);
        assertions::getParam(nh, "max_s", max_s);
        assertions::getParam(nh, "max_v", max_v);

        assertions::getParam(nh, "debug", debug);


    }
}
