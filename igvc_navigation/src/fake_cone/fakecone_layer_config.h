#ifndef SRC_FAKECONE_LAYER_CONFIG_H
#define SRC_FAKECONE_LAYER_CONFIG_H

#include <ros/ros.h>
#include "../mapper/map_config.h"

namespace fakecone_layer
{
    class FakeconeLayerConfig
            {
            public:
                explicit FakeconeLayerConfig(const ros::NodeHandle& parent_nh);

                ros::NodeHandle nh;
                map::MapConfig map;
            };
}

#endif //SRC_FAKECONE_LAYER_CONFIG_H
