//
// Created by aaronmao on 12/28/19.
//

#include "fakecone_layer_config.h"

namespace fakecone_layer
{
FakeconeLayerConfig::FakeconeLayerConfig(const ros::NodeHandle &parent_nh)
    : nh {parent_nh, "fakecone_layer"}
    , map{nh}
{
}
}