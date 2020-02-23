//
// Created by aaronmao on 12/27/19.
//

#ifndef SRC_FAKECONE_LAYER_H
#define SRC_FAKECONE_LAYER_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../mapper/gridmap_layer.h"
#include "fakecone_layer_config.h"
#include "fake_cone.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <igvc_msgs/fake_cone.h>
#include <igvc_msgs/fake_coneRequest.h>
#include <igvc_msgs/fake_coneResponse.h>

namespace fakecone_layer {
    class FakeconeLayer : public gridmap_layer::GridmapLayer {
    public:
        using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

        FakeconeLayer();

        void onInitialize() override;

        void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;

        void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x,
                          double *max_y) override;

        void markHit(const grid_map::Index &index);
        void insertLine(std::vector<geometry_msgs::Point> line);
        bool scanAndGenerate(igvc_msgs::fake_coneRequest &req,
                             igvc_msgs::fake_coneResponse &res);

    private:
        static constexpr auto logodds_layer = "logodds";
        static constexpr auto probability_layer = "probability";
        // This generate the fakeConeService
        fake_cone::FakeConeService fakeConeService;
        ros::Publisher endPointPublisher;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        grid_map::Matrix *layer_{};
        FakeconeLayerConfig config_;

        ros::Publisher costmap_pub_;
        std::vector<int8_t> cost_translation_table_;

        costmap_2d::Costmap2D costmap_2d_{};

        void initGridmap();
        void initPubSub();
        void matchCostmapDims(const costmap_2d::Costmap2D& master_grid);
        void transferToCostmap();
        void updateStaticWindow();
        void publishCostmap();
        void initCostTranslationTable();
    };
}

#endif //SRC_FAKECONE_LAYER_H
