#include <mapper/probability_utils.h>
#include <pluginlib/class_list_macros.h>
#include "fakecone_layer.h"
#include "../mapper/map_config.h"
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
#include <igvc_msgs/fake_cone.h>
#include <costmap_2d/costmap_layer.h>
#include <tf2_ros/transform_listener.h>

PLUGINLIB_EXPORT_CLASS(fakecone_layer::FakeconeLayer, costmap_2d::Layer)

namespace fakecone_layer
{
    fake_cone::FakeConeService fakeConeService;

    FakeconeLayer::FakeconeLayer() : GridmapLayer({logodds_layer, probability_layer}), private_nh_{"~"}, config_{private_nh_}
    {
        initGridmap();
        initPubSub();

        ros::NodeHandle nh;
        endPointPublisher = nh.advertise<sensor_msgs::PointCloud2>("fakecone_cone", 1);

        ros::ServiceServer service = nh.advertiseService("fake_cone_service", &FakeconeLayer::scanAndGenerate, this);
        ROS_INFO("Ready to accept input");
    }

    void FakeconeLayer::initGridmap()
    {
        map_.setFrameId(config_.map.frame_id);
        grid_map::Length dimensions{config_.map.length_x, config_.map.length_y};
        map_.setGeometry(dimensions, config_.map.resolution);
        layer_ = &map_.get(logodds_layer);
        (*layer_).setZero();
        
        grid_map::Position top_left;
        map_.getPosition(map_.getStartIndex(), top_left);
    }
    
    void FakeconeLayer::initPubSub() 
    {
        costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(config_.map.costmap_topic, 1);
        matchCostmapDims(*layered_costmap_->getCostmap());
    }
    
    void FakeconeLayer::onInitialize() 
    {
        GridmapLayer::onInitialize();
    }
    
    void FakeconeLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) 
    {
        matchCostmapDims(master_grid);
        transferToCostmap();
        if (config_.map.debug.enabled) {
            publishCostmap();
        }
        resetDirty();

        uchar *master_array = master_grid.getCharMap();
        uchar *line_array = costmap_2d_.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();
        
        for (int j = min_j; j < max_j; j++) {
            unsigned int it = j * span + min_i;
            for (int i = min_i; i < max_i; i++) {
                unsigned char old_cost = master_array[it];
                if (old_cost == costmap_2d::NO_INFORMATION || old_cost < line_array[it])
                    master_array[it] = line_array[it];
                it++;
            }
        }
    }
    
    void FakeconeLayer::matchCostmapDims(const costmap_2d::Costmap2D &master_grid) 
    {
        unsigned int cells_x = master_grid.getSizeInCellsX();
        unsigned int cells_y = master_grid.getSizeInCellsY();
        double resolution = master_grid.getResolution();
        bool different_dims = costmap_2d_.getSizeInCellsX() != cells_x || costmap_2d_.getSizeInCellsY() != cells_y ||
                              costmap_2d_.getResolution() != resolution;

        double origin_x = master_grid.getOriginX();
        double origin_y = master_grid.getOriginY();

        if (different_dims)
        {
            costmap_2d_.resizeMap(cells_x, cells_y, resolution, origin_x, origin_y);
        }
        costmap_2d_.updateOrigin(origin_x, origin_y);
    }

    void FakeconeLayer::transferToCostmap()
    {
        updateStaticWindow();
    }

    void FakeconeLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                 double *max_x, double *max_y)
    {
        GridmapLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

        if (rolling_window_)
        {
            costmap_2d_.updateOrigin(robot_x - costmap_2d_.getSizeInMetersX() / 2,
                                     robot_y - costmap_2d_.getSizeInMetersY() / 2);
        }
    }

    void FakeconeLayer::updateStaticWindow()
    {
        size_t num_cells = map_.getSize().prod();
        uchar *char_map = costmap_2d_.getCharMap();

        auto optional_it = getDirtyIterator();

        if (!optional_it)
        {
            return;
        }

        for (auto it = *optional_it; !it.isPastEnd(); ++it)
        {
            const auto &log_odds = (*layer_)((*it)[0], (*it)[1]);
            float probability = probability_utils::fromLogOdds(log_odds);
            size_t linear_index = grid_map::getLinearIndexFromIndex(*it, map_.getSize(), false);

            if (probability > config_.map.occupied_threshold)
            {
                char_map[num_cells - linear_index - 1] = costmap_2d::LETHAL_OBSTACLE;
            }
            else
            {
                char_map[num_cells - linear_index - 1] = costmap_2d::FREE_SPACE;
            }
        }
    }

    void FakeconeLayer::markHit(const grid_map::Index &index)
    {
        (*layer_)(index[0], index[1]) = config_.map.max_occupancy;
    }

    void FakeconeLayer::insertLine(std::vector<geometry_msgs::Point> line)
    {
        for (geometry_msgs::Point point : line)
        {
            grid_map::Index index;
            grid_map::Position pos{point.x, point.y};

            index = grid_map::Index{point.x, point.y};
            markHit(index);
        }
    }

    void FakeconeLayer::publishCostmap()
    {
        if (cost_translation_table_.empty())
        {
            initCostTranslationTable();
        }

        nav_msgs::OccupancyGridPtr msg = boost::make_shared<nav_msgs::OccupancyGrid>();

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_2d_.getMutex()));
        double resolution = costmap_2d_.getResolution();

        msg->header.frame_id = config_.map.frame_id;
        msg->header.stamp = ros::Time::now();
        msg->info.resolution = resolution;

        msg->info.width = costmap_2d_.getSizeInCellsX();
        msg->info.height = costmap_2d_.getSizeInCellsY();

        grid_map::Position position = map_.getPosition() - 0.5 * map_.getLength().matrix();  // NOLINT
        msg->info.origin.position.x = position.x();
        msg->info.origin.position.y = position.y();
        msg->info.origin.position.z = 0.0;
        msg->info.origin.orientation.w = 1.0;

        msg->data.resize(msg->info.width * msg->info.height);

        unsigned char *data = costmap_2d_.getCharMap();
        for (size_t i = 0; i < msg->data.size(); i++)
        {
            msg->data[i] = cost_translation_table_[data[i]];
        }

        costmap_pub_.publish(msg);
    }

    bool FakeconeLayer::scanAndGenerate(igvc_msgs::fake_coneRequest &req,
            igvc_msgs::fake_coneResponse &res) {

        //NOTE: the robot will always be at the center of the map for a rolling window
        std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins = layered_costmap_->getPlugins();
        unsigned char* grid;

        for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {

            boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;

            if(plugin->getName().find("line_layer")!=std::string::npos) {
                boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
                costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
                grid = costmap->getCharMap();
            }
        }

        nav_msgs::OccupancyGrid costMapGrid = fakeConeService.convertCharMap(grid);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("odom", "imu",
            ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        
        geometry_msgs::Point currentPos;
        currentPos.x = transformStamped.transform.translation.x;
        currentPos.y = transformStamped.transform.translation.y;

        geometry_msgs::Point leftContact = fakeConeService.findLine(costMapGrid, currentPos, true);
        geometry_msgs::Point rightContact = fakeConeService.findLine(costMapGrid, currentPos, false);

        std::vector<geometry_msgs::Point> leftLine = fakeConeService.linearProbe(costMapGrid, leftContact);
        std::vector<geometry_msgs::Point> rightLine = fakeConeService.linearProbe(costMapGrid, rightContact);

        geometry_msgs::Point leftEndPoint = fakeConeService.findEndpoint(leftLine, leftContact, currentPos);
        geometry_msgs::Point rightEndPoint = fakeConeService.findEndpoint(rightLine, rightContact, currentPos);

        std::vector<geometry_msgs::Point> fakeConeLine = fakeConeService.connectEndpoints(leftEndPoint, rightEndPoint);
        pcl::PointCloud<pcl::PointXY> fakeConeCloud;

        fakeConeCloud.resize(costMapGrid.info.width * costMapGrid.info.height);
        for(geometry_msgs::Point point : fakeConeLine) {
            pcl::PointXY index;
            index.x = point.x;
            index.y = point.y;

            fakeConeCloud.push_back(index);
        }

        //Publishing the debug message for the calculated line for rviz
        sensor_msgs::PointCloud2 lineDebug;
        pcl::toROSMsg(fakeConeCloud, lineDebug);
        endPointPublisher.publish(lineDebug);

        //TODO: dummy return
        return true;
    }
}