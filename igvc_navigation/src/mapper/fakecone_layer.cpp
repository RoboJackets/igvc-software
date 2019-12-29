#include <mapper/probability_utils.h>
#include "fakecone_layer.h"
#include "map_config.h"

//PLUGINLIB_EXPORT_CLASS(fakecone_layer::FakeconeLayer, costmap_2d::Layer)

namespace fakecone_layer
{
    void FakeconeLayer::initGridmap() {
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
        if (config_.map.debug.enabled) {
            //TODO: create debug publishers and subscribers in the header file and initialize
        }
        costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(config_.map.costmap_topic, 1);
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
            //TODO: implement own debugging steps
            //updateProbabilityLayer();
            //debugPublishMap();
            //publishCostmap();
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
    
    void FakeconeLayer::markEmpty(const grid_map::Index, double distance) 
    {
        //TODO: implement the function
    }
    
    void FakeconeLayer::markHit(const grid_map::Index, double distance)
    {
        //TODO: implement the function
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

    void FakeconeLayer::updateProbabilityLayer()
    {
        auto optional_it = getDirtyIterator();
        if (!optional_it)
        {
            return;
        }

        for (auto it = *optional_it; !it.isPastEnd(); ++it)
        {
            map_.at(probability_layer, *it) = probability_utils::fromLogOdds((*layer_)((*it)[0], (*it)[1]));
        }
    }

    void FakeconeLayer::transferToCostmap()
    {
        if (rolling_window_)
        {
            updateRollingWindow();
        }
        else
        {
            updateStaticWindow();
        }
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

    void FakeconeLayer::updateRollingWindow()
    {
        // Rolling window, so we need to move everything

        // Convert from costmap_2d_ coordinate frame to gridmap coordinate frame
        const size_t cells_x = costmap_2d_.getSizeInCellsX();
        const size_t cells_y = costmap_2d_.getSizeInCellsY();
        const double resolution = costmap_2d_.getResolution();
        grid_map::Position costmap_br_corner{ costmap_2d_.getOriginX(), costmap_2d_.getOriginY() };
        grid_map::Position costmap_tl_corner =
                costmap_br_corner + grid_map::Position{ costmap_2d_.getSizeInMetersX(), costmap_2d_.getSizeInMetersY() };
        grid_map::Index start_index;
        map_.getIndex(costmap_tl_corner, start_index);

        grid_map::Index submap_buffer_size{ costmap_2d_.getSizeInCellsX(), costmap_2d_.getSizeInCellsY() };

        uchar *char_map = costmap_2d_.getCharMap();

        size_t x_idx = cells_x - 1;
        size_t y_idx = cells_y - 1;

        // This goes top -> down, left -> right
        // but costmap_2d_ indicies go down -> up, left -> right
        for (grid_map::SubmapIterator it{ map_, start_index, submap_buffer_size }; !it.isPastEnd(); ++it)
        {
            const auto &log_odds = (*layer_)((*it)[0], (*it)[1]);
            float probability = probability_utils::fromLogOdds(log_odds);

            const size_t linear_idx = x_idx + y_idx * cells_x;

            if (probability > config_.map.occupied_threshold)
            {
                char_map[linear_idx] = costmap_2d::LETHAL_OBSTACLE;
            }
            else
            {
                char_map[linear_idx] = costmap_2d::FREE_SPACE;
            }

            if (y_idx == 0)
            {
                y_idx = cells_y - 1;
                x_idx--;
            }
            else
            {
                y_idx--;
            }
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

    void FakeconeLayer::debugPublishMap()
    {
        map_.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap message;
        std::vector<std::string> layers{ probability_layer };
        grid_map::GridMapRosConverter::toMessage(map_, layers, message);
        gridmap_pub_.publish(message);
    }

    void FakeconeLayer::initCostTranslationTable()
    {
        constexpr int8_t free_space_msg_cost = 0;
        constexpr int8_t inflated_msg_cost = 99;
        constexpr int8_t lethal_msg_cost = 100;
        constexpr int8_t unknown_msg_cost = -1;

        cost_translation_table_.resize(std::numeric_limits<uchar>::max() + 1);

        cost_translation_table_[costmap_2d::FREE_SPACE] = free_space_msg_cost;                 // NO obstacle
        cost_translation_table_[costmap_2d::INSCRIBED_INFLATED_OBSTACLE] = inflated_msg_cost;  // INSCRIBED obstacle
        cost_translation_table_[costmap_2d::LETHAL_OBSTACLE] = lethal_msg_cost;                // LETHAL obstacle
        cost_translation_table_[costmap_2d::NO_INFORMATION] = unknown_msg_cost;                // UNKNOWN

        // regular cost values scale the range 1 to 252 (inclusive) to fit
        // into 1 to 98 (inclusive).
        for (int i = 1; i <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1; i++)
        {
            // NOLINTNEXTLINE
            cost_translation_table_[i] = static_cast<uint8_t>(1 + (97 * (i - 1)) / 251);
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
}