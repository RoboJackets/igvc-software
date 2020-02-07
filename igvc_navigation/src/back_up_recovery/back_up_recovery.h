#ifndef SRC_BACK_UP_RECOVERY_H
#define SRC_BACK_UP_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>
namespace back_up_recovery
{
/**
 * @class BackUpRecovery
 * @brief A recovery behavior that makes the robot move backwards a bit
 */
class BackUpRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  BackUpRecovery();

  /**
   * @brief  Initialization function for the BackupRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   * @param global_costmap (unused)
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
  void initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*,
                  costmap_2d::Costmap2DROS* local_costmap) override;

  /**
   * @brief  Run the BackupRecovery recovery behavior.
   */
  void runBehavior() override;

  /**
   * @brief  Destructor for the back up recovery behavior
   */
  ~BackUpRecovery() override;

private:
  bool initialized_;
  double velocity_, frequency_, threshold_distance_, obstacle_distance_;
  std::unique_ptr<costmap_2d::Costmap2DROS> local_costmap_;
  std::unique_ptr<base_local_planner::CostmapModel> world_model_;

  bool nextStepValid(geometry_msgs::Point current_position, double yaw);
};

}  // namespace back_up_recovery
#endif  // SRC_BACK_UP_RECOVERY_H
