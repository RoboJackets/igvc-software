#ifndef IMU_VISUAL_H
#define IMU_VISUAL_H

#include <sensor_msgs/Imu.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}  // namespace Ogre

namespace rviz
{
class Arrow;
}

namespace igvc_rviz_plugins
{
class ImuVisual
{
public:
  ImuVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  ~ImuVisual();

  void setMessage(const sensor_msgs::Imu::ConstPtr& msg, bool use_2d, std::unique_ptr<sensor_msgs::Imu>& last_message,
                  float ema_weight);

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setAccelerationColor(float r, float g, float b, float a);
  void setOrientationColor(float r, float g, float b, float a);
  void setAngularColor(float r, float g, float b, float a);
  void setAccelerationLength(float length);
  void setOrientationLength(float length);
  void setAngularLength(float length);

  void setAccelerationArrow(const Ogre::Vector3& acc);
  void setOrientationArrow(const Ogre::Vector3& acc);
  void setAngularArrow(float angular_velocity);

private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
  std::unique_ptr<rviz::Arrow> acceleration_arrow_;
  std::unique_ptr<rviz::Arrow> orientation_arrow_;
  std::unique_ptr<rviz::Arrow> angular_arrow_;
  float acceleration_multiplier_;
  float orientation_multiplier_;
  float angular_multiplier_;
};

}  // namespace igvc_rviz_plugins

#endif  // IMU_VISUAL_H
