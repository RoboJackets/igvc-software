#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include <rviz/ogre_helpers/arrow.h>

#include <tf/transform_datatypes.h>

#include <imu_viz/imu_visual.h>

namespace igvc_rviz_plugins
{
// BEGIN_TUTORIAL
ImuVisual::ImuVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
  : scene_manager_{ scene_manager }
  , frame_node_{ parent_node->createChildSceneNode() }
  , acceleration_arrow_{ std::make_unique<rviz::Arrow>(scene_manager_, frame_node_) }
  , orientation_arrow_{ std::make_unique<rviz::Arrow>(scene_manager_, frame_node_) }
  , angular_arrow_{ std::make_unique<rviz::Arrow>(scene_manager_, frame_node_) }
  , acceleration_multiplier_{ 1.0f }
  , orientation_multiplier_{ 1.0f }
  , angular_multiplier_{ 1.0f }
{
}

ImuVisual::~ImuVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void ImuVisual::setMessage(const sensor_msgs::Imu::ConstPtr& msg, bool use_2d,
                           std::unique_ptr<sensor_msgs::Imu>& last_msg, float ema_weight)
{
  geometry_msgs::Vector3 a;
  tf::Matrix3x3 orientation_3x3;
  // First message.
  if (!last_msg)
  {
    last_msg = std::make_unique<sensor_msgs::Imu>();
    a = msg->linear_acceleration;

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(msg->orientation, orientation);
    orientation_3x3 = tf::Matrix3x3{ orientation };
  }
  else
  {
    const geometry_msgs::Vector3& new_a = msg->linear_acceleration;
    const geometry_msgs::Vector3& old_a = last_msg->linear_acceleration;
    a.x = ema_weight * new_a.x + (1.0f - ema_weight) * old_a.x;
    a.y = ema_weight * new_a.y + (1.0f - ema_weight) * old_a.y;
    a.z = ema_weight * new_a.z + (1.0f - ema_weight) * old_a.z;

    tf::Quaternion new_orientation;
    tf::quaternionMsgToTF(msg->orientation, new_orientation);

    tf::Quaternion last_orientation;
    tf::quaternionMsgToTF(last_msg->orientation, last_orientation);
    last_orientation.slerp(new_orientation, ema_weight);
    orientation_3x3 = tf::Matrix3x3{ new_orientation };
  }
  float angular_velocity = msg->angular_velocity.z;

  Ogre::Vector3 acc(a.x, a.y, a.z);

  tf::Vector3 forward{ 1.0f, 0.0f, 0.0f };
  forward = orientation_3x3 * forward;
  Ogre::Vector3 orientation_ogre(forward.x(), forward.y(), forward.z());
  if (use_2d)
  {
    acc.z = 0;
    orientation_ogre.z = 0;
  }

  setAccelerationArrow(acc);
  setOrientationArrow(orientation_ogre);
  setAngularArrow(angular_velocity);

  last_msg->linear_acceleration = a;
  tf::Quaternion quat;
  orientation_3x3.getRotation(quat);
  tf::quaternionTFToMsg(quat, last_msg->orientation);
  last_msg->angular_velocity.z = angular_velocity;
}

void ImuVisual::setAccelerationArrow(const Ogre::Vector3& acc)
{
  float acceleration_length = acc.length();

  Ogre::Vector3 acceleration_scale(acceleration_length, acceleration_length, acceleration_length);
  acceleration_arrow_->setScale(acceleration_multiplier_ * acceleration_scale);
  acceleration_arrow_->setDirection(acc);
}

void ImuVisual::setOrientationArrow(const Ogre::Vector3& orientation)
{
  Ogre::Vector3 orientation_scale(1, 1, 1);
  orientation_arrow_->setScale(orientation_multiplier_ * orientation_scale);

  orientation_arrow_->setDirection(frame_node_->getOrientation().Inverse() * orientation);
}

void ImuVisual::setAngularArrow(float angular_velocity)
{
  Ogre::Vector3 angular_scale(1, 1, 1);
  angular_arrow_->setScale(angular_multiplier_ * angular_velocity * angular_scale);
  angular_arrow_->setDirection(Ogre::Vector3::UNIT_Y);
}

void ImuVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void ImuVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

void ImuVisual::setAccelerationColor(float r, float g, float b, float a)
{
  acceleration_arrow_->setColor(r, g, b, a);
}

void ImuVisual::setOrientationColor(float r, float g, float b, float a)
{
  orientation_arrow_->setColor(r, g, b, a);
}

void ImuVisual::setAngularColor(float r, float g, float b, float a)
{
  angular_arrow_->setColor(r, g, b, a);
}

void ImuVisual::setAccelerationLength(float multiplier)
{
  acceleration_multiplier_ = multiplier;
}

void ImuVisual::setOrientationLength(float multiplier)
{
  orientation_multiplier_ = multiplier;
}

void ImuVisual::setAngularLength(float multiplier)
{
  angular_multiplier_ = multiplier;
}
// END_TUTORIAL

}  // namespace igvc_rviz_plugins
