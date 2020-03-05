#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>

#include <imu_viz/imu_visual.h>

#include <imu_viz/imu_display.h>

namespace igvc_rviz_plugins
{
ImuDisplay::ImuDisplay()
{
  acceleration_color_ =
      new rviz::ColorProperty("Acceleration Color", QColor(204, 51, 204), "Color to draw the acceleration arrows.",
                              this, SLOT(updateColorAndAlpha()));
  acceleration_alpha_ = new rviz::FloatProperty(
      "Acceleration Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateColorAndAlpha()));
  accel_arrow_length_ = new rviz::FloatProperty("Acceleration arrow length", 1.0,
                                                "Multiplier for the length to draw the acceleration arrow", this,
                                                SLOT(updateAccelerationArrowLength()));
  ema_weight_ = new rviz::FloatProperty(
      "EMA weight", 1.0, "Coeffecient used for EMA filter. 1.0 to turn off, 0.0 to turn off new observations.", this);

  orientation_color_ =
      new rviz::ColorProperty("Orientation Color", QColor(204, 51, 204), "Color to draw the orientation arrows.", this,
                              SLOT(updateColorAndAlpha()));
  orientation_alpha_ = new rviz::FloatProperty("Orientation Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
                                               this, SLOT(updateColorAndAlpha()));
  orientation_arrow_length_ = new rviz::FloatProperty("Orientation arrow length", 1.0,
                                                      "Multiplier for the length to draw the orientation arrow", this,
                                                      SLOT(updateOrientationArrowLength()));

  angular_color_ =
      new rviz::ColorProperty("Angular Velocity Color", QColor(204, 51, 204),
                              "Color to draw the angular velocity arrows.", this, SLOT(updateColorAndAlpha()));
  angular_alpha_ = new rviz::FloatProperty(
      "Angular Velocity Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateColorAndAlpha()));
  angular_arrow_length_ = new rviz::FloatProperty("Orientation arrow length", 1.0,
                                                  "Multiplier for the length to draw the orientation arrow", this,
                                                  SLOT(updateAngularArrowLength()));

  use_2d_ = new rviz::BoolProperty("Use 2d", false, "Enable to ignore z", this);

  history_length_ = new rviz::IntProperty("History Length", 1, "Number of prior measurements to display.", this,
                                          SLOT(updateHistoryLength()));
  history_length_->setMin(1);
  history_length_->setMax(100000);
}

void ImuDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

void ImuDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void ImuDisplay::updateColorAndAlpha()
{
  float acceleration_alpha = acceleration_alpha_->getFloat();
  Ogre::ColourValue acceleration_color = acceleration_color_->getOgreColor();
  float orientation_alpha = orientation_alpha_->getFloat();
  Ogre::ColourValue orientation_color = orientation_color_->getOgreColor();

  for (const auto& visual : visuals_)
  {
    visual->setAccelerationColor(acceleration_color.r, acceleration_color.g, acceleration_color.b, acceleration_alpha);
    visual->setOrientationColor(orientation_color.r, orientation_color.g, orientation_color.b, orientation_alpha);
  }
}

void ImuDisplay::updateAccelerationArrowLength()
{
  float length = accel_arrow_length_->getFloat();

  for (const auto& visual : visuals_)
  {
    visual->setAccelerationLength(length);
  }
}

void ImuDisplay::updateOrientationArrowLength()
{
  float length = orientation_arrow_length_->getFloat();

  for (const auto& visual : visuals_)
  {
    visual->setOrientationLength(length);
  }
}

void ImuDisplay::updateAngularArrowLength()
{
  float length = angular_arrow_length_->getFloat();

  for (const auto& visual : visuals_)
  {
    visual->setAngularLength(length);
  }
}

void ImuDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_->getInt());
}

void ImuDisplay::processMessage(const sensor_msgs::ImuConstPtr& msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation))
  {
    ROS_DEBUG_STREAM("Error transforming from frame " << msg->header.frame_id << " to frame "
                                                      << fixed_frame_.toStdString());
  }

  std::unique_ptr<ImuVisual> visual;
  if (visuals_.full())
  {
    visual = std::move(visuals_.front());
  }
  else
  {
    visual = std::make_unique<ImuVisual>(context_->getSceneManager(), scene_node_);
  }

  bool use_2d = use_2d_->getBool();

  float ema_weight = ema_weight_->getFloat();
  visual->setMessage(msg, use_2d, last_message_, ema_weight);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float acceleration_alpha = acceleration_alpha_->getFloat();
  Ogre::ColourValue acceleration_color = acceleration_color_->getOgreColor();
  float orientation_alpha = orientation_alpha_->getFloat();
  Ogre::ColourValue orientation_color = orientation_color_->getOgreColor();
  float angular_alpha = angular_alpha_->getFloat();
  Ogre::ColourValue angular_color = angular_color_->getOgreColor();

  visual->setAccelerationColor(acceleration_color.r, acceleration_color.g, acceleration_color.b, acceleration_alpha);
  visual->setOrientationColor(orientation_color.r, orientation_color.g, orientation_color.b, orientation_alpha);
  visual->setAngularColor(angular_color.r, angular_color.g, angular_color.b, angular_alpha);

  visuals_.push_back(std::move(visual));
}

}  // namespace igvc_rviz_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(igvc_rviz_plugins::ImuDisplay, rviz::Display)
// END_TUTORIAL
