#ifndef IMU_DISPLAY_H
#define IMU_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <sensor_msgs/Imu.h>

#include <imu_viz/imu_visual.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}  // namespace rviz

namespace igvc_rviz_plugins
{
class ImuDisplay : public rviz::MessageFilterDisplay<sensor_msgs::Imu>
{
  Q_OBJECT
public:
  ImuDisplay();
  ~ImuDisplay() override = default;

protected:
  void onInitialize() override;

  void reset() override;

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();
  void updateAccelerationArrowLength();
  void updateOrientationArrowLength();
  void updateAngularArrowLength();
  void updateEMA();
  void update2D();

private:
  void processMessage(const sensor_msgs::ImuConstPtr& msg) override;

  boost::circular_buffer<std::unique_ptr<ImuVisual> > visuals_;

  rviz::ColorProperty* acceleration_color_;
  rviz::FloatProperty* acceleration_alpha_;
  rviz::FloatProperty* accel_arrow_length_;
  rviz::FloatProperty* ema_weight_;

  rviz::ColorProperty* orientation_color_;
  rviz::FloatProperty* orientation_alpha_;
  rviz::FloatProperty* orientation_arrow_length_;

  rviz::ColorProperty* angular_color_;
  rviz::FloatProperty* angular_alpha_;
  rviz::FloatProperty* angular_arrow_length_;

  rviz::BoolProperty* use_2d_;
  rviz::IntProperty* history_length_;

  std::unique_ptr<sensor_msgs::Imu> last_message_;
};

}  // namespace igvc_rviz_plugins

#endif  // IMU_DISPLAY_H
