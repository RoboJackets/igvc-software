#include <igvc_rviz_plugins_old/speed_panel.h>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QVBoxLayout>
#include <string>

namespace rviz_plugins
{
void SpeedPanel::subCallback(const igvc_msgs::velocity_pair& msg)
{
  speed = (msg.left_velocity + msg.right_velocity) / 2;
  std::string str = std::to_string(speed);
  speedometer->setValue(speed);
  // QString qstr = QString::fromStdString(str);
  // velocity_label->setText(qstr);
  // std::string str2=std::to_string(msg.left_velocity);
  // QString qstr2 = QString::fromStdString(str2);
  // velocity_left_label->setText(qstr2);
  // std::string str3=std::to_string(msg.right_velocity);
  // QString qstr3 = QString::fromStdString(str3);
  // velocity_right_label->setText(qstr3);
}

SpeedPanel::SpeedPanel(QWidget* parent) : rviz::Panel(parent)
{
  speedometer = new Speedometer(parent);
  QHBoxLayout* speedDisp = new QHBoxLayout;
  speedDisp->addWidget(new QLabel("SPEEDOMETER"));
  speedDisp->addWidget(speedometer);

  /*QHBoxLayout* velocity_layout = new QHBoxLayout;
  velocity_layout->addWidget( new QLabel( "VELOCITY:" ) );
  velocity_label = new QLabel("COOLBEFORE");
  velocity_layout->addWidget( velocity_label );

  QHBoxLayout* velocity_left_layout = new QHBoxLayout;
  velocity_left_layout->addWidget( new QLabel( "VELOCITY_LEFT:" ) );
  velocity_left_label = new QLabel("COOLBEFORE");
  velocity_left_layout->addWidget( velocity_left_label );

  QHBoxLayout* velocity_right_layout = new QHBoxLayout;
  velocity_right_layout->addWidget( new QLabel( "VELOCITY_RIGHT:" ) );
  velocity_right_label = new QLabel("COOLBEFORE");
  velocity_right_layout->addWidget( velocity_right_label );
*/
  QVBoxLayout* layout = new QVBoxLayout;
  /*layout->addLayout( velocity_layout );
  layout->addLayout( velocity_left_layout );
  layout->addLayout( velocity_right_layout);*/
  layout->addLayout(speedDisp);
  setLayout(layout);

  sub = nh_.subscribe("/encoders", 1, &SpeedPanel::subCallback, this);

  // connect( this, SIGNAL( changeText() ), output_topic_editor_, SLOT( setTextLabel() ));
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::SpeedPanel, rviz::Panel)
