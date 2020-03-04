#include <stdio.h>
#include <string>

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <igvc_rviz_plugins_old/eStop_panel.h>

namespace rviz_plugins
{
void EStopPanel::subCallback(const std_msgs::Bool& msg)
{
  if (msg.data)
  {
    output_topic_editor_->setStyleSheet("QLabel {color : green;}");
    output_topic_editor_->setText("enabled");
  }
  else
  {
    output_topic_editor_->setStyleSheet("QLabel {color : red;}");
    output_topic_editor_->setText("disabled");
  }
}

EStopPanel::EStopPanel(QWidget* parent) : rviz::Panel(parent)
{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("eStop Status:"));
  output_topic_editor_ = new QLabel("No Signal");
  topic_layout->addWidget(output_topic_editor_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  setLayout(layout);

  sub = nh_.subscribe("/robot_enabled", 1, &EStopPanel::subCallback, this);

  // connect( this, SIGNAL( changeText() ), output_topic_editor_, SLOT( setTextLabel() ));
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::EStopPanel, rviz::Panel)
