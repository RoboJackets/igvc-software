#include <stdio.h>

#include <QHBoxLayout>
#include <QLineEdit>
#include <QVBoxLayout>

#include <igvc_rviz_plugins_old/bat_panel.h>

namespace rviz_plugins
{
void BatPanel::batCallback(const std_msgs::UInt8& msg)
{
  char str[4];
  sprintf(str, "%u", msg.data);
  output_topic_editor_->setText(str);
}

BatPanel::BatPanel(QWidget* parent) : rviz::Panel(parent)
{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Battery Level:"));
  output_topic_editor_ = new QLabel("TEST");
  topic_layout->addWidget(output_topic_editor_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  setLayout(layout);
  output_topic_editor_->setText("No Signal");

  sub = nh_.subscribe("/battery", 1, &BatPanel::batCallback, this);

  // connect( this, SIGNAL( changeText() ), output_topic_editor_, SLOT( setTextLabel() ));
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::BatPanel, rviz::Panel)
