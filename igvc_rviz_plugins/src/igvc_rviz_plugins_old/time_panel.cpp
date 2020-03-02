#include <stdio.h>

#include <QHBoxLayout>
#include <QLineEdit>
#include <QVBoxLayout>

#include <igvc_rviz_plugins_old/time_panel.h>

namespace rviz_plugins
{
void TimePanel::timeCallback(const std_msgs::UInt8& msg)
{
  char buf[80];
  struct tm tstruct;
  time_t diff = (time(0) - start);
  tstruct = *localtime(&diff);
  strftime(buf, sizeof(buf), "%M:%S", &tstruct);
  output_topic_editor_->setText(buf);
}

TimePanel::TimePanel(QWidget* parent) : rviz::Panel(parent)
{
  start = time(0);

  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Uptime:"));
  output_topic_editor_ = new QLabel("TEST");
  topic_layout->addWidget(output_topic_editor_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  setLayout(layout);
  output_topic_editor_->setText("No Signal");

  sub = nh_.subscribe("/battery", 1, &TimePanel::timeCallback, this);

  // connect( this, SIGNAL( changeText() ), output_topic_editor_, SLOT( setTextLabel() ));
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TimePanel, rviz::Panel)
