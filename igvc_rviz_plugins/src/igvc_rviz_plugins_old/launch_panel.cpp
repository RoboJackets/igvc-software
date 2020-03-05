#include <stdio.h>

#include <QHBoxLayout>
#include <QLineEdit>
#include <QVBoxLayout>

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <QDir>
#include <QStringList>

#include <igvc_rviz_plugins_old/launch_panel.h>

namespace rviz_plugins
{
void LaunchPanel::handleButton()
{
  MyThread* myThread = new MyThread();
  QString cmd = "roslaunch igvc " + output_topic_editor_->currentText();
  std::string stdcmd = cmd.toStdString();
  const char* strcmd = stdcmd.c_str();
  myThread->setCmd(strcmd);
  myThread->start();
}

LaunchPanel::LaunchPanel(QWidget* parent) : rviz::Panel(parent)
{
  doOnce = true;
  start = time(0);

  QHBoxLayout* topic_layout = new QHBoxLayout;
  QDir recoredDir("igvc/launch/");
  recoredDir.setNameFilters(QStringList() << "*.launch");
  QStringList allFiles = recoredDir.entryList(
      QDir::NoDotAndDotDot | QDir::System | QDir::Hidden | QDir::AllDirs | QDir::Files, QDir::DirsFirst);
  output_topic_editor_ = new QComboBox();
  for (int i = 0; i < allFiles.size(); i++)
  {
    output_topic_editor_->addItem(allFiles[i]);
  }
  topic_layout->addWidget(output_topic_editor_);

  m_button = new QPushButton("Launch", this);
  m_button->setGeometry(QRect(QPoint(100, 100), QSize(200, 50)));
  topic_layout->addWidget(m_button);

  connect(m_button, SIGNAL(released()), this, SLOT(handleButton()));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  setLayout(layout);
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::LaunchPanel, rviz::Panel)
