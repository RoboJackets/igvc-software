#include <stdio.h>

#include <QHBoxLayout>
#include <QLineEdit>

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <QDir>
#include <QStringList>

#include <igvc_rviz_plugins_old/node_panel.h>

namespace rviz_plugins
{
void NodePanel::closeNode(int num)
{
  if (myThreads.at(num) != NULL)
  {
    myThreads.at(num)->close();
  }
}

void NodePanel::handleButton()
{
  myThreads.push_back(new MyThread());
  curThread++;
  QString cmd = "rosrun igvc " + output_topic_editor_->currentText();
  std::string stdcmd = cmd.toStdString();
  const char* strcmd = stdcmd.c_str();
  myThreads.at(curThread)->setCmd(strcmd);
  myThreads.at(curThread)->start();

  QHBoxLayout* topic_layout = new QHBoxLayout;

  test_btn = new NumButton("close", this, curThread);
  test_btn->setGeometry(QRect(QPoint(100, 100), QSize(200, 50)));
  topic_layout->addWidget(test_btn);

  connect(test_btn, SIGNAL(numClicked(int)), this, SLOT(closeNode(int)));

  layout->addLayout(topic_layout);
}

NodePanel::NodePanel(QWidget* parent) : rviz::Panel(parent)
{
  doOnce = true;
  start = time(0);
  curThread = -1;
  myThreads.reserve(10);

  QHBoxLayout* topic_layout = new QHBoxLayout;
  QDir recoredDir("../../devel/lib/igvc/");
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

  close_btn = new NumButton("Close", this);
  close_btn->setGeometry(QRect(QPoint(100, 100), QSize(200, 50)));
  topic_layout->addWidget(close_btn);

  connect(m_button, SIGNAL(released()), this, SLOT(handleButton()));

  layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  setLayout(layout);
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::NodePanel, rviz::Panel)
