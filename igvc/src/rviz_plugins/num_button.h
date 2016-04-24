#ifndef NUM_BUTTON_H
#define NUM_BUTTON_H

#include <ros/ros.h>
#include <QLabel>
#include <std_msgs/UInt8.h>

#include <rviz/panel.h>

#include <ctime>

#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>

namespace rviz_plugins
{

class NumButton: public QPushButton
{
  Q_OBJECT
  public:
    NumButton(const QString &text, QWidget *parent = 0, int n = 0);

  public Q_SLOTS:
    void emitNum();

  Q_SIGNALS:
    void numClicked(int num);

  private:
    int num;
  

};

}

#endif // NUM_BUTTON_H