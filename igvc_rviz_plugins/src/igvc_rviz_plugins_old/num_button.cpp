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

#include <igvc_rviz_plugins_old/num_button.h>

namespace rviz_plugins
{
NumButton::NumButton(const QString &text, QWidget *parent, int n) : QPushButton(text, parent)
{
  num = n;
  connect(this, SIGNAL(clicked()), this, SLOT(emitNum()));
}

void NumButton::emitNum()
{
  Q_EMIT numClicked(num);
}
}  // namespace rviz_plugins
