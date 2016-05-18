#include <stdio.h>

#include <QHBoxLayout>
#include <QLineEdit>

#include <string>
#include <iostream>
#include <cstdio>
#include <memory>
#include <thread>

#include <QStringList>
#include <QDir>

#include "num_button.h"

namespace rviz_plugins
{

NumButton::NumButton( const QString &text, QWidget *parent, int n )
  : QPushButton( text, parent )
{
	num = n;
    connect(this, SIGNAL (clicked()), this, SLOT (emitNum()));
}

void NumButton::emitNum() {
	Q_EMIT numClicked(num);
}

}