#include "batteryindicator.h"

BatteryIndicator::BatteryIndicator(QWidget *parent) :
    QProgressBar(parent)
{
    this->setMaximum(100);
    this->setValue(0);
}

void BatteryIndicator::onBatteryLevelChanged(int level)
{
    this->setValue(level);
}
