#ifndef BATTERYINDICATOR_H
#define BATTERYINDICATOR_H

#include <QProgressBar>

/*!
 * \brief Widget for displaying battery charge level.
 * \note Functionality is not yet implemented.
 */
class BatteryIndicator : public QProgressBar
{
    Q_OBJECT
public:
    explicit BatteryIndicator(QWidget *parent = 0);

signals:
    
public slots:
    void onBatteryLevelChanged(int level);
    
};

#endif // BATTERYINDICATOR_H
