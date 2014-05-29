#ifndef MOTORBOARDADAPTER_H
#define MOTORBOARDADAPTER_H

#include <QWidget>
#include <hardware/actuators/motors/MotorDriver.hpp>

namespace Ui {
class MotorBoardAdapter;
}

class MotorBoardAdapter : public QWidget
{
    Q_OBJECT

protected:
    typedef std::pair<double,double> VelocityPair;
    typedef std::pair<VelocityPair,VelocityPair> VelocityPairs;

public:
    explicit MotorBoardAdapter(MotorDriver *driver, QWidget *parent = 0);
    ~MotorBoardAdapter();

public slots:
    void onNewCurrentVelocities(double left, double right);

private slots:
    void paintEvent(QPaintEvent *);

    void on_lineEdit_leftVel_returnPressed();

    void on_lineEdit_rightVel_returnPressed();

private:
    Ui::MotorBoardAdapter *ui;

    MotorDriver *_driver;

    std::vector<VelocityPairs> _data;
};

#endif // MOTORBOARDADAPTER_H
