#ifndef LIGHTSHIELDADAPTER_H
#define LIGHTSHIELDADAPTER_H

#include <QWidget>
#include <hardware/actuators/lights/lightcontroller.h>

namespace Ui {
class LightShieldAdapter;
}

class LightShieldAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit LightShieldAdapter(LightController *controller, QWidget *parent = 0);
    ~LightShieldAdapter();

private slots:
    void on_checkBox_safetyLight_toggled(bool checked);

    void on_horizontalSlider_red_sliderMoved(int position);

    void on_horizontalSlider_green_sliderMoved(int position);

    void on_horizontalSlider_blue_sliderMoved(int position);

    void on_horizontalSlider_b1s1_sliderMoved(int position);

    void on_horizontalSlider_b1s2_sliderMoved(int position);

    void on_horizontalSlider_b1s3_sliderMoved(int position);

    void on_horizontalSlider_b2s1_sliderMoved(int position);

    void on_horizontalSlider_b2s2_sliderMoved(int position);

    void on_horizontalSlider_b2s3_sliderMoved(int position);

private:
    Ui::LightShieldAdapter *ui;

    LightController *_controller;
};

#endif // LIGHTSHIELDADAPTER_H
