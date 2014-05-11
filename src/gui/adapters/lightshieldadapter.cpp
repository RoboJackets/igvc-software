#include "lightshieldadapter.h"
#include "ui_lightshieldadapter.h"

LightShieldAdapter::LightShieldAdapter(LightController *controller, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LightShieldAdapter)
{
    ui->setupUi(this);
    _controller = controller;
}

LightShieldAdapter::~LightShieldAdapter()
{
    delete ui;
}

void LightShieldAdapter::on_checkBox_safetyLight_toggled(bool checked)
{
    _controller->setSafetyLight(checked);
}

void LightShieldAdapter::on_horizontalSlider_red_sliderMoved(int position)
{
    _controller->setUnderglowColor(position, ui->horizontalSlider_green->value(), ui->horizontalSlider_blue->value());
}

void LightShieldAdapter::on_horizontalSlider_green_sliderMoved(int position)
{
    _controller->setUnderglowColor(ui->horizontalSlider_red->value(), position, ui->horizontalSlider_green->value());
}

void LightShieldAdapter::on_horizontalSlider_blue_sliderMoved(int position)
{
    _controller->setUnderglowColor(ui->horizontalSlider_red->value(), ui->horizontalSlider_green->value(), position);
}

void LightShieldAdapter::on_horizontalSlider_b1s1_sliderMoved(int position)
{
    _controller->setUnderglowBrightness(1, position, ui->horizontalSlider_b1s2->value(), ui->horizontalSlider_b1s3->value());
}

void LightShieldAdapter::on_horizontalSlider_b1s2_sliderMoved(int position)
{
    _controller->setUnderglowBrightness(1, ui->horizontalSlider_b1s1->value(), position, ui->horizontalSlider_b1s3->value());
}

void LightShieldAdapter::on_horizontalSlider_b1s3_sliderMoved(int position)
{
    _controller->setUnderglowBrightness(1, ui->horizontalSlider_b1s1->value(), ui->horizontalSlider_b1s2->value(), position);
}

void LightShieldAdapter::on_horizontalSlider_b2s1_sliderMoved(int position)
{
    _controller->setUnderglowBrightness(2, position, ui->horizontalSlider_b2s2->value(), ui->horizontalSlider_b2s3->value());
}

void LightShieldAdapter::on_horizontalSlider_b2s2_sliderMoved(int position)
{
    _controller->setUnderglowBrightness(2, ui->horizontalSlider_b2s1->value(), position, ui->horizontalSlider_b2s3->value());
}

void LightShieldAdapter::on_horizontalSlider_b2s3_sliderMoved(int position)
{
    _controller->setUnderglowBrightness(2, ui->horizontalSlider_b2s1->value(), ui->horizontalSlider_b2s2->value(), position);
}
