#include "pathadapter.h"
#include "ui_pathadapter.h"
#include <QPainter>

PathAdapter::PathAdapter(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PathAdapter)
{
    ui->setupUi(this);
}

PathAdapter::~PathAdapter()
{
    delete ui;
}

void PathAdapter::paintEvent(QPaintEvent *)
{
    QPainter painter(this);



    painter.end();
}
