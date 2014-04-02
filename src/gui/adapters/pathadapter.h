#ifndef PATHADAPTER_H
#define PATHADAPTER_H

#include <QWidget>

namespace Ui {
class PathAdapter;
}

class PathAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit PathAdapter(QWidget *parent = 0);
    ~PathAdapter();

private:
    Ui::PathAdapter *ui;

    void paintEvent(QPaintEvent *);
};

#endif // PATHADAPTER_H
