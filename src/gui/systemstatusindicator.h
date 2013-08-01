#ifndef SYSTEMSTATUSINDICATOR_H
#define SYSTEMSTATUSINDICATOR_H

#include <QWidget>

class SystemStatusIndicator : public QWidget
{
    Q_OBJECT
public:
    explicit SystemStatusIndicator(QWidget *parent = 0);
    
protected:
    void paintEvent(QPaintEvent *);

signals:
    
public slots:
    
};

#endif // SYSTEMSTATUSINDICATOR_H
