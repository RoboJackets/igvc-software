#ifndef SYSTEMSTATUSINDICATOR_H
#define SYSTEMSTATUSINDICATOR_H

#include <QWidget>

/*!
 * \brief Widget for displaying iconic representation of overall system status
 * \note Functionality is not yet implemented.
 */
class SystemStatusIndicator : public QWidget
{
    Q_OBJECT
public:
    explicit SystemStatusIndicator(QWidget *parent = 0);
    
protected:
    void paintEvent(QPaintEvent *);

private:
    bool _isEnabled;
    
public slots:
    void onEStopStatusChanged(bool isEnabled);
    
};

#endif // SYSTEMSTATUSINDICATOR_H
