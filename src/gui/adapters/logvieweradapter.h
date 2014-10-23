#ifndef LOGVIEWERADAPTER_H
#define LOGVIEWERADAPTER_H

#include <QWidget>
#include "common/logger/customhighlighter.h"

namespace Ui {
class LogViewerAdapter;
}

class LogViewerAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit LogViewerAdapter(QWidget *parent = 0);
    ~LogViewerAdapter();

private slots:
    void on_btn_open_clicked();

private:
    Ui::LogViewerAdapter *ui;
    CustomHighlighter *highlighter;

};

#endif // LOGVIEWERADAPTER_H
