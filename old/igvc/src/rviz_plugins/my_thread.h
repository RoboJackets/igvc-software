#include <QThread>

#include <QProcess>

class MyThread : public QThread
{
  Q_OBJECT

protected:
  void run();

public:
  void setCmd(const char*);
  void close();

public Q_SLOTS:

protected Q_SLOTS:

protected:
  char cmd[100];
  QProcess* proc;
};