#include <igvc_rviz_plugins_old/my_thread.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

using namespace std;

void MyThread::run()
{
  proc = new QProcess();
  std::string strCmd(cmd);
  std::string test = ("gnome-terminal -x bash -c \"" + strCmd + "\"");
  const char* cstrCmd = test.c_str();
  proc->start(cstrCmd);
  proc->waitForStarted();
}

void MyThread::setCmd(const char* command)
{
  std::strcpy(cmd, command);
}

void MyThread::close()
{
  proc->close();
}
