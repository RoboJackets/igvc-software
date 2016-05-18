#include "my_thread.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <memory>
#include <thread>

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <cstring>

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
	std::strcpy (cmd, command);
}

void MyThread::close()
{
	proc->close();
}