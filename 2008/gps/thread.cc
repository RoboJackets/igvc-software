#include "thread.h"

void Thread::Start() {
	// Create the thread with default attributes
	if (pthread_create(&_thread, NULL, Thread::RunWrapper, this)) {
		//failed
		return;
	}
}

void* Thread::RunWrapper(void *obj) {
	((Thread *)obj)->Run();

	return NULL;
}

void Thread::Join() {
	pthread_join(_thread, NULL);
}

