#ifndef THREAD_HPP
#define THREAD_HPP

#include <pthread.h>
#include <unistd.h>

/**
 * The thread class provides threading support to c++ classes.<br />
 * A class should inherit from the Thread class and then can be run 
 * as if it were a thread using the <code>Start<code> method. <br />
 * 
 * Note: For now this only works in linux.
 */
class Thread {	
public:
	/** Starts the thread execution. */
	void Start();

	/**
	 * Thread safe sleep method. Suspends thread execution for  
	 * the specified number of microseconds.
	 * @param usec number of microseconds to sleep for
	 */
	static void Sleep(unsigned int usec) { usleep(usec); };

	/** Wait for the thread to terminate.
	 */
	void Join();
protected:
	Thread() {};
	virtual ~Thread() {};
			
	/**
	 * The code for the thread execution. <br />
	 * When this method returns, the thread has finished executing
	 */
	virtual void Run() = 0;
			
private:
	/** Protects against copy constructor */
	Thread(Thread&);
			
	/**
	 * Static run used to call the run of the child class<br />
	 * This is needed to make pthread_create happy
	 * @param obj is the pointer to the object that needs to be run
	 */
	static void* RunWrapper(void *obj);
			
	/** Prevent copying of the thread object */
	Thread& operator= (const Thread&);
		
	pthread_t _thread;
};

#endif /* THREAD_HPP */

