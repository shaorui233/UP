#ifndef MYTHREADCLASS_H
#define MYTHREADCLASS_H

#include <pthread.h>

class MyThreadClass
{
public:
   MyThreadClass() {/* empty */}
   virtual ~MyThreadClass() {/* empty */}

   /** Returns true if the thread was successfully started, false if there was an error starting the thread */
   bool StartInternalThread();
   /** Will not return until the internal thread has exited. */
   void WaitForInternalThreadToExit();    
   void ExitThread();

protected:
   /** Implement this method in your subclass with the code you want your thread to run. */
   virtual void InternalThreadEntry() = 0;

private:
   static void * InternalThreadEntryFunc(void * This){((MyThreadClass *)This)->InternalThreadEntry(); return NULL;}

   pthread_t _thread;
};



#endif