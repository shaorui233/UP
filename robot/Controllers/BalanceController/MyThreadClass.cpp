#include "MyThreadClass.hpp"

bool MyThreadClass::StartInternalThread()
{
   return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
}

void MyThreadClass::WaitForInternalThreadToExit()
{
   (void) pthread_join(_thread, NULL);
}

void MyThreadClass::ExitThread()
{
   (void) pthread_exit(NULL);
}

