#ifndef TEST_H
#define TEST_H

#include "Controller.hpp"

template <typename T>
class Test{
public:
  Test();
  virtual ~Test();

  virtual void TestInitialization() = 0;
  void getCommand(void* _command);

  int getPhase(){ return phase_;}

protected:
  virtual int _NextPhase(const int & phase) = 0;

  bool b_first_visit_;
  int phase_;
  std::vector<Controller<T>*> state_list_;
};


#endif
