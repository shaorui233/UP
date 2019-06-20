#ifndef JPOS_CONTROLLER
#define JPOS_CONTROLLER

#include <RobotController.h>

class JPos_Controller:public RobotController{
  public:
    JPos_Controller():RobotController(){}
    virtual ~JPos_Controller(){}

    virtual void initializeController(){}
    virtual void runController();
    virtual void updateVisualization(){}
  protected:
};

#endif
