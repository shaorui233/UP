#ifndef SAFETY_CHECKER_H
#define SAFETY_CHECKER_H

#include <iostream>

// Contains all of the control related data
#include "ControlFSMData.h"


/**
 * The SafetyChecker handles the
 */
template <typename T>
class SafetyChecker {
public:
  SafetyChecker(ControlFSMData<T>* dataIn): data(dataIn) { };
  //
  bool safetyPreCheck();

  //
  bool safetyPostCheck();

  // Pre checks to make sure controls are safe to run
  void checkSafeOrientation();	// robot's orientation is safe to control

  // Post checks to make sure controls can be sent to robot
  void checkPDesFoot();				// desired foot position is not too far
  void checkForceFeedForward();		// desired feedforward forces are not too large

  ControlFSMData<T>* data;

private:

};



#endif // SAFETY_CHECKER_H