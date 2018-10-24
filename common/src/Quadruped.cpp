//
// Created by jared on 10/12/18.
//

#include "Quadruped.h"
#include <spatial.h>
#include <orientation_tools.h>
using namespace ori;
using namespace spatial;

/*!
 * Build a FloatingBaseModel of the quadruped
 */
template<typename T>
FloatingBaseModel<T> Quadruped<T>::buildModel() {
  FloatingBaseModel<T> model;

  // we assume the cheetah's body (not including rotors) can be modeled as a uniformly distributed box.
  Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);
  model.addBase(_bodyMass, Vec3<T>(0,0,0), rotInertiaOfBox(_bodyMass, bodyDims));

  // add contact for the cheetah's body
  model.addGroundContactBoxPoints(5, bodyDims);

  const int baseID = 5;
  int bodyID = baseID;
  //T sideSign = -1;

  for(int legID = 0; legID < 4; legID++) {
    // Ab/Ad joint
    bodyID++;


  }

  return model;
}


template class Quadruped<double>;
template class Quadruped<float>;