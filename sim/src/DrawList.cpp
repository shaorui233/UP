/*! @file DrawList.cpp
 *  @brief Data structure to store robot model to be drawn.
 *
 *  Stores all the data (except for joint positions) for the robot.
 *  Knows how to load cheetah robots from file.
 */

#include "DrawList.h"

void DrawList::loadFiles() {
  std::vector<std::string> names = {"c3_body.obj", "mini_abad.obj", "c3_upper_link.obj", "c3_lower_link.obj"};//,"mini_body.obj", "mini_abad.obj", "mini_upper_link.obj", "mini_lower_link.obj"};
  objLoader::ObjLoader loader;
  for (const auto &name : names) {
    std::string filename = _baseFileName + name;
    _vertexData.emplace_back();
    _normalData.emplace_back();
    _colorData.emplace_back();
    loader.load(filename.c_str(), _vertexData.back(), _normalData.back());
    setSolidColor(_colorData.back(), _vertexData.back().size(), .2, .2, .2);
    _nUnique++;
  }
}
/*!
 * Load the cheetah 3 model and build the draw list.
 * Returns an index number that can later be used to update the position of the robot.
 */
size_t DrawList::loadCheetah3() {
  // cheetah 3 has 13 bodies, but only 4 unique objects:

  std::vector<std::string> names = {"c3_body.obj", "mini_abad.obj", "c3_upper_link.obj", "c3_lower_link.obj"};
  objLoader::ObjLoader loader;

  size_t i0 = 0;
  size_t j0 = _nTotal;
//  for (const auto &name : names) {
//    std::string filename = _baseFileName + name;
//    _vertexData.emplace_back();
//    _normalData.emplace_back();
//    _colorData.emplace_back();
//    loader.load(filename.c_str(), _vertexData.back(), _normalData.back());
//    setSolidColor(_colorData.back(), _vertexData.back().size(), .2, .2, .2);
//    _nUnique++;
//  }

  // set model offsets:
  QMatrix4x4 bodyOffset, abadOffset, lowerOffset, eye;
  eye.setToIdentity();
  QMatrix4x4 upperOffsets[2];

  // body
  bodyOffset.setToIdentity();
  bodyOffset.rotate(90,1,0,0);
  bodyOffset.rotate(90,0,0,1);


  // abad
  abadOffset.setToIdentity();

  // upper link
  upperOffsets[0].setToIdentity();
  upperOffsets[0].rotate(-180, 0, 1, 0);
  upperOffsets[0].translate(0, -.045f, 0);
  upperOffsets[0].rotate(-180, 0, 0, 1);

  upperOffsets[1].setToIdentity();
  upperOffsets[1].rotate(-180, 0, 1, 0);
  upperOffsets[1].translate(0, .045f, 0);

  lowerOffset.setToIdentity();
  lowerOffset.rotate(180,0,1,0);
  lowerOffset.translate(0, 0, 0);

  // add bodies
  _objectMap.push_back(i0 + 0);
  _modelOffsets.push_back(bodyOffset);
  _kinematicXform.push_back(eye);
  _nTotal++;

  for(int i = 0; i < 4; i++) {
    _objectMap.push_back(i0 + 1);
    _modelOffsets.push_back(abadOffset);
    _kinematicXform.push_back(eye);

    _objectMap.push_back(i0 + 2);
    _modelOffsets.push_back(upperOffsets[i%2]);
    _kinematicXform.push_back(eye);

    _objectMap.push_back(i0 + 3);
    _modelOffsets.push_back(lowerOffset);
    _kinematicXform.push_back(eye);
    _nTotal += 3;
  }

  printf("size of kinematicXform: %lu, j0: %lu\n", _kinematicXform.size(), j0);

  buildDrawList();
  return j0;
}

/*!
 * Load the mini cheetah model and builds the draw list.
 * Returns an index number that can later be used to update the position of the robot.
 */
size_t DrawList::loadMiniCheetah() {
  std::vector<std::string> names = {"mini_body.obj", "mini_abad.obj", "mini_upper_link.obj", "mini_lower_link.obj"};
  objLoader::ObjLoader loader;

  size_t i0 = _nUnique;
  size_t j0 = _nTotal;
  for(size_t i = 0; i < names.size(); i++) {
    std::string filename = _baseFileName + names[i];
    _vertexData.emplace_back();
    _normalData.emplace_back();
    _colorData.emplace_back();
    loader.load(filename.c_str(), _vertexData.back(), _normalData.back());
    setSolidColor(_colorData.back(), _vertexData.back().size(), .2, .2, .2);
    _nUnique++;
  }

  // set model offsets:
  QMatrix4x4 bodyOffset, upper, lower, eye;
  QMatrix4x4 abadOffsets[4];
  eye.setToIdentity();

  // body
  bodyOffset.setToIdentity();

  // abads (todo, check these)
  abadOffsets[0].setToIdentity(); // n
  abadOffsets[0].rotate(-90,0,0,1);
  abadOffsets[0].translate(0, -.0565f, 0);
  abadOffsets[0].rotate(180,0,1,0);

  abadOffsets[1].setToIdentity(); // p
  abadOffsets[1].rotate(-90,0,0,1);
  abadOffsets[1].translate(0, -.0565f, 0);
  abadOffsets[1].rotate(0,0,1,0);

  abadOffsets[2].setToIdentity(); // n
  abadOffsets[2].rotate(90,0,0,1);
  abadOffsets[2].translate(0, -.0565f, 0);
  abadOffsets[2].rotate(0,0,1,0);

  abadOffsets[3].setToIdentity(); // p
  abadOffsets[3].rotate(90,0,0,1);
  abadOffsets[3].translate(0, -.0565f, 0);
  abadOffsets[3].rotate(180,0,1,0);

  // upper
  upper.setToIdentity();
  upper.rotate(-90, 0, 1, 0);

  // lower
  lower.setToIdentity();
  lower.rotate(180, 0, 1, 0);

  _objectMap.push_back(i0 + 0);
  _modelOffsets.push_back(bodyOffset);
  _kinematicXform.push_back(eye);

  _objectMap.push_back(i0 + 1);
  _modelOffsets.push_back(abadOffsets[0]);
  _kinematicXform.push_back(eye);

  _objectMap.push_back(i0 + 2);
  _modelOffsets.push_back(upper);
  _kinematicXform.push_back(eye);

  _objectMap.push_back(i0 + 3);
  _modelOffsets.push_back(lower);
  _kinematicXform.push_back(eye);

  _nTotal += 4;
  buildDrawList();
  return j0;
}

/*!
 * Rebuilds the drawing list and sets the flag indicating that model data must be reloaded.
 */
void DrawList::buildDrawList() {

  _glVertexData.clear();
  _glColorData.clear();
  _glNormalData.clear();
  _glArrayOffsets.clear();

  for(size_t i = 0; i < _nUnique; i++) {
    _glArrayOffsets.push_back(_glVertexData.size());
    _glArraySizes.push_back(_vertexData.at(i).size());
    // add the data for the objects
    _glVertexData.insert(_glVertexData.end(), _vertexData.at(i).begin(), _vertexData.at(i).end());
    _glColorData.insert(_glColorData.end(), _colorData.at(i).begin(), _colorData.at(i).end());
    _glNormalData.insert(_glNormalData.end(), _normalData.at(i).begin(), _normalData.at(i).end());
  }

  printf("[Graphics 3D] Rebuilt draw list (%lu, %lu, %lu)\n", _glVertexData.size(), _glNormalData.size(), _glColorData.size());
  _reloadNeeded = true;
}