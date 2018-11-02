/*! @file DrawList.cpp
 *  @brief Data structure to store robot model to be drawn.
 *
 *  Stores all the data (except for joint positions) for the robot.
 *  Knows how to load cheetah robots from file.
 */

#include "DrawList.h"

/*!
 * Load the cheetah 3 model and build the draw list.
 */
void DrawList::loadCheetah3() {
  // cheetah 3 has 13 bodies, but only 4 unique objects:

  std::vector<std::string> names = {"c3_body.obj", "mini_abad.obj", "c3_upper_link.obj", "c3_lower_link.obj"};
  //resize(names.size(), 4);
  objLoader::ObjLoader loader;

  size_t i0 = _nUnique;
  for(size_t i = 0; i < names.size(); i++) {
    std::string filename = _baseFileName + names[i];
    _vertexData.emplace_back();
    _normalData.emplace_back();
    _colorData.emplace_back();
    loader.load(filename.c_str(), _vertexData.back(), _normalData.back());
    setSolidColor(_colorData.back(), _vertexData.back().size(), .2, .2, .2);
    _nUnique++;
  }
  _objectMap.push_back(i0 + 0);
  _objectMap.push_back(i0 + 1);
  _objectMap.push_back(i0 + 2);
  _objectMap.push_back(i0 + 3);
  _nTotal += 4;

  buildDrawList();
}

void DrawList::loadMiniCheetah() {
  std::vector<std::string> names = {"mini_body.obj", "mini_abad.obj", "mini_upper_link.obj", "mini_lower_link.obj"};
  //resize(names.size(), 4);
  objLoader::ObjLoader loader;

  size_t i0 = _nUnique;
  for(size_t i = 0; i < names.size(); i++) {
    std::string filename = _baseFileName + names[i];
    _vertexData.emplace_back();
    _normalData.emplace_back();
    _colorData.emplace_back();
    loader.load(filename.c_str(), _vertexData.back(), _normalData.back());
    setSolidColor(_colorData.back(), _vertexData.back().size(), .2, .2, .2);
    _nUnique++;
  }
  _objectMap.push_back(i0 + 0);
  _objectMap.push_back(i0 + 1);
  _objectMap.push_back(i0 + 2);
  _objectMap.push_back(i0 + 3);
  _nTotal += 4;

  buildDrawList();
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