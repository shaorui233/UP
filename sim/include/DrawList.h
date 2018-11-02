/*! @file DrawList.h
 *  @brief Data structure to store robot model to be drawn.
 *
 *  Stores all the data (except for joint positions) for the robot.
 *  Knows how to load cheetah robots from file.
 *  Will need to support adding a variable number of items (for loading terrain which needs to be drawn)
 *
 *  Also supports having duplicated objects
 */

#ifndef PROJECT_DRAWLIST_H
#define PROJECT_DRAWLIST_H

#include <stdlib.h>
#include <vector>
#include <obj_loader.h>

#include <cppTypes.h>
#include <spatial.h>


class DrawList {
public:
  void loadCheetah3();
  void loadMiniCheetah();

  // add object

  size_t getNumDrawables();

  /*!
   * Resize to hold size objects.
   */
  void resize(size_t nUniqueObject, size_t nTotalObjects) {
    _nUnique = nUniqueObject;
    _nTotal = nTotalObjects;
    _vertexData.resize(nUniqueObject);
    _normalData.resize(nUniqueObject);
    _colorData.resize(nUniqueObject);
    _offsetXforms.resize(nUniqueObject);
    _objectMap.resize(nTotalObjects);
  }

  void buildDrawList();

private:
  size_t _nUnique = 0, _nTotal;
  std::vector<std::vector<float>>   _vertexData;
  std::vector<std::vector<float>>   _normalData;
  std::vector<std::vector<float>>   _colorData;
  vectorAligned<Mat4<float>>        _offsetXforms; // these are NOT coordinate transformations!
  std::string                       _baseFileName = "../resources/";
  std::vector<size_t>               _objectMap;
};


#endif //PROJECT_DRAWLIST_H
