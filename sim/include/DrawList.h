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

// background colors.
static constexpr float windows2000[] = {58.f / 256.f, 110.f / 256.f, 165.f / 256.f};
static constexpr float disgustingGreen[] = {0.f, 0.2f, 0.2f};

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


  /*!
   * Get the total number of objects to be drawn
   */
  size_t getNumObjectsToDraw() {
    return _nTotal;
  }

  /*!
   * For the i-th object, get the offset into the model data.
   * For use with the glDrawArrays function.
   */
  size_t getGLDrawArrayOffset(size_t i) {
    return _glArrayOffsets.at(_objectMap.at(i));
  }

  /*!
   * For the i-th object, get the size of the model data array
   */
  size_t getGLDrawArraySize(size_t i) {
    return _glArraySizes.at(_objectMap.at(i));
  }

  float* getVertexArray() {
    return _glVertexData.data();
  }

  float* getNormalArray() {
    return _glNormalData.data();
  }

  float* getColorArray() {
    return _glColorData.data();
  }

  /*!
   * Returns true a single time if we have changed geometries and need to reload.
   */
  bool needsReload() {
    if(_reloadNeeded) {
      _reloadNeeded = false;
      return true;
    }
    return false;
  }



  void buildDrawList();

  /*!
   * Fill color data with a solid color
   */
  static void setSolidColor(std::vector<float>& data, size_t size, float r, float g, float b) {
    data.clear();
    data.resize(size);

    if((size % 3) != 0) {
      throw std::runtime_error("setSolidColor invalid size");
    }

    for(size_t i = 0; i < size/3; i++) {
      data[i*3 + 0] = r;
      data[i*3 + 1] = g;
      data[i*3 + 2] = b;
    }
  }


private:
  size_t _nUnique = 0, _nTotal = 0;
  std::vector<std::vector<float>>   _vertexData;
  std::vector<std::vector<float>>   _normalData;
  std::vector<std::vector<float>>   _colorData;
  vectorAligned<Mat4<float>>        _offsetXforms; // these are NOT coordinate transformations!
  std::string                       _baseFileName = "../resources/";

  std::vector<size_t>               _objectMap;

  std::vector<size_t>               _glArrayOffsets;
  std::vector<size_t>               _glArraySizes;

  std::vector<float>               _glVertexData;
  std::vector<float>               _glNormalData;
  std::vector<float>               _glColorData;

  bool _reloadNeeded = true;

};


#endif //PROJECT_DRAWLIST_H
