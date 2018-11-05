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

#include "obj_loader.h"
#include "FloatingBaseModel.h"
#include "DynamicsSimulator.h"
#include "sim_utilities.h"
#include "cppTypes.h"
#include "spatial.h"
#include "Colors.h"
#include "Checkerboard.h"
#include "CollisionPlane.h"

#include <QMatrix4x4>

#include <stdlib.h>
#include <vector>

class DrawList {
public:
  DrawList() {
    loadFiles();
  }
  size_t addCheetah3();
  size_t addMiniCheetah();
  void buildDrawList();
  void loadFiles();
  size_t addCheckerboard(Checkerboard& checkerBoard);
  size_t addDebugSphere(float radius);

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
   * Note that several objects may have the same geometry, so this will return the same value for these objects!
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

  /*!
   * Get the array containing all vertex data.  Use getGLDrawArrayOffset/Size to get indices and sizes for each object
   */
  float *getVertexArray() {
    return _glVertexData.data();
  }

  /*!
   * Get the array containing all normal data.  Use getGLDrawArrayOffset/Size to get indices and sizes for each object
   */
  float *getNormalArray() {
    return _glNormalData.data();
  }

  size_t getSizeOfAllData() {
    return _glVertexData.size();
  }

  /*!
   * Get the array containing all color data.
   */
  float *getColorArray() {
    return _glColorData.data();
  }

  /*!
   * Get the Qt transformation matrix which should be applied to the model geometry
   * This is to correct for errors when exporting parts and shifts them all to the origin.
   */
  QMatrix4x4 &getModelBaseTransform(size_t i) {
    return _modelOffsets.at(i);
  }

  /*!
   * Get the Qt transformation matrix which should be applied to move a model from the origin
   * to where it should be in the world
   */
  QMatrix4x4 &getModelKinematicTransform(size_t i) {
    return _kinematicXform.at(i);
  }

  /*!
   * Get size of data used by the GPU in megabytes
   * For debugging
   */
  float getGLDataSizeMB() {
    size_t bytes = _glColorData.size() + _glNormalData.size() + _glVertexData.size();
    bytes = bytes * sizeof(float);
    return (float) bytes / float(1 << 20);
  }

  /*!
   * Returns true a single time if we have changed geometries and need to reload.
   */
  bool needsReload() {
    if (_reloadNeeded) {
      _reloadNeeded = false;
      return true;
    }
    return false;
  }

  /*!
   * Update the position of a robot's bodies using the result of a dynamics simulation.
   * Doesn't run the simulator - just pulls _Xa from the DynamicsSimulator
   * @param model  : the simulator
   * @param id     : the id returned from the loadCheetah3 or loadMiniCheetah function.
   */
  template<typename T>
  void updateRobotFromModel(DynamicsSimulator<T> &model, size_t id) {
    for (size_t modelID = 5, graphicsID = id; modelID < model.getNumBodies(); modelID++, graphicsID++) {
      _kinematicXform.at(graphicsID) = spatialTransformToQT(model._Xa.at(modelID));
    }
  }

  /*!
   * Updates the position of a checkerboard to match an infinite collision plane
   * The infinite collision plane only specifies orientation, so we
   * @param model : the collision plane
   * @param id    : the id retured from creating the checkerboard
   */
  template<typename T>
  void updateCheckerboardFromCollisionPlane(CollisionPlane<T>& model, size_t id) {
    //Mat4<T> H = sxformToHomogeneous(model.getLocation());
    _kinematicXform.at(id) =  spatialTransformToQT(model.getLocation());
  }

  template<typename T>
  void updateDebugSphereLocation(Vec3<T>& position, size_t id) {
    QMatrix4x4 H;
    H.setToIdentity();
    H.translate(position[0], position[1], position[2]);
    _kinematicXform.at(id) = H;
  }

  /*!
   * Fill color data with a solid color
   */
  static void setSolidColor(std::vector<float> &data, size_t size, float r, float g, float b) {
    data.clear();
    data.resize(size);

    if ((size % 3) != 0) {
      throw std::runtime_error("setSolidColor invalid size");
    }

    for (size_t i = 0; i < size / 3; i++) {
      data[i * 3 + 0] = r;
      data[i * 3 + 1] = g;
      data[i * 3 + 2] = b;
    }
  }


private:
  size_t _nUnique = 0, _nTotal = 0;
  std::vector<std::vector<float>> _vertexData;
  std::vector<std::vector<float>> _normalData;
  std::vector<std::vector<float>> _colorData;
  vectorAligned<Mat4<float>> _offsetXforms; // these are NOT coordinate transformations!
  std::string _baseFileName = "../resources/";

  std::vector<size_t> _objectMap;

  std::vector<size_t> _glArrayOffsets;
  std::vector<size_t> _glArraySizes;

  std::vector<float> _glVertexData;
  std::vector<float> _glNormalData;
  std::vector<float> _glColorData;

  std::vector<QMatrix4x4> _modelOffsets;
  std::vector<QMatrix4x4> _kinematicXform;

  bool _reloadNeeded = true;

  size_t _cheetah3LoadIndex = 0, _miniCheetahLoadIndex = 0, _sphereLoadIndex = 0;

};


#endif //PROJECT_DRAWLIST_H
