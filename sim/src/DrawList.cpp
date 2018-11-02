/*! @file DrawList.cpp
 *  @brief Data structure to store robot model to be drawn.
 *
 *  Stores all the data (except for joint positions) for the robot.
 *  Knows how to load cheetah robots from file.
 */

#include "DrawList.h"

void DrawList::loadCheetah3() {
  // cheetah 3 has 13 bodies, but only 4 unique objects:

  std::vector<std::string> names = {"c3_body.obj", "mini_abad.obj", "c3_upper_link.obj", "c3_lower_link.obj"};
  resize(names.size(), 13);
  objLoader::ObjLoader loader;

  for(size_t i = 0; i < names.size(); i++) {
    std::string filename = _baseFileName + names[i];
    loader.load(filename.c_str(), _vertexData[i], _normalData[i]);
  }

}