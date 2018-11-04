/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes Eigen types, template types,
 *  aliases, ...
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <eigen3/Eigen/Dense>

#include <vector>

// Rotation Matrix
template<typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 Vector
template<typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template<typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 3x3 Matrix
template<typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 4x1 Vector
template<typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// Spatial Vector (6x1, all subspaces)
template<typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// Spatial Transform (6x6)
template<typename T>
using SXform = typename Eigen::Matrix<T, 6, 6>;

// 6x6 Matrix
template<typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 4x4 Matrix
template<typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// 10x1 Vector
template<typename T>
using MassProperties = typename Eigen::Matrix<T, 10, 1>;

// Dynamically sized vector
template<typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template<typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// std::vector (a list) of Eigen things
template<typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;

#endif //PROJECT_CPPTYPES_H
