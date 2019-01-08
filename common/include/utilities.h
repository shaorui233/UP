#ifndef PROJECT_UTILITIES_H
#define PROJECT_UTILITIES_H

#include "cppTypes.h"
#include <vector>
#include <algorithm>

/*!
 * Are two floating point values almost equal?
 */
template<typename T>
bool fpEqual(T a, T b, T tol) {
  return std::abs(a - b) <= tol;
}

/*!
 * Are two std::vectors equal?
 */
template<typename T>
bool vectorEqual(const std::vector<T>& a, const std::vector<T>& b) {
  if(a.size() != b.size())
    return false;
  for(size_t i = 0; i < a.size(); i++) {
    if(a[i] != b[i])
      return false;
  }
  return true;
}

/*!
 * Coerce in to be between min and max
 */
template<typename T>
T coerce(T in, T min, T max) {
  if(in < min) {
    in = min;
  }
  if(in > max) {
    in = max;
  }
  return in;
}

/*!
 * Get the sign of a number
 * 1 for positive, 0 for 0, -1 for negative...
 */
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
#endif //PROJECT_UTILITIES_H
