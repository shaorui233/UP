#ifndef PROJECT_UTILITIES_H
#define PROJECT_UTILITIES_H

#include "cppTypes.h"
#include <vector>
#include <algorithm>
#include <random>
#include <unordered_map>

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

/*!
 * Fill an eigen type with random numbers from a random generator and uniform real distribution.
 * TODO: is there a way to make this work nicely with normal distributions too?
 */
template <typename T>
void fillEigenWithRandom(const Eigen::MatrixBase<T> &v, std::mt19937& gen, std::uniform_real_distribution<typename T::Scalar>& dist) {
  for(size_t i = 0; i < T::RowsAtCompileTime; i++) {
    for(size_t j = 0; j < T::ColsAtCompileTime; j++) {
      v(i,j) = dist(gen);
    }
  }
}

/*!
 * Does the unordered map contain the given element?
 */
template <typename T1, typename T2>
bool uMapContains(const std::unordered_map<T1, T2>& set, T1 key) {
  return set.find(key) != set.end();
}

/*!
 * Convert a floating point number to a string.  Is preferable over std::to_string
 * because this uses scientific notation and won't truncate small/large numbers.
 */
template <typename T>
std::string numberToString(T number) {
  static_assert(std::is_floating_point<T>::value, "numberToString must use a floating point type!");
  char buffer[100];
  sprintf(buffer, "%e", number);
  return std::string(buffer);
}

void writeStringToFile(const std::string& fileName, const std::string& fileData);
std::string getCurrentTimeAndDate();


#endif //PROJECT_UTILITIES_H
