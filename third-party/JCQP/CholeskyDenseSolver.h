#ifndef QPSOLVER_CHOLESKYDENSESOLVER_H
#define QPSOLVER_CHOLESKYDENSESOLVER_H

#include "types.h"
//#include "ThreadGroup.h"

template<typename T>
class CholeskyDenseSolver
{
public:
  CholeskyDenseSolver() = default;
  void setup(const DenseMatrix<T>& kktMat);
  void solve(Vector<T>& in);
  void solveAVX(Vector<T>& in);
  void setupAVX(T* mat, T* result, T* vec, u64 row);
  DenseMatrix<T>& getInternal() { return L; }
  DenseMatrix<T> getReconstructedPermuted();

private:
  DenseMatrix<T> L;

  T* solve1, *solve2;
  Vector<T> D;
  s64* pivots;
  s64 n = 0;
};


#endif //QPSOLVER_CHOLESKYDENSESOLVER_H
