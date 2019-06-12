#ifndef QPSOLVER_QPPROBLEM_H
#define QPSOLVER_QPPROBLEM_H

#include <vector>
#include <eigen3/Eigen/SparseCholesky>
#include <eigen3/Eigen/Sparse>
#include "types.h"
#include "CholeskyDenseSolver.h"
#include "CholeskySparseSolver.h"


template<typename T>
struct QpProblemSettings {
  s64 scalingIterations = 10;
  s64 maxIterations = 1000000;
  T rho = 6;
  T sigma = 1e-6;
  T alpha = 1.6;

  // numerical hacks
  T infty = 1e10;
  T eqlTol = 1e-10;
  T rhoEqualityScale = 1e3;
  T rhoInfty = 1e-6;

  T terminate = 1e-3;

  void print()
  {
    printf("rho: %f\n"
           "sigma: %f\n"
           "alpha: %f\n", rho, sigma, alpha);
  }
};

enum class ConstraintType {
  INFINITE,
  INEQUALITY,
  EQUALITY
};

template<typename T>
struct ConstraintInfo {
  T rho;
  T invRho;
  ConstraintType type;
};

template<typename T>
class QpProblem
{
public:

    QpProblem(s64 n_, s64 m_) : A(m_,n_), P(n_,n_), 
    l(m_), u(m_), q(n_), 
    n(n_), m(m_), 
    _kkt(n_ + m_, n_ + m_),
    _xzTilde(n_ + m_), _y(m_),
    _x0(n_), _x1(n_), _z0(m_), _z1(m_),   
    _Ar(m_), _Pr(n_), _AtR(n_), 
    _deltaY(m_), _AtDeltaY(n_),
    _deltaX(n_), _PDeltaX(n_), _ADeltaX(m_) {
        _constraintInfos.resize(m);

    }

    void run(s64 nIterations = -1, bool sparse = false);

    Vector<T>& getSolution() { return *_x; }



  // public data
  QpProblemSettings<T> settings;
  DenseMatrix<T> A, P;
  Vector<T> l, u, q;
  s64 n, m;

  ~QpProblem() {
    delete _kktSparseSolver;
  }

private:
  void coldStart();
  void computeConstraintInfos();
  void setupLinearSolverDense();
  void setupLinearSolverSparse();
  void setupLinearSolverCommon();
  void setupCholeskyDenseSolver();
  void stepSetup();
  void solveLinearSystem();
  void solveLinearSystemSparse();
  void stepX();
  void stepZ();
  void stepY();
  T calcAndDisplayResidual();
  void check();
  bool isSolved();
  void stepV2();
  T infNorm(const Vector<T>& v);


  DenseMatrix<T> _kkt;
  SparseMatrix<T> _kktSparse;
  Vector<T> _kktSparseX;

  Eigen::LDLT<DenseMatrix<T>> _kktSolver;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<T>>* _kktSparseSolver = nullptr;
  CholeskyDenseSolver<T> _cholDenseSolver;
  CholeskySparseSolver<T> _cholSparseSolver;
  Eigen::SparseMatrix<T> Asparse;

  Vector<T> _xzTilde, _y;
  Vector<T> _x0, _x1, _z0, _z1;
  Vector<T> *_x, *_z, *_xPrev, *_zPrev;
  Vector<T> _Ar, _Pr, _AtR; // residuals
  Vector<T> _deltaY, _AtDeltaY, _deltaX, _PDeltaX, _ADeltaX; // infeasibilities
  std::vector<ConstraintInfo<T>> _constraintInfos;

  bool _hotStarted = false, _sparse = false;
};


#endif //QPSOLVER_QPPROBLEM_H
