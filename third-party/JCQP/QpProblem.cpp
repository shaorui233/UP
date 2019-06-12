#include <iostream>
#include "QpProblem.h"
#include "Timer.h"
#include <vector>

/*!
 * Cold start solution by setting x,y,z to zero
 */
template<typename T>
void QpProblem<T>::coldStart()
{
  _x = &_x0;
  _z = &_z0;
  _xPrev = &_x1;
  _zPrev = &_z1;

  _x->setZero();
  _xPrev->setZero();
  _z->setZero();
  _zPrev->setZero();

  _y.setZero();
}

/*!
 * Solve QP
 */
template<typename T>
void QpProblem<T>::run(s64 nIterations, bool sparse)
{
  _sparse = sparse;
  // print info
  printf("n: %ld\nm: %ld\n, sz %ld", n, m, sizeof(T));
  settings.print();

  // init variables
  if(!_hotStarted)
    coldStart();

  // setup constraints and KKT
  computeConstraintInfos();
  setupLinearSolverCommon(); // do not include setup time to build sparse matrix to be consistent

  if(sparse)
    Asparse = A.sparseView();

  Timer preSetupTimer;
  if(sparse) // don't include time to build sparse matrices
    _cholSparseSolver.preSetup(_kkt);


  printf("Pre-setup in %.3f ms\n", preSetupTimer.getMs());

  Timer totalTimer;
  Timer setupTimer;

  if(sparse)
    _cholSparseSolver.setup(_kkt); // set up this one first.
  else
    _cholDenseSolver.setup(_kkt);

  double setupTimeMs = setupTimer.getMs();
  printf("Setup in %.3f ms\n", setupTimeMs);

  std::vector<Vector<T>> solutionLog;
  double totalResidTime = 0;

  for(s64 iteration = 0; iteration < settings.maxIterations; iteration++) {

    Timer iterationTimer;
    stepSetup();

    solveLinearSystem();

    stepX();
    stepZ();
    stepY();

    if(!((iteration + 1) % 300)) {
      Timer residTimer;

      printf("Iteration %5ld: ", iteration + 1);
      T residual = calcAndDisplayResidual();

      if(residual < settings.terminate || iteration + 1 >= nIterations) {
        printf("\n\nTERMINATE at iteration %ld, total time %.3f ms (%.3f ms on setup, %.3f on termination checks), residual %g\n",
            iteration + 1, totalTimer.getMs(), setupTimeMs, totalResidTime, residual);
        fprintf(stderr, "%.3f, ", totalTimer.getMs());
        check();

        if(!solutionLog.empty()) {
          for(s64 i = 0; i < solutionLog.size() -1; i++) {
            Vector<T> diff = solutionLog[i] - solutionLog[i + 1];
            printf("DIFF %ld: %.3f\n", i, diff.norm());
          }
        }

        break;
      }

      printf(" took %.3f ms (%.3f on residual), %6.3f total\n", iterationTimer.getMs(), residTimer.getMs(), totalTimer.getMs());

      totalResidTime += residTimer.getMs();
    }
  }
}


/*!
 * Determine constraint types and pick rho for each constraint.
 */
template<typename T>
void QpProblem<T>::computeConstraintInfos()
{
  for(s64 i = 0; i < m; i++) {
    if(l(i) < -settings.infty || u(i) > settings.infty){
      _constraintInfos[i].type = ConstraintType::INFINITE;
      _constraintInfos[i].rho = settings.rhoInfty;
      printf("infty!!!\n\n\n");
    } else if(u(i) - l(i) < settings.eqlTol) {
      _constraintInfos[i].type = ConstraintType::EQUALITY;
      _constraintInfos[i].rho = settings.rho * settings.rhoEqualityScale;
      //printf("equal!!! %f %f\n\n\n", u(i), l(i));
    } else {
      _constraintInfos[i].type = ConstraintType::INEQUALITY;
      _constraintInfos[i].rho = settings.rho;
    }
    _constraintInfos[i].invRho = T(1) / _constraintInfos[i].rho;
  }
}

/*!
 * Perform one-time linear solver setup
 */
template<typename T>
void QpProblem<T>::setupLinearSolverDense()
{
  _kktSolver = Eigen::LDLT<DenseMatrix<T>>(_kkt);
  //std::cout << "DIAr:" << _kktSolver.vectorD().transpose() << "\n\n";

}

template<typename T>
void QpProblem<T>::setupLinearSolverSparse()
{
  _kktSparseSolver = new Eigen::SimplicialLDLT<Eigen::SparseMatrix<T>>();
  _kktSparseX.resize(m + n, 1);
  _kktSparse = _kkt.sparseView();
  _kktSparseSolver->compute(_kktSparse);
}

template<typename T>
void QpProblem<T>::setupCholeskyDenseSolver()
{

}

template<typename T>
void QpProblem<T>::setupLinearSolverCommon()
{
  // first, build KKT matrix
  _kkt.topLeftCorner(n,n) = P;
  _kkt.topRightCorner(n,m) = A.transpose();
  _kkt.bottomLeftCorner(m,n) = A;
  _kkt.bottomRightCorner(m,m).setZero();
  for(s64 i = 0; i < n; i++)
    _kkt(i,i) += settings.sigma;

  for(s64 i = 0; i < m; i++)
    _kkt(i + n, i + n) -= _constraintInfos[i].invRho;
}

template<typename T>
void QpProblem<T>::stepSetup()
{
  std::swap(_x, _xPrev);
  std::swap(_z, _zPrev);
}

template<typename T>
void QpProblem<T>::solveLinearSystem()
{
  // set up rhs
  for(s64 i = 0; i < n; i++)
    _xzTilde[i] = settings.sigma * (*_xPrev)[i] - q[i];

  for(s64 i = 0; i < m; i++)
    _xzTilde[i + n] = (*_zPrev)[i] - _constraintInfos[i].invRho * _y[i];

  if(_sparse)
    _cholSparseSolver.solve(_xzTilde);
  else
    _cholDenseSolver.solveAVX(_xzTilde);

  // step!
  for(s64 i = 0; i < m; i++)
    _xzTilde[i + n] = (*_zPrev)[i] + _constraintInfos[i].invRho * (_xzTilde[i + n] - _y[i]);
}

template<typename T>
void QpProblem<T>::solveLinearSystemSparse()
{
  // set up rhs
  for(s64 i = 0; i < n; i++)
    _kktSparseX[i] = settings.sigma * (*_xPrev)[i] - q[i];

  for(s64 i = 0; i < m; i++)
    _kktSparseX[i + n] = (*_zPrev)[i] - _constraintInfos[i].invRho * _y[i];


  _xzTilde = _kktSparseSolver->solve(_kktSparseX);


  // step!
  for(s64 i = 0; i < m; i++)
    _xzTilde[i + n] = (*_zPrev)[i] + _constraintInfos[i].invRho * (_xzTilde[i + n] - _y[i]);
}

template<typename T>
void QpProblem<T>::stepX()
{
  for(s64 i = 0; i < n; i++)
    (*_x)[i] = settings.alpha * _xzTilde[i] + (T(1) - settings.alpha) * (*_xPrev)[i];

  //std::cout << "x: " << _xPrev.transpose() << " -> " << _x.transpose() << "\n";
  _deltaX = (*_x) - (*_xPrev);
}

template<typename T>
void QpProblem<T>::stepZ()
{
  for(s64 i = 0; i < m; i++) {
    (*_z)[i] = settings.alpha * _xzTilde[i + n] + (T(1) - settings.alpha) * (*_zPrev)[i] + _constraintInfos[i].invRho * _y[i];

    // msvc is garbage and turns this into branches, todo investigate more
    if((*_z)[i] < l[i]) (*_z)[i] = l[i];
    if((*_z)[i] > u[i]) (*_z)[i] = u[i];
  }
}

template<typename T>
void QpProblem<T>::stepY()
{
  for(s64 i = 0; i < m; i++) {
    _deltaY[i] = _constraintInfos[i].rho * (settings.alpha * _xzTilde[i + n] + (T(1) - settings.alpha) * (*_zPrev)[i] - (*_z)[i]);
    _y[i] += _deltaY[i];
  }
}

template<typename T>
T QpProblem<T>::infNorm(const Vector<T>& v) {
  T m(0);
  for(u64 i = 0; i < v.rows(); i++) {
    if(std::abs(v[i]) > m) m = std::abs(v[i]);
  }
  return m;
}


template<typename T>
T QpProblem<T>::calcAndDisplayResidual()
{

  if(_sparse) {
    Vector<T> Axz = Asparse * (*_x) - (*_zPrev);
    T p = infNorm(Axz);

    Vector<T> dual = P * (*_x) + q + Asparse.transpose() * _y;
    T d = infNorm(dual);

    printf("p: %8.4f | d: %8.4f", p, d);
    return (d + p)/4;
  } else {
    Vector<T> Axz = A * (*_x) - (*_zPrev);
    T p = infNorm(Axz);

    Vector<T> dual = P * (*_x) + q + A.transpose() * _y;
    T d = infNorm(dual);

    printf("p: %8.4f | d: %8.4f", p, d);
    return (d + p)/4;
  }

}

template<typename T>
void QpProblem<T>::check()
{


//  std::cout << "x_: " << _x.transpose() << "\n";
//  std::cout << "x~: " << _xzTilde.topLeftCorner(n,1).transpose() << "\n";
//  std::cout << "z_: " << _z.transpose() << "\n";
//  std::cout << "z~: " << _xzTilde.bottomLeftCorner(m,1).transpose() << "\n";
//  Vector<T> Ax = A * _x;
//  std::cout << "Ax: " << Ax.transpose() << "\n";
  //std::cout << _x.transpose() << "\n";

  //  // choldense testing.
//  DenseMatrix<T> diff = _cholDenseSolver.getInternal() - _kktSolver.matrixLDLT();
//  printf("Diff %g to %g\n", diff.maxCoeff(), diff.minCoeff());
//  Vector<T> cholRes(n + m);
//  Vector<T> cholIn(n + m);
//  cholIn.setRandom();
//
//  Timer mySolverTime;
//  for(s64 i = 0; i < 1000; i++)
//  {
//    cholRes = cholIn;
//    _cholDenseSolver.solveAVX(cholRes);
//  }
//  printf("choldense: %.5f ms\n", mySolverTime.getMs());
//
//  Vector<T> prod = _kkt * cholRes;
//  Vector<T> diff2 = _kkt * cholRes - cholIn;
//  printf("diff2 %g to %g\n", diff2.maxCoeff(), diff2.minCoeff());
//
//  Timer eigenSolveTime;
//  for(s64 i = 0; i < 1000; i++)
//  {
//    Vector<T> ref = _kktSolver.solve(cholIn);
//  }
//  printf("eigen: %.5f ms\n", eigenSolveTime.getMs());
}


template class QpProblem<double>;
//template class QpProblem<float>;
