#ifndef CHEETAH_SOFTWARE_SPARSECMPC_H
#define CHEETAH_SOFTWARE_SPARSECMPC_H

#include "FootstepPlanner/GraphSearch.h"
#include "cppTypes.h"
#include "../../../third-party/JCQP/SparseMatrixMath.h"

struct BblockID {
  u32 foot;
  u32 timestep;
};

class SparseCMPC {
public:
  SparseCMPC();
  void run();


  // setup methods
  template<typename T>
  void setRobotParameters(Mat3<T>& inertia, T mass, T maxForce) {
    _Ibody = inertia.template cast<double>();
    _mass = mass;
    _maxForce = maxForce;
  }

  void setFriction(double mu) {
    _mu = mu;
  }

  template<typename T>
  void setWeights(Vec12<T>& weights, T alpha) {
    _weights = weights.template cast<double>();
    _alpha = alpha;
  }

  template<typename T>
  void setX0(Vec3<T> p, Vec3<T> v, Vec4<T> q, Vec3<T> w, T yaw, ContactState& contact0) {
    _p0 = p.template cast<double>();
    _v0 = v.template cast<double>();
    _q0 = q.template cast<double>();
    _w0 = w.template cast<double>();
    _yaw = yaw;
    _contact0 = contact0;
  }

  template<typename T>
  void setContactTrajectory(ContactState* contacts, std::size_t length) {
    _contactTrajectory.resize(length);
    for(std::size_t i = 0; i < length; i++) {
      _contactTrajectory[i] = contacts[i];
    }
  }

  template<typename T>
  void setStateTrajectory(vectorAligned<Vec12<double>>& traj) {
    _stateTrajectory = traj;
  }

  template<typename T>
  void setDtTrajectory(std::vector<T>& traj) {
    _dtTrajectory.clear();
    _dtTrajectory.reserve(traj.size());
    for(auto pt : traj)
      _dtTrajectory.push_back(pt);
  }

  template<typename T>
  void setFeet(Vec12<T>& feet) {
    _pFeet = feet.template cast<double>();
  }


private:
  void buildX0();
  void buildCT();
  void buildDT();
  void c2d(u32 trajIdx, u32 bBlockStartIdx, u32 block_count);


  u32 getStateIndex(u32 trajIdx);
  u32 getControlIndex(u32 bBlockIdx);
  u32 addConstraint(u32 size);
  void addConstraintTriple(double value, u32 row, u32 col);

  void addX0Constraint();

  // inputs
  Mat3<double> _Ibody;
  Vec12<double> _weights;
  double _mass, _maxForce, _mu, _alpha, _yaw;
  Vec3<double> _p0, _v0, _w0, _rpy0;
  Vec4<double> _q0;
  Vec12<double> _x0;
  Vec12<double> _pFeet, _g;
  ContactState _contact0;

  // input trajectories
  std::vector<ContactState> _contactTrajectory;
  vectorAligned<Vec12<double>> _stateTrajectory;
  std::vector<double> _dtTrajectory;

  // intermediates
  vectorAligned<Mat12<double>> _aMat;
  std::vector<BblockID> _bBlockIds;
  vectorAligned<Eigen::Matrix<double,12,3>> _bBlocks;
  std::vector<u32> _contactCounts;
  std::vector<u32> _runningContactCounts;

  std::vector<SparseTriple<double>> _constraintTriples, _costTriples;
  std::vector<double> _lb, _ub;




  u32 _trajectoryLength;
  u32 _bBlockCount;
  u32 _constraintCount;
};

#endif //CHEETAH_SOFTWARE_SPARSECMPC_H
