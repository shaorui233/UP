#include "SparseCMPC/SparseCMPC.h"
#include "Math/orientation_tools.h"


// X0 != x[0]
// X0 uses u[0] and dt[0] to get to x[0]
// so same number of steps in u, dt, x

// 0 - roll
// 1 - pitch
// 2 - yaw
// 3 - x_pos
// 4 - y_pos
// 5 - z_pos
// 6 - roll_rate
// 7 - pitch_rate
// 8 - yaw_rate
// 9 - x_vel
// 10- y_vel
// 11- z_vel

SparseCMPC::SparseCMPC() : _contact0(false, false, false, false) {

}




void SparseCMPC::run() {
  // check trajectory length
  if((_stateTrajectory.size() != _contactTrajectory.size()) || (_contactTrajectory.size() != _dtTrajectory.size())) {
    throw std::runtime_error("SparseCMPC trajectory length error!");
  }

  _g.setZero();
  _g[11] = -9.81;
  _constraintCount = 0;
  _trajectoryLength = _stateTrajectory.size();
  _constraintTriples.clear();
  _costTriples.clear();
  _ub.clear();
  _lb.clear();
  _contactCounts.clear();
  _runningContactCounts.clear();

  // build initial state and data
  buildX0();
  buildCT();
  buildDT();

  // build optimization problem
  addX0Constraint();
  // addDynamicsConstraints();
  // addForceConstraints();
  // addCost();
  // runSolver();
}

/*!
 * Configure initial state
 */
void SparseCMPC::buildX0() {
  _rpy0 = ori::quatToRPY(_q0);
  _x0 << _rpy0, _p0, _w0, _v0;
}

/*!
 * Build continuous time matrices
 */
void SparseCMPC::buildCT() {
  // one 12x12 A matrix per timestep
  _aMat.clear();
  _aMat.resize(_trajectoryLength);

  // B "blocks" are for a single foot for a single iteration (so 12x3)
  _bBlockIds.clear();
  _bBlocks.clear();

  // loop over trajectory
  for(u32 i = 0; i < _trajectoryLength; i++) {
    // rotation from world into yaw frame
    Mat3<double> Ryaw = ori::coordinateRotation(ori::CoordinateAxis::Z, _stateTrajectory[i][2]); // todo check

    // transform inertia to world and invert
    Mat3<double> Iworld = Ryaw.transpose() * _Ibody * Ryaw;
    Mat3<double> Iinv = Iworld.inverse();

    // build A matrix (continuous time)
    Mat12<double>& A = _aMat[i];
    A.setZero();
    A(3,9) = 1; // x position integration
    A(4,10) = 1; // y position integration
    A(5,11) = 1; // z position integration
    A.block(0,6,3,3) = Ryaw; // omega integration

    // build up to four B blocks for the four feet
    auto& contactState = _contactTrajectory[i];
    for(uint32_t foot = 0; foot < 4; foot++) {
      if(contactState.contact[foot]) { // if foot is touching ground, add it.
        _bBlocks.emplace_back();
        _bBlockIds.push_back({foot, i});
        auto& B = _bBlocks.back();
        Vec3<double> pFoot = _pFeet.block(foot*3,0,3,1);

        B.setZero();
        B.block(6,0,3,3) = Iinv * ori::crossMatrix(pFoot);    // r x f torque
        B.block(9,0,3,3) = Mat3<double>::Identity() / _mass;  // f = ma
      }
    }
  }
  _bBlockCount = _bBlockIds.size();
}

/*!
 * Build discrete time matrices
 */
void SparseCMPC::buildDT() {
  u32 runningContactCount = 0;
  for(u32 i = 0; i < _trajectoryLength; i++) {
    u32 contactCount = 0;
    for(auto contact : _contactTrajectory[i].contact) {
      if(contact) contactCount++;
    }

    _contactCounts.push_back(contactCount);
    _runningContactCounts.push_back(runningContactCount);
    c2d(i, runningContactCount, contactCount);
    runningContactCount += contactCount;
  }
}


// for now, all xs occur before all us
u32 SparseCMPC::getStateIndex(u32 trajIdx) {
  return trajIdx * 12;
}

u32 SparseCMPC::getControlIndex(u32 bBlockIdx) {
  return (_trajectoryLength * 12) + (bBlockIdx * 3);
}

u32 SparseCMPC::addConstraint(u32 size) {
  u32 rv = _constraintCount;
  _constraintCount += size;
  return rv;
}

void SparseCMPC::addConstraintTriple(double value, u32 row, u32 col) {
  if(value != 0) {
    _constraintTriples.push_back({value, row, col});
  }
}


void SparseCMPC::addX0Constraint() {
  // x[0] = A[0] * X0 + B[0] * u[0] + g*dt;
  // x[0] - (B[0] * u[0]) = A[0]*X0 + g*dt;

  // select x[0] with identity matrix
  u32 state_idx = getStateIndex(0);
  u32 constraint_idx = addConstraint(12);
  for(u32 i = 0; i < 12; i++) { // diagonal of the identity
    _constraintTriples.push_back({1,constraint_idx + i, state_idx + i});
  }

  if(_contactCounts[0] || _runningContactCounts[0]) throw std::runtime_error("contact count error!");

  u32 ctrl_cnt = _contactCounts[0];

  for(u32 i = 0; i < ctrl_cnt; i++) { // Bblocks within this b (contact feet)
    // select -B[0]*u[0]
    u32 bbIdx = _runningContactCounts[0];
    for(u32 ax = 0; ax < 3; ax++) { // columns within the b block (forces axes)
      for(u32 row = 0; row < 12; row++) { // rows within the b block
        addConstraintTriple(-_bBlocks[bbIdx](row, ax), row, getControlIndex(bbIdx) + ax);
      }
    }
  }

  // compute right hand side A[0]*X0 + g*dt.
  Vec12<double> rhs = _aMat[0] * _x0 + _g * _dtTrajectory[0];

  // add to problem.
  for(u32 i = 0; i < 12; i++) {
    _ub.push_back(rhs[i]);
    _lb.push_back(rhs[i]);
  }

}