/*
 * Copyright 2013,
 * Olivier Stasse,
 *
 * CNRS, LAAS
 *
 */
//#define VP_DEBUG
//#define VP_DEBUG_MODE 10
#include <sot/core/debug.hh>
#include <sot/core/matrix-geometry.hh>
#ifdef VP_DEBUG
class sotJTE__INIT {
public:
  sotJTE__INIT(void) { dynamicgraph::sot::DebugTrace::openFile(); }
};
sotJTE__INIT sotJTE_initiator;
#endif //#ifdef VP_DEBUG

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/factory.h>

#include <sot/core/joint-trajectory-entity.hh>

#include "joint-trajectory-command.hh"

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;
using namespace dynamicgraph::command;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SotJointTrajectoryEntity,
                                   "SotJointTrajectoryEntity");

SotJointTrajectoryEntity::SotJointTrajectoryEntity(const std::string &n)
    : Entity(n), refresherSINTERN("SotJointTrajectoryEntity(" + n +
                                  ")::intern(dummy)::refresher"),
      OneStepOfUpdateS(
          boost::bind(&SotJointTrajectoryEntity::OneStepOfUpdate, this, _1, _2),
          refresherSINTERN << trajectorySIN,
          "SotJointTrajectory(" + n + ")::onestepofupdate"),
      positionSOUT(
          boost::bind(&SotJointTrajectoryEntity::getNextPosition, this, _1, _2),
          OneStepOfUpdateS,
          "SotJointTrajectory(" + n + ")::output(vector)::position"),
      comSOUT(boost::bind(&SotJointTrajectoryEntity::getNextCoM, this, _1, _2),
              OneStepOfUpdateS,
              "SotJointTrajectory(" + n + ")::output(vector)::com"),
      zmpSOUT(boost::bind(&SotJointTrajectoryEntity::getNextCoP, this, _1, _2),
              OneStepOfUpdateS,
              "SotJointTrajectory(" + n + ")::output(vector)::zmp"),
      waistSOUT(
          boost::bind(&SotJointTrajectoryEntity::getNextWaist, this, _1, _2),
          OneStepOfUpdateS,
          "SotJointTrajectory(" + n + ")::output(MatrixHomogeneous)::waist"),
      seqIdSOUT(boost::bind(&SotJointTrajectoryEntity::getSeqId, this, _1, _2),
                OneStepOfUpdateS,
                "SotJointTrajectory(" + n + ")::output(uint)::seqid"),
      trajectorySIN(NULL, "SotJointTrajectory(" + n +
                              ")::input(trajectory)::trajectoryIN"),
      index_(0), traj_timestamp_(0, 0), seqid_(0), deque_traj_(0) {
  using namespace command;
  sotDEBUGIN(5);

  signalRegistration(positionSOUT << comSOUT << zmpSOUT << waistSOUT
                                  << seqIdSOUT << trajectorySIN);
  refresherSINTERN.setDependencyType(TimeDependency<int>::ALWAYS_READY);
  refresherSINTERN.setReady(true);

  std::string docstring;
  docstring = "    \n"
              "    initialize the first trajectory.\n"
              "    \n"
              "      Input:\n"
              "        = a string : .\n"
              "    \n";
  addCommand("initTraj",
             makeCommandVoid1(*this, &SotJointTrajectoryEntity::setInitTraj,
                              docCommandVoid1("Set initial trajectory",
                                              "string (trajectory)")));
  sotDEBUGOUT(5);
}

void SotJointTrajectoryEntity::UpdatePoint(const JointTrajectoryPoint &aJTP) {

  sotDEBUGIN(5);
  // Posture
  std::vector<JointTrajectoryPoint>::size_type possize = aJTP.positions_.size();
  if (possize == 0)
    return;

  pose_.resize(aJTP.positions_.size());
  for (std::vector<JointTrajectoryPoint>::size_type i = 0; i < possize - 5;
       i++) {
    pose_(i) = aJTP.positions_[i];
    sotDEBUG(5) << pose_(i) << " " << std::endl;
  }

  // Center of Mass
  com_.resize(3);
  for (std::vector<JointTrajectoryPoint>::size_type i = possize - 5, j = 0;
       i < possize - 2; i++, j++)
    com_(j) = aJTP.positions_[i];

  sotDEBUG(5) << "com: " << com_ << std::endl;

  // Add a constant height TODO: make it variable
  dynamicgraph::Vector waistXYZTheta;
  waistXYZTheta.resize(4);

  waistXYZTheta(0) = com_(0);
  waistXYZTheta(1) = com_(1);
  waistXYZTheta(2) = 0.65;
  waistXYZTheta(3) = com_(2);
  waist_ = XYZThetaToMatrixHomogeneous(waistXYZTheta);

  sotDEBUG(5) << "waist: " << waist_ << std::endl;
  // Center of Pressure
  cop_.resize(3);
  for (std::vector<JointTrajectoryPoint>::size_type i = possize - 2, j = 0;
       i < possize; i++, j++)
    cop_(j) = aJTP.positions_[i];
  cop_(2) = -0.055;
  sotDEBUG(5) << "cop_: " << cop_ << std::endl;
  sotDEBUGOUT(5);
}

void SotJointTrajectoryEntity::UpdateTrajectory(const Trajectory &aTrajectory) {
  sotDEBUGIN(3);
  sotDEBUG(3) << "traj_timestamp: " << traj_timestamp_
              << " aTrajectory.header_.stamp_" << aTrajectory.header_.stamp_;

  // Do we have the same trajectory, or are we at the initialization phase ?
  if ((traj_timestamp_ == aTrajectory.header_.stamp_) &&
      (deque_traj_.size() != 0))
    index_++;
  else {
    // No we have a new trajectory.
    sotDEBUG(3) << "index: " << index_ << " aTrajectory.points_.size(): "
                << aTrajectory.points_.size();

    // Put the new trajectory in the queue

    // if there is nothing
    if (deque_traj_.size() == 0) {
      deque_traj_.push_back(aTrajectory);
      index_ = 0;
    } else {
      index_++;
      // if it is not already inside the queue.
      if (deque_traj_.back().header_.stamp_ == aTrajectory.header_.stamp_) {
      } else
        deque_traj_.push_back(aTrajectory);
    }
  }

  sotDEBUG(3) << "step 1 " << std::endl
              << "header: " << std::endl
              << "  timestamp:"
              << static_cast<double>(aTrajectory.header_.stamp_.secs_) +
                     0.000000001 *
                         static_cast<double>(aTrajectory.header_.stamp_.nsecs_)
              << " seq:" << aTrajectory.header_.seq_ << " "
              << " frame_id:" << aTrajectory.header_.frame_id_
              << " index_: " << index_
              << " aTrajectory.points_.size():" << aTrajectory.points_.size()
              << std::endl;

  // Strategy at the end of the trajectory.
  if (index_ >= deque_traj_.front().points_.size()) {

    if (deque_traj_.size() > 1) {
      deque_traj_.pop_front();
      index_ = 0;
    }

    // If the new trajectory has a problem
    if (deque_traj_.front().points_.size() == 0) {
      // then neutralize the entity
      index_ = 0;
      sotDEBUG(3) << "current_traj_.points_.size()="
                  << deque_traj_.front().points_.size() << std::endl;
      return;
    }

    // Strategy at the end of the trajectory when no new information is
    // available: It is assumed that the last pose is balanced, and we keep
    // providing this pose to the robot.
    if ((index_ != 0) && (deque_traj_.size() == 1)) {
      index_ = deque_traj_.front().points_.size() - 1;
    }
    sotDEBUG(3) << "index_=current_traj_.points_.size()-1;" << std::endl;
  }

  sotDEBUG(3) << "index_:" << index_ << " current_traj_.points_.size():"
              << deque_traj_.front().points_.size() << std::endl;

  seqid_ = deque_traj_.front().header_.seq_;
  UpdatePoint(deque_traj_.front().points_[index_]);
  sotDEBUGOUT(3);
}

int &SotJointTrajectoryEntity::OneStepOfUpdate(int &dummy, const int &time) {
  sotDEBUGIN(4);
  const Trajectory &atraj = trajectorySIN(time);
  if ((atraj.header_.stamp_.secs_ !=
       deque_traj_.front().header_.stamp_.secs_) ||
      (atraj.header_.stamp_.nsecs_ !=
       deque_traj_.front().header_.stamp_.nsecs_)) {
    if (index_ < deque_traj_.front().points_.size() - 1) {
      sotDEBUG(4) << "Overwrite trajectory without completion." << index_ << " "
                  << deque_traj_.front().points_.size() << std::endl;
    }
  }
  sotDEBUG(4) << "Finished to read trajectorySIN" << std::endl;
  UpdateTrajectory(atraj);

  sotDEBUG(4) << "Finished to update trajectory" << std::endl;

  sotDEBUGOUT(4);
  return dummy;
}

sot::MatrixHomogeneous SotJointTrajectoryEntity::XYZThetaToMatrixHomogeneous(
    const dynamicgraph::Vector &xyztheta) {
  assert(xyztheta.size() == 4);
  dynamicgraph::Vector t(3);
  t(0) = xyztheta(0);
  t(1) = xyztheta(1);
  t(2) = xyztheta(2);
  Eigen::Affine3d trans;
  trans = Eigen::Translation3d(t);
  Eigen::Affine3d _Rd(Eigen::AngleAxisd(xyztheta(3), Eigen::Vector3d::UnitZ()));
  sot::MatrixHomogeneous res;
  res = _Rd * trans;
  return res;
}

dynamicgraph::Vector &
SotJointTrajectoryEntity::getNextPosition(dynamicgraph::Vector &pos,
                                          const int &time) {
  sotDEBUGIN(5);
  OneStepOfUpdateS(time);
  pos = pose_;
  sotDEBUG(5) << pos;
  sotDEBUGOUT(5);
  return pos;
}

dynamicgraph::Vector &
SotJointTrajectoryEntity::getNextCoM(dynamicgraph::Vector &com,
                                     const int &time) {
  sotDEBUGIN(5);
  OneStepOfUpdateS(time);
  com = com_;
  sotDEBUGOUT(5);
  return com;
}

dynamicgraph::Vector &
SotJointTrajectoryEntity::getNextCoP(dynamicgraph::Vector &cop,
                                     const int &time) {
  sotDEBUGIN(5);
  OneStepOfUpdateS(time);
  cop = cop_;
  sotDEBUGOUT(5);
  return cop;
}

sot::MatrixHomogeneous &
SotJointTrajectoryEntity::getNextWaist(sot::MatrixHomogeneous &waist,
                                       const int &time) {
  sotDEBUGIN(5);
  OneStepOfUpdateS(time);
  waist = waist_;
  sotDEBUGOUT(5);
  return waist_;
}

unsigned int &SotJointTrajectoryEntity::getSeqId(unsigned int &seqid,
                                                 const int &time) {
  sotDEBUGIN(5);
  OneStepOfUpdateS(time);
  seqid = seqid_;
  sotDEBUGOUT(5);
  return seqid;
}

void SotJointTrajectoryEntity::loadFile(const std::string &) {
  sotDEBUGIN(5);
  // TODO
  sotDEBUGOUT(5);
}

void SotJointTrajectoryEntity::display(std::ostream &os) const {
  sotDEBUGIN(5);
  os << this;
  sotDEBUGOUT(5);
}

void SotJointTrajectoryEntity::setInitTraj(const std::string &as) {
  sotDEBUGIN(5);
  std::istringstream is(as);
  init_traj_.deserialize(is);
  UpdateTrajectory(init_traj_);

  sotDEBUGOUT(5);
}
