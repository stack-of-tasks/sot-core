/*
 * Copyright 2013,
 * Olivier Stasse,
 *
 * CNRS
 *
 */

#ifndef SOT_JOINT_TRAJECTORY_ENTITY_HH
#define SOT_JOINT_TRAJECTORY_ENTITY_HH

#include <deque>
#include <list>

// Maal
#include <dynamic-graph/linear-algebra.h>

// SOT
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include <sot/core/matrix-geometry.hh>
#include <sot/core/trajectory.hh>
#include <sstream>
// API

#if defined(WIN32)
#if defined(joint_trajectory_entity_EXPORTS)
#define SOTJOINT_TRAJECTORY_ENTITY_EXPORT __declspec(dllexport)
#else
#define SOTJOINT_TRAJECTORY_ENTITY_EXPORT __declspec(dllimport)
#endif
#else
#define SOTJOINT_TRAJECTORY_ENTITY_EXPORT
#endif

// Class

namespace dynamicgraph {
namespace sot {

/** \brief This object handles trajectory of quantities and publish them as
   signals.

 */

class SOTJOINT_TRAJECTORY_ENTITY_EXPORT SotJointTrajectoryEntity
    : public dynamicgraph::Entity {
 public:
  DYNAMIC_GRAPH_ENTITY_DECL();

  /// \brief Constructor
  SotJointTrajectoryEntity(const std::string &name);
  virtual ~SotJointTrajectoryEntity() {}

  void loadFile(const std::string &name);

  /// \brief Return the next pose for the legs.
  dynamicgraph::Vector &getNextPosition(dynamicgraph::Vector &pos,
                                        const sigtime_t &time);

  /// \brief Return the next com.
  dynamicgraph::Vector &getNextCoM(dynamicgraph::Vector &com,
                                   const sigtime_t &time);

  /// \brief Return the next cop.
  dynamicgraph::Vector &getNextCoP(dynamicgraph::Vector &cop,
                                   const sigtime_t &time);

  /// \brief Return the next waist.
  sot::MatrixHomogeneous &getNextWaist(sot::MatrixHomogeneous &waist,
                                       const sigtime_t &time);

  /// \brief Return the current seq identified of the current trajectory.
  std::size_t &getSeqId(std::size_t &seqid, const sigtime_t &time);

  /// \brief Convert a xyztheta vector into an homogeneous matrix
  sot::MatrixHomogeneous XYZThetaToMatrixHomogeneous(
      const dynamicgraph::Vector &xyztheta);

  /// \brief Perform one update of the signals.
  size_type &OneStepOfUpdate(size_type &dummy, const sigtime_t &time);

  /// @name Display
  /// @{
  virtual void display(std::ostream &os) const;
  SOTJOINT_TRAJECTORY_ENTITY_EXPORT
  friend std::ostream &operator<<(std::ostream &os,
                                  const SotJointTrajectoryEntity &r) {
    r.display(os);
    return os;
  }
  /// @}

 public:
  typedef size_type Dummy;

  /// @name Signals
  /// @{
  /// \brief Internal signal for synchronisation.
  dynamicgraph::SignalTimeDependent<size_type, sigtime_t> refresherSINTERN;

  /// \brief Internal signal to trigger one step of the algorithm.
  SignalTimeDependent<Dummy, sigtime_t> OneStepOfUpdateS;

  /// \brief Publish pose for each evaluation of the graph.
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, sigtime_t> positionSOUT;

  /// \brief Publish com for each evaluation of the graph.
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, sigtime_t> comSOUT;

  /// \brief Publish zmp for each evaluation of the graph.
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, sigtime_t> zmpSOUT;

  /// \brief Publish waist for each evaluation of the graph.
  dynamicgraph::SignalTimeDependent<sot::MatrixHomogeneous, sigtime_t> waistSOUT;

  /// \brief Publish ID of the trajectory currently realized.
  dynamicgraph::SignalTimeDependent<std::size_t, sigtime_t> seqIdSOUT;

  /// \brief Read a trajectory.
  dynamicgraph::SignalPtr<Trajectory, sigtime_t> trajectorySIN;
  ///@}

 protected:
  /// \brief Index on the point along the trajectory.
  std::deque<sot::Trajectory>::size_type index_;

  /// \brief Keep the starting time as an identifier of the trajector
  timestamp traj_timestamp_;

  /// \brief Store the pos;
  dynamicgraph::Vector pose_;

  /// \brief Store the center of mass.
  dynamicgraph::Vector com_;

  /// \brief Store the center of pressure ZMP.
  dynamicgraph::Vector cop_;

  /// \brief Store the waist position.
  sot::MatrixHomogeneous waist_;

  /// \brief Store the current seq identifier.
  std::size_t seqid_;

  /// \brief Initial state of the trajectory.
  sot::Trajectory init_traj_;

  /// \brief Queue of trajectories.
  std::deque<sot::Trajectory> deque_traj_;

  /// \brief Update the entity with the current point of the trajectory.
  void UpdatePoint(const JointTrajectoryPoint &aJTP);

  /// \brief Update the entity with the trajectory aTrajectory.
  void UpdateTrajectory(const Trajectory &aTrajectory);

  /// \brief Implements the parsing and the affectation of initial trajectory.
  void setInitTraj(const std::string &os);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // SOT_JOINT_TRAJECTORY_ENTITY_HH
