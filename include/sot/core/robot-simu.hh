/*
 * Copyright 2010,
 * Nicolas Mansard, Olivier Stasse, François Bleibel, Florent Lamiraux
 *
 * CNRS
 *
 */

#ifndef DYNAMICGRAPH_SOT_ROBOT_SIMU_HH
#define DYNAMICGRAPH_SOT_ROBOT_SIMU_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <pinocchio/fwd.hpp>

/* -- MaaL --- */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include "sot/core/api.hh"
#include "sot/core/device.hh"

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(robot_simu_EXPORTS)
#define SOT_ROBOT_SIMU_EXPORT __declspec(dllexport)
#else
#define SOT_ROBOT_SIMU_EXPORT __declspec(dllimport)
#endif
#else
#define SOT_ROBOT_SIMU_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOT_ROBOT_SIMU_EXPORT RobotSimu : public Device {
 public:
  RobotSimu(const std::string &inName);
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }
};
}  // namespace sot
}  // namespace dynamicgraph

#endif /* #ifndef DYNAMICGRAPH_SOT_ROBOT_SIMU_HH */
