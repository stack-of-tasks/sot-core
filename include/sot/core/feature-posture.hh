/*
 * Copyright 2010,
 * Florent Lamiraux
 * Thomas Moulard,
 *
 * CNRS/AIST
 *
 */

#ifndef SOT_CORE_FEATURE_POSTURE_HH
#define SOT_CORE_FEATURE_POSTURE_HH

#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/value.h>

#include "sot/core/api.hh"
#include "sot/core/feature-abstract.hh"
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(feature_posture_EXPORTS)
#define SOTFEATUREPOSTURE_EXPORT __declspec(dllexport)
#else
#define SOTFEATUREPOSTURE_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFEATUREPOSTURE_EXPORT
#endif

namespace dynamicgraph {
namespace sot {
using command::Command;
using command::Value;

/*! @class dynamicgraph::sot::FeaturePosture feature-posture.hh
 * Feature that observes the posture of the robot, ie whose Jacobian is the
 * identity, or slices of the identity. This feature can be exactly
 * obtained with a generic posture, given the identity matrix as the input
 * Jacobian, the identity matrix. It is even prefereable, as the reference
 * value is then given by a signal, which can be reevalutated at each
 * iteration, for example to track a reference trajectory in the
 * configuration space. See for example the toFlag python function in the
 * sot-dyninv module to nicely selec the posture DOF.
 */

class SOTFEATUREPOSTURE_EXPORT FeaturePosture : public FeatureAbstract {
  class SelectDof;
  friend class SelectDof;

  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  typedef dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> signalIn_t;
  typedef dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, sigtime_t>
      signalOut_t;

  DECLARE_NO_REFERENCE;

  explicit FeaturePosture(const std::string &name);
  virtual ~FeaturePosture();
  virtual size_type &getDimension(size_type &res, sigtime_t);
  void selectDof(std::size_t dofId, bool control);

 protected:
  virtual dynamicgraph::Vector &computeError(dynamicgraph::Vector &res,
                                             sigtime_t);
  virtual dynamicgraph::Matrix &computeJacobian(dynamicgraph::Matrix &res,
                                                sigtime_t);
  virtual dynamicgraph::Vector &computeErrorDot(dynamicgraph::Vector &res,
                                                sigtime_t time);

  signalIn_t state_;
  signalIn_t posture_;
  signalIn_t postureDot_;
  signalOut_t error_;

 private:
  std::vector<bool> activeDofs_;
  std::size_t nbActiveDofs_;
};  // class FeaturePosture
}  // namespace sot
}  // namespace dynamicgraph

#endif  // SOT_CORE_FEATURE_POSTURE_HH
