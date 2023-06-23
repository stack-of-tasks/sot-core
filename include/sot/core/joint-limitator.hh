/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef SOT_FEATURE_JOINTLIMITS_HH
#define SOT_FEATURE_JOINTLIMITS_HH
// Matrix
#include <dynamic-graph/linear-algebra.h>

// SOT
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include <sot/core/exception-task.hh>

#if defined(WIN32)
#if defined(joint_limitator_EXPORTS)
#define SOTJOINTLIMITATOR_EXPORT __declspec(dllexport)
#else
#define SOTJOINTLIMITATOR_EXPORT __declspec(dllimport)
#endif
#else
#define SOTJOINTLIMITATOR_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/// \brief Filter control vector to avoid exceeding joint maximum values.
///
/// This must be plugged between the entity producing the command
/// (i.e. usually the sot) and the entity executing it (the device).
class SOTJOINTLIMITATOR_EXPORT JointLimitator : public dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  JointLimitator(const std::string &name);
  virtual ~JointLimitator() {}

  virtual dynamicgraph::Vector &computeControl(dynamicgraph::Vector &res,
                                               sigtime_t time);
  dynamicgraph::Vector &computeWidthJl(dynamicgraph::Vector &res,
                                       const sigtime_t &time);

  virtual void display(std::ostream &os) const;

  /// \name Signals
  /// \{
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> jointSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> upperJlSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> lowerJlSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> controlSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, sigtime_t>
      controlSOUT;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, sigtime_t>
      widthJlSINTERN;
  /// \}
};
}  // end of namespace sot.
}  // namespace dynamicgraph

#endif  //! SOT_FEATURE_JOINTLIMITS_HH
