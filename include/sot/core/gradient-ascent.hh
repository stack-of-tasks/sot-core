/*
 * Copyright 2018,
 * Julian Viereck
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_GRADIENTASCENT_H__
#define __SOT_GRADIENTASCENT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <sot/core/config.hh>

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- TRACER ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

using dynamicgraph::Entity;
using dynamicgraph::SignalPtr;
using dynamicgraph::SignalTimeDependent;

class SOT_CORE_DLLAPI GradientAscent : public Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  SignalPtr<dynamicgraph::Vector, sigtime_t> gradientSIN;
  SignalPtr<double, sigtime_t> learningRateSIN;
  SignalTimeDependent<size_type, sigtime_t> refresherSINTERN;
  SignalTimeDependent<dynamicgraph::Vector, sigtime_t> valueSOUT;

 public:
  GradientAscent(const std::string &n);
  virtual ~GradientAscent(void);

 protected:
  dynamicgraph::Vector &update(dynamicgraph::Vector &res,
                               const sigtime_t &inTime);

  dynamicgraph::Vector value;

  double alpha;
  bool init;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_TRACER_H__ */
