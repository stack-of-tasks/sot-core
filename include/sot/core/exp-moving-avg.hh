/*
 * Copyright 2018,
 * Julian Viereck
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_EXPMOVINGAVG_H__
#define __SOT_EXPMOVINGAVG_H__

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

class SOT_CORE_DLLAPI ExpMovingAvg : public Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

public:
  SignalPtr<dynamicgraph::Vector, int> updateSIN;
  SignalTimeDependent<int, int> refresherSINTERN;
  SignalTimeDependent<dynamicgraph::Vector, int> averageSOUT;

public:
  ExpMovingAvg(const std::string &n);
  virtual ~ExpMovingAvg(void);

  void setAlpha(const double &alpha_);

protected:
  dynamicgraph::Vector &update(dynamicgraph::Vector &res, const int &inTime);

  dynamicgraph::Vector average;

  double alpha;
  bool init;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_TRACER_H__ */
