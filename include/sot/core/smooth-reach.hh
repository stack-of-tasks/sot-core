/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SMOOTHREACH_H_H
#define __SOT_SMOOTHREACH_H_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(com_freezer_EXPORTS)
#define SOTSMOOTHREACH_EXPORT __declspec(dllexport)
#else
#define SOTSMOOTHREACH_EXPORT __declspec(dllimport)
#endif
#else
#define SOTSMOOTHREACH_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSMOOTHREACH_EXPORT SmoothReach : public dynamicgraph::Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName() const { return CLASS_NAME; }

 private:
  dynamicgraph::Vector start, goal;
  size_type startTime, lengthTime;
  bool isStarted, isParam;
  size_type smoothMode;
  double smoothParam;

  double smoothFunction(double x);

 public: /* --- CONSTRUCTION --- */
  SmoothReach(const std::string &name);
  virtual ~SmoothReach(void){};

 public: /* --- SIGNAL --- */
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> startSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, sigtime_t> goalSOUT;

 public: /* --- FUNCTION --- */
  dynamicgraph::Vector &goalSOUT_function(dynamicgraph::Vector &goal,
                                          const sigtime_t &time);

  void set(const dynamicgraph::Vector &goal, const std::int64_t &length);
  const dynamicgraph::Vector &getGoal(void);
  const size_type &getLength(void);
  const size_type &getStart(void);

  void setSmoothing(const std::int64_t &mode, const double &param);

 public: /* --- PARAMS --- */
  virtual void display(std::ostream &os) const;
  void initCommands(void);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_SMOOTHREACH_H_H */
