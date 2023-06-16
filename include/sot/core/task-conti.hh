/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_TASKCONTI_H__
#define __SOT_TASKCONTI_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* STD */
#include <string>

/* SOT */
#include <sot/core/exception-task.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/flags.hh>
#include <sot/core/task.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(task_conti_EXPORTS)
#define SOTTASKCONTI_EXPORT __declspec(dllexport)
#else
#define SOTTASKCONTI_EXPORT __declspec(dllimport)
#endif
#else
#define SOTTASKCONTI_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

class SOTTASKCONTI_EXPORT TaskConti : public Task {
 protected:
  enum TimeRefValues { TIME_REF_UNSIGNIFICANT = -1, TIME_REF_TO_BE_SET = -2 };

  sigtime_t timeRef;
  double mu;
  dynamicgraph::Vector q0;

 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

 public:
  TaskConti(const std::string &n);

  void referenceTime(const sigtime_t &t) { timeRef = t; }
  const sigtime_t &referenceTime(void) { return timeRef; }

  /* --- COMPUTATION --- */
  VectorMultiBound &computeContiDesiredVelocity(VectorMultiBound &task,
                                                const sigtime_t &time);

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> controlPrevSIN;

  /* --- DISPLAY ------------------------------------------------------------ */
  void display(std::ostream &os) const;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_TASKCONTI_H__ */
