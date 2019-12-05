/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_CONSTRAINT_H__
#define __SOT_CONSTRAINT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* STD */
#include <string>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <sot/core/exception-signal.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/flags.hh>
#include <sot/core/task-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(constraint_EXPORTS)
#define SOTCONSTRAINT_EXPORT __declspec(dllexport)
#else
#define SOTCONSTRAINT_EXPORT __declspec(dllimport)
#endif
#else
#define SOTCONSTRAINT_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTCONSTRAINT_EXPORT Constraint : public TaskAbstract {
protected:
  typedef std::list<Signal<dg::Matrix, int> *> JacobianList;
  JacobianList jacobianList;

public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

public:
  Constraint(const std::string &n);

  void addJacobian(Signal<dg::Matrix, int> &sig);
  void clearJacobianList(void);

  void setControlSelection(const Flags &act);
  void addControlSelection(const Flags &act);
  void clearControlSelection(void);

  /* --- COMPUTATION --- */
  dg::Matrix &computeJacobian(dg::Matrix &J, int time);

  /* --- DISPLAY ------------------------------------------------------------ */
  SOTCONSTRAINT_EXPORT friend std::ostream &operator<<(std::ostream &os,
                                                       const Constraint &t);
};
} // namespace sot
} // namespace dynamicgraph

#endif /* #ifndef __SOT_CONSTRAINT_H__ */
