/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_Control_GR_HH__
#define __SOT_Control_GR_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(control_gr_EXPORTS)
#define ControlGR_EXPORT __declspec(dllexport)
#else
#define ControlGR_EXPORT __declspec(dllimport)
#endif
#else
#define ControlGR_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class ControlGR_EXPORT ControlGR : public Entity {

public: /* --- CONSTRUCTOR ---- */
  ControlGR(const std::string &name);

public: /* --- INIT --- */
  void init(const double &step);

public: /* --- CONSTANTS --- */
  /* Default values. */
  static const double TIME_STEP_DEFAULT; // = 0.001

public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

protected:
  /* Parameters of the torque-control function:
   * tau = - A*qddot = g */
  double TimeStep;
  double _dimension;

public: /* --- SIGNALS --- */
  SignalPtr<dg::Matrix, int> matrixASIN;
  SignalPtr<dg::Vector, int> accelerationSIN;
  SignalPtr<dg::Vector, int> gravitySIN;
  SignalTimeDependent<dg::Vector, int> controlSOUT;

protected:
  double &setsize(int dimension);
  dg::Vector &computeControl(dg::Vector &tau, int t);
};

} // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __SOT_Control_GR_HH__
