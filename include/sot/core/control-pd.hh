/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_Control_PD_HH__
#define __SOT_Control_PD_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(control_pd_EXPORTS)
#define ControlPD_EXPORT __declspec(dllexport)
#else
#define ControlPD_EXPORT __declspec(dllimport)
#endif
#else
#define ControlPD_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class ControlPD_EXPORT ControlPD : public Entity {
 public: /* --- CONSTRUCTOR ---- */
  ControlPD(const std::string &name);

 public: /* --- INIT --- */
  void init(const double &step);

 public: /* --- CONSTANTS --- */
  /* Default values. */
  static const double TIME_STEP_DEFAULT;  // = 0.001

 public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

 protected:
  /* Parameters of the torque-control function:
   * tau = kp * (qd-q) + kd* (dqd-dq) */
  double TimeStep;

 public: /* --- SIGNALS --- */
  SignalPtr<dynamicgraph::Vector, int> KpSIN;
  SignalPtr<dynamicgraph::Vector, int> KdSIN;
  SignalPtr<dynamicgraph::Vector, int> positionSIN;
  SignalPtr<dynamicgraph::Vector, int> desiredpositionSIN;
  SignalPtr<dynamicgraph::Vector, int> velocitySIN;
  SignalPtr<dynamicgraph::Vector, int> desiredvelocitySIN;
  SignalTimeDependent<dynamicgraph::Vector, int> controlSOUT;
  SignalTimeDependent<dynamicgraph::Vector, int> positionErrorSOUT;
  SignalTimeDependent<dynamicgraph::Vector, int> velocityErrorSOUT;

 protected:
  dynamicgraph::Vector &computeControl(dynamicgraph::Vector &tau, int t);
  dynamicgraph::Vector position_error_;
  dynamicgraph::Vector velocity_error_;
  dynamicgraph::Vector &getPositionError(dynamicgraph::Vector &position_error,
                                         int t);
  dynamicgraph::Vector &getVelocityError(dynamicgraph::Vector &velocity_error,
                                         int t);
};

}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __SOT_Control_PD_HH__
