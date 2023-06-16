/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_CLAMP_WORKSPACE_HH__
#define __SOT_CLAMP_WORKSPACE_HH__

/* STL */
#include <utility>

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include <sot/core/exception-task.hh>
#include <sot/core/matrix-geometry.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(clamp_workspace_EXPORTS)
#define SOTCLAMPWORKSPACE_EXPORT __declspec(dllexport)
#else
#define SOTCLAMPWORKSPACE_EXPORT __declspec(dllimport)
#endif
#else
#define SOTCLAMPWORKSPACE_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTCLAMPWORKSPACE_EXPORT ClampWorkspace : public dynamicgraph::Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dynamicgraph::SignalPtr<MatrixHomogeneous, sigtime_t> positionrefSIN;
  dynamicgraph::SignalPtr<MatrixHomogeneous, sigtime_t> positionSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Matrix, sigtime_t> alphaSOUT;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Matrix, sigtime_t> alphabarSOUT;
  dynamicgraph::SignalTimeDependent<MatrixHomogeneous, sigtime_t> handrefSOUT;

 public:
  ClampWorkspace(const std::string &name);
  virtual ~ClampWorkspace(void) {}

  void update(sigtime_t time);

  virtual dynamicgraph::Matrix &computeOutput(dynamicgraph::Matrix &res,
                                              sigtime_t time);
  virtual dynamicgraph::Matrix &computeOutputBar(dynamicgraph::Matrix &res,
                                                 sigtime_t time);
  virtual MatrixHomogeneous &computeRef(MatrixHomogeneous &res, sigtime_t time);

  virtual void display(std::ostream &) const;

 private:
  sigtime_t timeUpdate;

  dynamicgraph::Matrix alpha;
  dynamicgraph::Matrix alphabar;
  MatrixHomogeneous prefMp;
  dynamicgraph::Vector pd;
  MatrixRotation Rd;
  MatrixHomogeneous handref;

  double beta;
  double scale;
  double dm_min;
  double dm_max;
  double dm_min_yaw;
  double dm_max_yaw;
  double theta_min;
  double theta_max;
  int mode;

  enum { FRAME_POINT, FRAME_REF } frame;

  std::pair<double, double> bounds[3];
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif
