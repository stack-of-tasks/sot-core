/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_OP_POINT_MODIFIOR_H__
#define __SOT_OP_POINT_MODIFIOR_H__

#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include <sot/core/debug.hh>
#include <sot/core/matrix-geometry.hh>

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(op_point_modifier_EXPORTS)
#define SOTOPPOINTMODIFIER_EXPORT __declspec(dllexport)
#else
#define SOTOPPOINTMODIFIER_EXPORT __declspec(dllimport)
#endif
#else
#define SOTOPPOINTMODIFIER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

///
/// \brief Compute position and jacobian of a local frame attached to a joint.
///
/// The position of the local frame in the frame of the joint is represented by
/// transformation.
///
class SOTOPPOINTMODIFIER_EXPORT OpPointModifier : public dynamicgraph::Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

 public:
  dynamicgraph::SignalPtr<dynamicgraph::Matrix, int> jacobianSIN;
  dynamicgraph::SignalPtr<MatrixHomogeneous, int> positionSIN;

  dynamicgraph::SignalTimeDependent<dynamicgraph::Matrix, int> jacobianSOUT;
  dynamicgraph::SignalTimeDependent<MatrixHomogeneous, int> positionSOUT;

 public:
  OpPointModifier(const std::string &name);
  virtual ~OpPointModifier(void) {}

  dynamicgraph::Matrix &jacobianSOUT_function(dynamicgraph::Matrix &res,
                                              const int &time);
  MatrixHomogeneous &positionSOUT_function(MatrixHomogeneous &res,
                                           const int &time);
  void setTransformation(const Eigen::Matrix4d &tr);
  void setTransformationBySignalName(std::istringstream &cmdArgs);
  const Eigen::Matrix4d &getTransformation(void);

 private:
  MatrixHomogeneous transformation;

  /* This bool tunes the effect of the modifier for end-effector Jacobian (ie
   * the output velocity is expressed in the end-effector frame) of from the
   * world-ref Jacobian (ie
   * the ouput velocity is computed in the world frame). */
  bool isEndEffector;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  //  __SOT_OP_POINT_MODIFIOR_H__
