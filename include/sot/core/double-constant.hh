/*
 * Copyright 2019,
 * Joseph Mirabel
 *
 * LAAS-CNRS
 *
 */

#ifndef DYNAMICGRAPH_SOT_DOUBLE_CONSTANT_H
#define DYNAMICGRAPH_SOT_DOUBLE_CONSTANT_H

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>

namespace dynamicgraph {
namespace sot {

class DoubleConstant : public Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  DoubleConstant(const std::string &name);

  virtual ~DoubleConstant(void) {}

  SignalTimeDependent<double, int> SOUT;

  /// \brief Set value of vector (and therefore of output signal)
  void setValue(const double &inValue);
};

}  // namespace sot
}  // namespace dynamicgraph

#endif  // DYNAMICGRAPH_SOT_DOUBLE_CONSTANT_H
